#include "drv8825.h"

#include <math.h>

#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/task.h"

// #include "robot_config.h"

#define TAG "DRV8825"

#define LOW_STEP_SPEED 150
#define MED_STEP_SPEED 800
#define HIGH_STEP_SPEED 1000

#define FIXED_STEP_HIGH_TIME 5 //micro seconds
#define FIXED_STEP_LOW_TIME 2 //micro seconds
#define PWM_RES (1U*1000U*1000U)
#define SIN_FREQ_HZ 0.5

static void motor_microstep_config(drv8825_t *drv, drv8825_microstep_t factor);
static void update_microstepping(drv8825_t *drv, float abs_speed);
static float apply_accel_limit(float current, float target, float max_accel, float dt);
static void motor_driver_set_speed(drv8825_t* drv, int16_t steps_sec);

static esp_err_t drv8825_set_direction(drv8825_t *drv, uint8_t new_dir);


/*Select microstepping*/
//factor 1 2 4 8 16 32 => FULL STEP, HALFD, QUARTER, EIGHTH, 1SIXTEENTH, 1THIRTY_SECOND
static void motor_microstep_config(drv8825_t *drv, drv8825_microstep_t factor){

    int m0 = 0, m1 = 0, m2 = 0;
    switch (factor) {
        case 1:  m0 = 0; m1 = 0; m2 = 0; break; //FULL
        case 2:  m0 = 1; m1 = 0; m2 = 0; break; //HALF
        case 4:  m0 = 0; m1 = 1; m2 = 0; break;//QUARTER
        case 8:  m0 = 1; m1 = 1; m2 = 0; break; //EIGTHS
        case 16: m0 = 0; m1 = 0; m2 = 1; break; //SIXTEENTH
        case 32: m0 = 1; m1 = 0; m2 = 1; break;//1/32 steps
        default: break;
    }
    drv->current_microstep_factor=factor;

    gpio_set_level(drv->config.m0, m0);
    gpio_set_level(drv->config.m1, m1);
    gpio_set_level(drv->config.m2, m2);
}

//Update microstepping according to current speed
static void update_microstepping(drv8825_t *drv, float abs_speed) {
    drv8825_microstep_t new_microstep;

    if (abs_speed < LOW_STEP_SPEED) {
        new_microstep = SIXTEENTH_STEP;
    } else if (abs_speed < MED_STEP_SPEED) {
        new_microstep = EIGHTH_STEP;
    } else if (abs_speed < HIGH_STEP_SPEED){
        new_microstep = QUARTER_STEP;
    } else {
        new_microstep = FULL_STEP;
    }

    // Appliquer uniquement si changement ET vitesse suffisamment basse
    if (new_microstep != drv->current_microstep_factor && abs_speed < (float)drv->config.max_step_sec) {
       // ESP_LOGI(TAG, "Microstepping factor : %d", new_microstep);
        motor_microstep_config(drv, new_microstep);
    }
}


//Apply acceleration limits
static float apply_accel_limit(float current, float target, float max_accel, float dt)
{
    float delta = target - current;
    float max_delta = max_accel * dt;
    if (delta > max_delta) delta = max_delta;
    if (delta < -max_delta) delta = -max_delta;
    return current + delta;
}


// Apply new period of the PWM according to step/s speed. It is modulated by the micro-stepping factor 
/// @param drv 
/// @param steps_sec 
static void motor_driver_set_speed(drv8825_t* drv, int16_t steps_sec){
    if (!drv ) {
        ESP_LOGI(TAG, "In motor driver set speed drv is null");
        return;
    }

    if(!drv->timer){
        ESP_LOGI(TAG, "In motor driver set speed drv->timer is null");
        return;
    }
    
    uint8_t requested_dir=0U;
    if(steps_sec<0){
        steps_sec=-steps_sec;
        requested_dir=1U;}
    if((uint16_t)steps_sec>drv->config.max_step_sec){steps_sec=drv->config.max_step_sec;}
    if((uint16_t)steps_sec<drv->config.min_step_sec){steps_sec=0.0f;}//Stop motor
   
    /*
     * Cas d'arrêt.
     */
    if(steps_sec==0.0f){
        if(drv->step_pulse_active){
            ESP_ERROR_CHECK(mcpwm_timer_start_stop(drv->timer, MCPWM_TIMER_STOP_FULL));
            // ESP_ERROR_CHECK(drv8825_stop(drv));
            drv->step_pulse_active = false;
        }
        return;
    }

    const float pulse_frequency =
        steps_sec * (float)drv->current_microstep_factor;

    if (pulse_frequency <= 0.0f) {
        ESP_LOGE(TAG, "Invalid pulse frequency: %.2f Hz", pulse_frequency);
        return;
    }
    uint32_t period_ticks =(uint32_t)((float)PWM_RES / pulse_frequency);

    /*
     * Éviter une période MCPWM nulle si la fréquence devient
     * supérieure à la résolution du timer.
     */
    if (period_ticks == 0U) {period_ticks = 1U;}
    /*
     * Vérifier si un changement de direction est nécessaire.
     */
    bool direction_change =
        !drv->direction_initialized ||
        drv->current_dir != requested_dir;

    /*
     * Si le moteur tourne, arrêter temporairement STEP avant
     * de modifier DIR.
     */
    if (direction_change && drv->step_pulse_active) {
        esp_err_t err = mcpwm_timer_start_stop(drv->timer,MCPWM_TIMER_STOP_FULL);

        if (err != ESP_OK) {
            ESP_LOGE(TAG,"Failed to stop MCPWM timer before direction change: %s", esp_err_to_name(err));
            return;
        }
        drv->step_pulse_active = false;
    }

    /*
     * Changer la direction lorsque nécessaire.
     *
     * Le timer est arrêté à ce moment si le moteur tournait,
     * ce qui évite qu'un front STEP apparaisse pendant
     * la transition de DIR.
     */
    if (direction_change) {
        esp_err_t err = drv8825_set_direction( drv, requested_dir );
        if (err != ESP_OK) {
            return;
        }
    }

    /*
     * Configurer la nouvelle période avant de démarrer
     * ou redémarrer le timer.
     */
    esp_err_t err = mcpwm_timer_set_period(drv->timer, period_ticks);

    if (err != ESP_OK) {
        ESP_LOGE( TAG, "Failed to set MCPWM period to %lu ticks: %s", (unsigned long)period_ticks, esp_err_to_name(err));
        return;
    }

    /*
     * Démarrer le timer uniquement une fois que :
     * - la direction est correcte ;
     * - le délai de garde est écoulé ;
     * - la période est configurée.
     */
    if (!drv->step_pulse_active) {
        err = mcpwm_timer_start_stop( drv->timer, MCPWM_TIMER_START_NO_STOP);

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start MCPWM timer: %s", esp_err_to_name(err) );
            return;
        }

        drv->step_pulse_active = true;
    }
}


static esp_err_t drv8825_set_direction(
    drv8825_t *drv,
    uint8_t new_dir
) {
    if (!drv) {
        return ESP_ERR_INVALID_ARG;
    }

    new_dir = new_dir ? 1 : 0;

    if (
        drv->direction_initialized &&
        drv->current_dir == new_dir
    ) {
        return ESP_OK;
    }

    
    esp_err_t err = gpio_set_level(drv->config.dir_pin, new_dir);

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Failed to set motor direction to %u: %s",
            new_dir,
            esp_err_to_name(err)
        );

        return err;
    }

    /*
     * Temps de garde entre la modification de DIR
     * et le prochain front STEP.
     */
    esp_rom_delay_us(2);

    drv->current_dir = new_dir;
    drv->direction_initialized = true;

    return ESP_OK;
}
//TODO : TO BE DELETED LATER
void drv8825_sine_task(void *pvParameters) {
    drv8825_t *drv = (drv8825_t*)pvParameters;
    drv8825_config_t init_config ={
        GPIO_NUM_1,
        GPIO_NUM_4,
        GPIO_NUM_NC,
        GPIO_NUM_NC,
        GPIO_NUM_11,
        GPIO_NUM_10,
        GPIO_NUM_18,
        2000,
        50,
        HALF_STEP,
        1000.0f
    };

    drv8825_init(drv, &init_config);
    drv8825_start(drv);
  
    const float dt = 0.01; //every 10ms a new command is sent
    const TickType_t delay_ticks=pdMS_TO_TICKS((uint16_t)(dt));
    float t=0.0;
    float current_speed = 0.0f;
    uint8_t loop=0;
    float target_speed;
    while(1){
        target_speed= 2000 *sinf(2*M_PI*SIN_FREQ_HZ*t);
        current_speed=apply_accel_limit(current_speed, target_speed, drv->config.max_accel, dt);
        update_microstepping(drv, fabs(current_speed));
        ESP_LOGI(TAG, "Speed set to %f", current_speed);

        //}
        motor_driver_set_speed(drv, current_speed);
        if (loop % 20 == 0) {
        ESP_LOGI(TAG, "Target: %.1f | Current: %.1f", target_speed, current_speed);
        loop = 0;
    }
        vTaskDelay(delay_ticks);
        t+=dt;
        loop+=1;

    }
}

static bool microstep_is_valid(drv8825_microstep_t microstep)
{
    switch (microstep) {
        case FULL_STEP:
        case HALF_STEP:
        case QUARTER_STEP:
        case EIGHTH_STEP:
        case SIXTEENTH_STEP:
        case THIRTY_SECOND:
            return true;

        default:
            return false;
    }
}

bool drv8825_validate_config(const drv8825_config_t *config){
    
    if(!GPIO_IS_VALID_OUTPUT_GPIO(config->step_pin)) return false;
    if(!GPIO_IS_VALID_OUTPUT_GPIO(config->dir_pin)) return false;
    if(!(config->sleep_pin==GPIO_NUM_NC ||GPIO_IS_VALID_OUTPUT_GPIO(config->sleep_pin))) return false;
    if(!(config->enable_pin==GPIO_NUM_NC || GPIO_IS_VALID_OUTPUT_GPIO(config->enable_pin))) return false;
    if(!GPIO_IS_VALID_OUTPUT_GPIO(config->m0)) return false;
    if(!GPIO_IS_VALID_OUTPUT_GPIO(config->m1)) return false;
    if(!GPIO_IS_VALID_OUTPUT_GPIO(config->m2)) return false;
    if(!config->max_step_sec) return false;
    if(!(config->min_step_sec<config->max_step_sec && config->min_step_sec>0)) return false;
    if(!config->mode) return false;
    if(!microstep_is_valid(config->mode)) return false;
    if(config->max_accel<=0 || !isfinite(config->max_accel)) return false;
    uint32_t slowest_period = PWM_RES/(config->min_step_sec*config->mode);
    uint32_t fastest_period = PWM_RES/(config->max_step_sec*config->mode);
    if(slowest_period>UINT16_MAX) {
        ESP_LOGE(TAG, "Slowest period value is %lu. For PWM Resolution %u and Minimum steps per seconds of %u and step factor of %u. Maximum allowed value is 65'535", slowest_period, PWM_RES, config->min_step_sec, config->mode);
        return false;
    }
    if(fastest_period<FIXED_STEP_HIGH_TIME+FIXED_STEP_LOW_TIME){
        ESP_LOGE(TAG, "Fastest period value is %lu. For PWM Resolution %u and Maximum steps per seconds of %u and step factor of %u. Minimum allowed value is %u", fastest_period, PWM_RES, config->max_step_sec, config->mode, FIXED_STEP_HIGH_TIME+FIXED_STEP_LOW_TIME);
        return false;
    }
    return true;
}

//API
esp_err_t drv8825_init(drv8825_t *drv, const drv8825_config_t *config){
    if (!config || !drv) return ESP_ERR_INVALID_ARG;
    //Init struct
    drv->config=*config;
    if(!drv8825_validate_config(config)){
        ESP_LOGE(TAG, "Validation of driver config failed");
        return ESP_ERR_INVALID_ARG;
    }
    drv->current_speed=0.0f;
    drv->target_speed=0.0f;
    drv->current_dir = 0U;
    drv->target_speed_mutex= xSemaphoreCreateMutex();

    drv->direction_initialized = false;

    
    //Config GPIO Step and Dir
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->m0) | (1ULL << config->m1) | (1ULL << config->m2) |
                        (1ULL << config->step_pin) | (1ULL << config->dir_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    //Init SLEEP optionnal
    if(config->sleep_pin !=GPIO_NUM_NC){
                gpio_config_t io_conf_sleep = {
        .pin_bit_mask = (1ULL << config->sleep_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf_sleep);
    gpio_set_level(config->sleep_pin, 0); //Sleep active by default
    }
    
    //Init ENABLE optionnal
    if(config->enable_pin !=GPIO_NUM_NC){
        gpio_config_t io_conf_enable = {
            .pin_bit_mask = (1ULL << config->enable_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
    gpio_config(&io_conf_enable);
    gpio_set_level(config->enable_pin, 0); //Enabled by default
    }

    //See MICROSTEP_MODE, possible values are : MICROSTEP_MODE_FULL, MICROSTEP_MODE_HALF, MICROSTEP_MODE_QUARTER, MICROSTEP_MODE_EIGTH,  
    motor_microstep_config(drv, config->mode);
    // Direction avant
    gpio_set_level( config->dir_pin, 0);


    // Configuration du MCPWM
        //Timer
    mcpwm_timer_config_t timer_config = {
        .group_id = 0, //or 1 
        .intr_priority=0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = PWM_RES, // 10 MHz: tick = 100 ns
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = 20000, // (20000 -> 20ms -> 50Hz ) period
        .flags.update_period_on_empty=1,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &drv->timer));

        //Operator
    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &drv->oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(drv->oper, drv->timer));
        
        //Comparator
    mcpwm_comparator_config_t comparator_config = {
        .intr_priority=0,
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(drv->oper, &comparator_config, &drv->comparator));
    uint32_t high_step_duration = PWM_RES*FIXED_STEP_HIGH_TIME/1000000;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(drv->comparator, high_step_duration)); // duty = 5 µs

        //Generator
    mcpwm_generator_config_t gen_config = {
        .gen_gpio_num = config->step_pin,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(drv->oper, &gen_config, &drv->generator));

    // Génère une impulsion HIGH au début, LOW après le compare
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(drv->generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_TIMER_EVENT_ACTION_END()));
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(drv->generator, 
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, drv->comparator, MCPWM_GEN_ACTION_LOW), 
        MCPWM_GEN_COMPARE_EVENT_ACTION_END()));



    // ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_FULL));
    // ESP_ERROR_CHECK(mcpwm_timer_disable(timer));
    drv->started=false;
    drv->step_pulse_active=false;
    ESP_LOGI(TAG, "DRV8825 initialized.");
    return ESP_OK;
}

esp_err_t drv8825_start(drv8825_t *drv){
    if(!drv || drv->started) return ESP_ERR_INVALID_STATE;
    ESP_ERROR_CHECK( mcpwm_timer_enable(drv->timer));
    drv->started=true;

    return ESP_OK;
}

esp_err_t drv8825_stop(drv8825_t *drv){
     if (!drv || !drv->started) return ESP_OK;

    

    xSemaphoreTake(drv->target_speed_mutex, portMAX_DELAY);
    drv->target_speed = 0.0f;
    drv->current_speed = 0.0f;
    xSemaphoreGive(drv->target_speed_mutex);
    motor_driver_set_speed(drv, 0.0f);

    ESP_ERROR_CHECK(mcpwm_timer_disable(drv->timer));
    drv->started = false;
    drv->step_pulse_active=false;
    ESP_LOGI(TAG,"DRV8825 Stopped.");

    return ESP_OK;
}

esp_err_t drv8825_deinit(drv8825_t* drv){
    if (!drv) return ESP_ERR_INVALID_ARG;

    // Sécurité : on stoppe tout si ce n’est pas déjà fait
    drv8825_stop(drv);

    // Libération MCPWM
    if (drv->generator) {
        ESP_ERROR_CHECK(mcpwm_del_generator(drv->generator));
        drv->generator = NULL;
    }
    if (drv->comparator) {
        ESP_ERROR_CHECK(mcpwm_del_comparator(drv->comparator));
        drv->comparator = NULL;
    }
    if (drv->oper) {
        ESP_ERROR_CHECK(mcpwm_del_operator(drv->oper));
        drv->oper = NULL;
    }
    if (drv->timer) {
        ESP_ERROR_CHECK(mcpwm_del_timer(drv->timer));
        drv->timer = NULL;
    }
    if (drv->target_speed_mutex) {
        vSemaphoreDelete(drv->target_speed_mutex);
        drv->target_speed_mutex = NULL;
    }   

    ESP_LOGI(TAG, "DRV8825 deinit done.");

    return ESP_OK;
}


esp_err_t drv8825_sleep(drv8825_t* drv, bool enable){
        if (!drv || drv->config.sleep_pin == GPIO_NUM_NC) return ESP_ERR_INVALID_STATE;
    gpio_set_level(drv->config.sleep_pin, enable ? 0 : 1); // SLEEP pin : 0 = sleep active 
    return ESP_OK;
}

esp_err_t drv8825_enable(drv8825_t* drv, bool enable){
        if (!drv || drv->config.enable_pin == GPIO_NUM_NC) return ESP_ERR_INVALID_STATE;
    gpio_set_level(drv->config.enable_pin, enable ? 0 : 1); // EN pin : 0 = enabled
    return ESP_OK;
}

void drv8825_set_target_speed(drv8825_t *drv, float speed){
    if (!drv) {
        ESP_LOGI(TAG, "In set_target_speed : Driver is null");
        return;}
    xSemaphoreTake(drv->target_speed_mutex, portMAX_DELAY);
    drv->target_speed = speed;
    xSemaphoreGive(drv->target_speed_mutex);
}

void drv8825_update(drv8825_t* drv, float dt) {

    if (!drv ) {
        ESP_LOGI(TAG, "In driver update, drv is null");
        return;
    }

    if(!drv->started){
        ESP_LOGI(TAG, "In driver update, drv is not started");
        return;
    }

    float target_speed;
    xSemaphoreTake(drv->target_speed_mutex, portMAX_DELAY);
    target_speed=drv->target_speed;
    xSemaphoreGive(drv->target_speed_mutex);
    drv->current_speed = apply_accel_limit(drv->current_speed, target_speed, drv->config.max_accel, dt);
    //drv->current_speed=target_speed;
    //update_microstepping(fabs(drv->current_speed));
    motor_driver_set_speed(drv, drv->current_speed);
}



bool drv8825_is_running(drv8825_t* drv){
    if(!drv) return false;
    return drv->step_pulse_active;
}


