#include "drv8825.h"

#include <math.h>

#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/task.h"

#include "robot_config.h"

#define TAG "DRV8825"

#define LOW_STEP_SPEED 150
#define MED_STEP_SPEED 800
#define HIGH_STEP_SPEED 1000


#define SIN_FREQ_HZ 0.5

static esp_err_t drv8825_set_direction(drv8825_t *drv, uint8_t new_dir);


static int step_factor = 1;
/*Select microstepping*/
//factor 1 2 4 8 16 32 => FULL STEP, HALFD, QUARTER, EIGHTH, 1SIXTEENTH, 1THIRTY_SECOND
static void motor_microstep_config(uint8_t factor){
    step_factor = factor;

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

    gpio_set_level(ROBOT_DRV8825_1_GPIO_M0, m0);
    gpio_set_level(ROBOT_DRV8825_1_GPIO_M1, m1);
    gpio_set_level(ROBOT_DRV8825_1_GPIO_M2, m2);
}

//Update microstepping according to current speed
void update_microstepping(float abs_speed) {
    int new_microstep;

    if (abs_speed < LOW_STEP_SPEED) {
        new_microstep = 16;
    } else if (abs_speed < MED_STEP_SPEED) {
        new_microstep = 8;
    } else if (abs_speed < HIGH_STEP_SPEED){
        new_microstep = 4;
    } else {
        new_microstep = 1;
    }

    // Appliquer uniquement si changement ET vitesse suffisamment basse
    if (new_microstep != step_factor && abs_speed < (float)ROBOT_DRV8825_1_MAX_STEPS_SEC) {
       // ESP_LOGI(TAG, "Microstepping factor : %d", new_microstep);
        motor_microstep_config(new_microstep);
        step_factor = new_microstep;
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

// static struct timeval last_time;
static uint16_t count;

// Apply new period of the PWM according to step/s speed. It is modulated by the micro-stepping factor 
/// @param drv 
/// @param steps_sec 
void motor_driver_set_speed(drv8825_t* drv, float steps_sec){
    if (!drv ) {
        ESP_LOGI(TAG, "In motor driver set speed drv is null");
        return;
    }

    if(!drv->timer){
        ESP_LOGI(TAG, "In motor driver set speed drv->timer is null");
        return;
    }
    
    //Block the call to this function to 10ms
    // struct timeval now;
    // gettimeofday(&now, NULL);
    // float my_dt = (now.tv_sec - last_time.tv_sec)*1000.0f + (now.tv_usec - last_time.tv_usec) / 1000.0f;
    
    // if(my_dt<10.0f){
    //     return;
    // }
    // last_time = now;


    uint8_t requested_dir=0U;
    if(steps_sec<0.0f){
        steps_sec=-steps_sec;
        requested_dir=1U;}
    if((uint16_t)steps_sec>ROBOT_DRV8825_1_MAX_STEPS_SEC){steps_sec=ROBOT_DRV8825_1_MAX_STEPS_SEC;}
    if((uint16_t)steps_sec<ROBOT_DRV8825_1_MIN_STEPS_SEC){steps_sec=0.0f;}//Stop motor
   
    /*
     * Cas d'arrêt.
     */
    if(steps_sec==0.0f){
        if(drv->running){
            ESP_ERROR_CHECK(mcpwm_timer_start_stop(drv->timer, MCPWM_TIMER_STOP_FULL));
            drv->running = false;
        }
        return;
    }

    const float pulse_frequency =
        steps_sec * (float)step_factor;

    if (pulse_frequency <= 0.0f) {
        ESP_LOGE(TAG, "Invalid pulse frequency: %.2f Hz", pulse_frequency);
        return;
    }
    uint32_t period_ticks =(uint32_t)((float)ROBOT_DRV8825_1_PWM_RES / pulse_frequency);

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
    if (direction_change && drv->running) {
        esp_err_t err = mcpwm_timer_start_stop(drv->timer,MCPWM_TIMER_STOP_FULL);

        if (err != ESP_OK) {
            ESP_LOGE(TAG,"Failed to stop MCPWM timer before direction change: %s", esp_err_to_name(err));
            return;
        }
        drv->running = false;
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
    if (!drv->running) {
        err = mcpwm_timer_start_stop( drv->timer, MCPWM_TIMER_START_NO_STOP);

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start MCPWM timer: %s", esp_err_to_name(err) );
            return;
        }

        drv->running = true;
    }

    count++;

    if (count >= 600U) {
        count = 0U;
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

    
    esp_err_t err = gpio_set_level(drv->dir_pin, new_dir);

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
    drv8825_init(drv, 1000.0f, ROBOT_DRV8825_1_GPIO_STEP, ROBOT_DRV8825_1_GPIO_DIR, GPIO_NUM_NC,GPIO_NUM_NC);
    drv8825_start(drv);
  
    const float dt = 0.01; //every 10ms a new command is sent
    const TickType_t delay_ticks=pdMS_TO_TICKS((uint16_t)(dt));
    float t=0.0;
    float current_speed = 0.0f;
    uint8_t loop=0;
    float target_speed;
    while(1){
        target_speed= ROBOT_DRV8825_1_MAX_STEPS_SEC *sinf(2*M_PI*SIN_FREQ_HZ*t);
        current_speed=apply_accel_limit(current_speed, target_speed, drv->max_accel, dt);
        update_microstepping(fabs(current_speed));
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

void drv8825_task(void *pvParameters) {
    drv8825_t *drv = (drv8825_t*)pvParameters;

    while(drv->running){
          drv->current_speed = apply_accel_limit(drv->current_speed, drv->target_speed, drv->max_accel, 0.01);
        motor_driver_set_speed(drv, drv->current_speed);
    }
    vTaskDelete(NULL);

}

//API
esp_err_t drv8825_init(drv8825_t *drv, float max_accel, gpio_num_t step_pin, gpio_num_t dir_pin, gpio_num_t sleep, gpio_num_t enable){
    if (!drv) return ESP_ERR_INVALID_ARG;
    count=0;
    //Init struct
    drv->max_accel=max_accel;
    drv->step_pin=step_pin;
    drv->dir_pin=dir_pin;
    drv->sleep_pin=sleep;
    drv->enable_pin=enable;
    drv->current_speed=0.0f;
    drv->target_speed=0.0f;
    drv->running=false;
    drv->target_speed_mutex= xSemaphoreCreateMutex();
    drv->current_dir = 0U;
    drv->direction_initialized = false;

    
    //Config GPIO Step and Dir
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ROBOT_DRV8825_1_GPIO_M0) | (1ULL << ROBOT_DRV8825_1_GPIO_M1) | (1ULL << ROBOT_DRV8825_1_GPIO_M2) |
                        (1ULL << step_pin) | (1ULL << dir_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    //Init SLEEP optionnal
    if(sleep !=GPIO_NUM_NC){
                gpio_config_t io_conf_sleep = {
        .pin_bit_mask = (1ULL << sleep),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf_sleep);
    gpio_set_level(sleep, 0); //Sleep active by default
    }
    
    //Init ENABLE optionnal
    if(enable !=GPIO_NUM_NC){
        gpio_config_t io_conf_enable = {
            .pin_bit_mask = (1ULL << enable),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
    gpio_config(&io_conf_enable);
    gpio_set_level(enable, 0); //Enabled by default
    }

    //See MICROSTEP_MODE, possible values are : MICROSTEP_MODE_FULL, MICROSTEP_MODE_HALF, MICROSTEP_MODE_QUARTER, MICROSTEP_MODE_EIGTH,  
    motor_microstep_config(2);
    // Direction avant
    gpio_set_level( drv->dir_pin, 0);


    // Configuration du MCPWM
        //Timer
    mcpwm_timer_config_t timer_config = {
        .group_id = 0, //or 1 
        .intr_priority=0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = ROBOT_DRV8825_1_PWM_RES, // 10 MHz: tick = 100 ns
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
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(drv->comparator, 50)); // duty = 5 µs

        //Generator
    mcpwm_generator_config_t gen_config = {
        .gen_gpio_num = step_pin,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(drv->oper, &gen_config, &drv->generator));

    // Génère une impulsion HIGH au début, LOW après le compare
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(drv->generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_TIMER_EVENT_ACTION_END()));
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(drv->generator, 
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, drv->comparator, MCPWM_GEN_ACTION_LOW), 
        MCPWM_GEN_COMPARE_EVENT_ACTION_END()));

        // Activer le timer
    ESP_ERROR_CHECK(mcpwm_timer_enable(drv->timer));


    // ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_FULL));
    // ESP_ERROR_CHECK(mcpwm_timer_disable(timer));
    ESP_LOGI(TAG, "DRV8825 initialized.");
    return ESP_OK;
}

esp_err_t drv8825_start(drv8825_t *drv){
    if(!drv || drv->running) return ESP_ERR_INVALID_STATE;
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(drv->timer, MCPWM_TIMER_START_NO_STOP));
    drv->running=true;
    // BaseType_t ret = xTaskCreate(drv8825_task, "driver_task", 4096, drv, 5, drv->motor_task_handle);
    //     if (ret != pdPASS) {
    //     drv->running = false;
    //     return ESP_FAIL;
    // }
    return ESP_OK;
}

esp_err_t drv8825_stop(drv8825_t *drv){
     if (!drv || !drv->running) return ESP_OK;

    drv->running = false;

    xSemaphoreTake(drv->target_speed_mutex, portMAX_DELAY);
    drv->target_speed = 0.0f;
    drv->current_speed = 0.0f;
    xSemaphoreGive(drv->target_speed_mutex);
    motor_driver_set_speed(drv, 0.0f);

    ESP_ERROR_CHECK(mcpwm_timer_start_stop(drv->timer, MCPWM_TIMER_STOP_FULL));

    ESP_LOGI(TAG,"DRV8825 Stopped.");

    return ESP_OK;
}

esp_err_t drv8825_deinit(drv8825_t* drv){
        if (!drv) return ESP_ERR_INVALID_ARG;


    // Sécurité : on stoppe tout si ce n’est pas déjà fait
    drv8825_stop(drv);

    //Disable the timer 
    ESP_ERROR_CHECK(mcpwm_timer_disable(drv->timer));

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
        if (!drv || drv->sleep_pin == GPIO_NUM_NC) return ESP_ERR_INVALID_STATE;
    gpio_set_level(drv->sleep_pin, enable ? 0 : 1); // SLEEP pin : 0 = sleep active 
    return ESP_OK;
}

esp_err_t drv8825_enable(drv8825_t* drv, bool enable){
        if (!drv || drv->enable_pin == GPIO_NUM_NC) return ESP_ERR_INVALID_STATE;
    gpio_set_level(drv->enable_pin, enable ? 0 : 1); // EN pin : 0 = enabled
    return ESP_OK;
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


    //if (!drv || !drv->running) return;

    float target_speed;
    xSemaphoreTake(drv->target_speed_mutex, portMAX_DELAY);
    target_speed=drv->target_speed;
    xSemaphoreGive(drv->target_speed_mutex);
    drv->current_speed = apply_accel_limit(drv->current_speed, target_speed, drv->max_accel, dt);
    //drv->current_speed=target_speed;
    //update_microstepping(fabs(drv->current_speed));
    motor_driver_set_speed(drv, drv->current_speed);
}



bool drv8825_is_running(drv8825_t* drv){
    if(!drv) return false;
    return drv->running;
}


