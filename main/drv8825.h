#ifndef DRV8825_H
#define DRV8825_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

typedef enum {
    FULL_STEP        = 1,
    HALF_STEP        = 2,
    QUARTER_STEP     = 4,
    EIGHTH_STEP      = 8,
    SIXTEENTH_STEP   = 16,
    THIRTY_SECOND    = 32
} drv8825_microstep_t;

typedef struct {
    gpio_num_t step_pin;
    gpio_num_t dir_pin;

    //optionnal for futur use
    gpio_num_t sleep_pin;
    gpio_num_t enable_pin;

    //Microstepping pins
    gpio_num_t m0;
    gpio_num_t m1;
    gpio_num_t m2;

    uint16_t max_step_sec;
    uint16_t min_step_sec;

    drv8825_microstep_t mode;
    float max_accel;

    

} drv8825_config_t;


typedef struct{
    
    float current_speed;
    float target_speed;
    uint8_t current_dir;
    drv8825_microstep_t current_microstep_factor;

    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t oper;
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t generator;

    SemaphoreHandle_t target_speed_mutex;
   
    bool direction_initialized;
    bool started;
    bool step_pulse_active; 

    drv8825_config_t config;

} drv8825_t;


//Init Driver step and dir pin are mandatory, sleep and enable are optionals set to GPIO_UNUSED
esp_err_t drv8825_init(drv8825_t *drv, const drv8825_config_t *config);
esp_err_t drv8825_start(drv8825_t *drv);
esp_err_t drv8825_stop(drv8825_t *drv);
esp_err_t drv8825_deinit(drv8825_t* drv);
esp_err_t drv8825_sleep(drv8825_t* drv, bool enable);
esp_err_t drv8825_enable(drv8825_t* drv, bool enable);
void drv8825_set_target_speed(drv8825_t *drv, float speed);
esp_err_t drv8825_update(drv8825_t* drv, float dt);
bool drv8825_is_running(drv8825_t* drv); //OK


//TODO Only to test motor control
void drv8825_sine_task(void *pvParameters);


#endif