#ifndef DRV8825_H
#define DRV8825_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"


void update_microstepping(float abs_speed);

//New driver



typedef struct{
    
    float max_accel;
    float current_speed;
    float target_speed;


    volatile bool running;

    gpio_num_t step_pin;
    gpio_num_t dir_pin;

    //optionnal for futur use
    gpio_num_t sleep_pin;
    gpio_num_t enable_pin;

    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t oper;
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t generator;

    SemaphoreHandle_t target_speed_mutex;

    uint8_t current_dir;
    bool direction_initialized;

} drv8825_t;

//Init Driver step and dir pin are mandatory, sleep and enable are optionals set to GPIO_UNUSED
esp_err_t drv8825_init(drv8825_t *drv, float max_accel, gpio_num_t step_pin,  gpio_num_t dir_pin, gpio_num_t sleep, gpio_num_t enable);
esp_err_t drv8825_start(drv8825_t *drv);
esp_err_t drv8825_stop(drv8825_t *drv);
esp_err_t drv8825_deinit(drv8825_t* drv);
esp_err_t drv8825_sleep(drv8825_t* drv, bool enable);
esp_err_t drv8825_enable(drv8825_t* drv, bool enable);
void drv8825_set_target_speed(drv8825_t *drv, float speed);
void drv8825_update(drv8825_t* drv, float dt);
bool drv8825_is_running(drv8825_t* drv); //OK


//TODO Only to test motor control
void drv8825_sine_task(void *pvParameters);


#endif