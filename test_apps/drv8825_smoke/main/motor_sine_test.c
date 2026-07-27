#include "driver/gpio.h"

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

void app_main(void)
{
    ESP_LOGI(TAG, "Booting...");

    //motor driver testing
    drv8825_t mydrv={0};
    xTaskCreate(drv8825_sine_task, "drv8825", 4096 ,&mydrv, 10, NULL);


}
