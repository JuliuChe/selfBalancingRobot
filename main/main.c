/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sdkconfig.h"


#include "drv8825.h"
#include "driver/gpio.h"

#include "controller.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "Booting...");

    //motor driver testing
    // drv8825_t mydrv={0};
    // xTaskCreate(drv8825_sine_task, "drv8825", 4096,&mydrv, 10, NULL);
    xTaskCreate(controller_task, "controller_task", 4096, NULL, 10, NULL);

    // xTaskCreate(drv8825_task, "drv8825_task", 2048, NULL, 5, NULL);
}
