#ifndef LED_RGB_H
#define LED_RGB_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "led_strip.h"

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} rgb_t;

typedef struct {
    TaskHandle_t led_task;
    led_strip_handle_t led_strip;
    uint8_t s_led_state;
    QueueHandle_t led_rgb_queue;
    volatile bool running;
} led_rgb_t;


void led_rgb_init(led_rgb_t* leds);
void led_rgb_task(void *pvParameters);

#endif // LED_RGB_H
