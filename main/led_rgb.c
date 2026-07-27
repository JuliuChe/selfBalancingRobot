#include "led_rgb.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sdkconfig.h"


#define BLINK_GPIO CONFIG_BLINK_GPIO

static const char *TAG = "LED_RGB";
// static led_strip_handle_t led_strip;
// static uint8_t s_led_state = 0;
// QueueHandle_t led_rgb_queue = NULL;
// volatile bool led_task_running=false;

void led_rgb_init(led_rgb_t* leds) {
    ESP_LOGI(TAG, "Initializing LED blinker...");
 
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
    };

#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &leds->led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "Unsupported LED backend"
#endif

led_strip_clear(leds->led_strip);
leds->s_led_state =0;
leds->led_rgb_queue = NULL;
leds->running=false;
}


static void blink_led_interruptible(led_rgb_t* leds, rgb_t* my_col)
{
    if (leds->s_led_state) {
        uint8_t step=5;
        uint8_t red=0;
        uint8_t green=0;
        uint8_t blue=0;

        while(red < my_col->red || green < my_col->green || blue < my_col->blue){

            if (red < my_col->red) red += step;
            if (green < my_col->green) green += step;
            if (blue < my_col->blue) blue += step;

            led_strip_set_pixel(leds->led_strip, 0, red, green, blue);
            esp_err_t err = led_strip_refresh(leds->led_strip);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Error refreshing LED strip: %s", esp_err_to_name(err));
            }

            vTaskDelay(pdMS_TO_TICKS(20));
        }
    } else {
        led_strip_clear(leds->led_strip);
    }
}


void led_rgb_task(void *pvParameters) {
    led_rgb_t* leds=(led_rgb_t *)pvParameters;
    
    leds->led_task=xTaskGetCurrentTaskHandle();

    ESP_LOGI(TAG, "Starting LED RGB task...");
    leds->running = true; // Set the running flag to true
    if(leds->led_rgb_queue == NULL) {
        ESP_LOGE(TAG, "led_rgb_queue is NULL");
    }
    
    rgb_t new_col = {0, 0, 0};
    while (leds->running) {
        if(xQueueReceive(leds->led_rgb_queue, &new_col, 10)){
            ESP_LOGI(TAG, "Received new color: R=%d, G=%d, B=%d", new_col.red, new_col.green, new_col.blue);

        };
        blink_led_interruptible(leds, &new_col);    
        leds->s_led_state = !leds->s_led_state;
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
    }
    led_strip_clear(leds->led_strip); // Clear the strip when the task ends
    led_strip_del(leds->led_strip); // Free the strip resources
    vQueueDelete(leds->led_rgb_queue);
    leds->led_rgb_queue=NULL;
    vTaskDelete(NULL); // Delete the task when done
}


