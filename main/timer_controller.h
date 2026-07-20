
#ifndef TIMER_CONTROLLER
#define TIMER_CONTROLLER
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_err.h"
#include "freertos/timers.h"
#include "controller.h"

#include "esp_log.h"

#define MAX_TIMERS 10  // Choisis un maximum arbitraire

esp_err_t timer_ctrl_start(controller_timer_context_t ctx, uint32_t timeout_ms, TimerHandle_t* out_timer);
esp_err_t timer_ctrl_cancel(TimerHandle_t timer);
esp_err_t timer_ctrl_stop_all();

#endif