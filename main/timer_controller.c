#include "timer_controller.h"
#include <string.h>


static const char* TAG = "TIMER";

static TimerHandle_t timers[MAX_TIMERS] = { NULL };

static void timer_callback(TimerHandle_t xTimer) {

    timer_event_context_t *ctx = (timer_event_context_t *) pvTimerGetTimerID(xTimer);

    if (ctx != NULL) {
        ESP_LOGI(TAG, "Sending event from timer controller: %s", ctrl_event_to_str(ctx->event_to_send.type));
        xQueueSend(ctx->timer_event_queue, &ctx->event_to_send, 0);
    }

    free(ctx);

    for (int j = 0; j < MAX_TIMERS; j++) {
        if (timers[j] == xTimer) {
            timers[j] = NULL;
            break;
        }
    }
    xTimerDelete(xTimer, 0);
}

esp_err_t timer_ctrl_start(timer_event_context_t ctx, uint32_t timeout_ms, TimerHandle_t* out_timer) {
    if (!ctx.timer_event_queue) return ESP_ERR_INVALID_ARG;


    for (int i = 0; i < MAX_TIMERS; i++) {
        if (timers[i] == NULL) {
            timer_event_context_t* msg = malloc(sizeof(timer_event_context_t));
            if (!msg) return ESP_ERR_NO_MEM;
            memcpy(msg, &ctx, sizeof(timer_event_context_t));

            TimerHandle_t timer = xTimerCreate("timer_ctrl__t", pdMS_TO_TICKS(timeout_ms), pdFALSE, msg, timer_callback);
            if (!timer) {
                free(msg);
                return ESP_FAIL;
            }

            timers[i] = timer;
            if(xTimerStart(timer, 0)!=pdPASS){
                xTimerDelete(timer, 0);
                free(msg);
                timers[i] = NULL;
                return ESP_FAIL;
            } else {
                if(out_timer) *out_timer=timer;
                return ESP_OK;
            }
        }
    }
    ESP_LOGW(TAG, "No available autonomous timer slot");
    return ESP_ERR_NO_MEM;
}

esp_err_t timer_ctrl_cancel(TimerHandle_t timer) {
    if (!timer) return ESP_ERR_INVALID_ARG;

    for (int i = 0; i < MAX_TIMERS; i++) {
        if (timers[i] == timer) {
            timers[i] = NULL;
            xTimerStop(timer, 0);
            timer_event_context_t* ev = (timer_event_context_t*) pvTimerGetTimerID(timer);
            free(ev);
            xTimerDelete(timer, 0);
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

esp_err_t timer_ctrl_stop_all() {
    
    for (int i = 0; i < MAX_TIMERS; i++) {
        if (timers[i] != NULL) {
            xTimerStop(timers[i], 0);
            timer_event_context_t* ev = (timer_event_context_t*) pvTimerGetTimerID(timers[i]);
            free(ev);
            xTimerDelete(timers[i], 0);
            timers[i] = NULL;
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}