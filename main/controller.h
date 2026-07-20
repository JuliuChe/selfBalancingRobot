#ifndef CONTROLLER
#define CONTROLLER


#include "mpu_reader.h"
#include "pid.h"
#include "led_rgb.h"
#include "drv8825.h"

typedef enum {
    EV_NONE = 0,
    EV_INIT,
    EV_INIT_DONE,
    EV_CONFIG_DONE,
    EV_CALIB_DONE,
    EV_MPU_STARTED,
    EV_MPU_DATA_READY,
    EV_ROBOT_LYING,
    EV_START_BALANCE,
    EV_BALANCE_READY,
    EV_STOP_BALANCING,
    EV_LOST_MPU,
    EV_ERROR,
    EV_STOP,
    // ... ajoute tes propres events ici
} ctrl_event_t;

static inline const char* ctrl_event_to_str(ctrl_event_t event) {
    switch(event) {
        case EV_NONE:           return "EV_NONE";
        case EV_INIT:           return "EV_INIT";
        case EV_INIT_DONE:      return "EV_INIT_DONE";
        case EV_CONFIG_DONE:    return "EV_CONFIG_DONE";
        case EV_CALIB_DONE:     return "EV_CALIB_DONE";
        case EV_MPU_STARTED:    return "EV_MPU_STARTED";
        case EV_ROBOT_LYING:    return "EV_ROBOT_LYING";
        case EV_MPU_DATA_READY: return "EV_MPU_DATA_READY";
        case EV_START_BALANCE:  return "EV_START_BALANCE";
        case EV_BALANCE_READY:  return "EV_BALANCE_READY";
        case EV_STOP_BALANCING: return "EV_STOP_BALANCING";
        case EV_LOST_MPU:       return "EV_LOST_MPU";
        case EV_STOP:           return "EV_STOP";
        case EV_ERROR:          return "EV_ERROR";
        // ... autres events
        default:               return "EV_UNKNOWN";
    }
};

typedef struct {
    ctrl_event_t type;         // Type d’événement
    esp_err_t err_code;        // Code d’erreur éventuel
    const char* msg;
    // Optionnel : timestamp, meta, etc.
} ctrl_event_msg_t;

typedef struct{
    QueueHandle_t timer_event_queue;
    ctrl_event_msg_t event_to_send;
} controller_timer_context_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float target_angle;
    float max_output;
    float max_accel;
} control_config_t;

    //New Global context structure
typedef struct {
    QueueHandle_t ctrl_event_queue; //OK on init
    TimerHandle_t my_timer; //OK on int
    controller_timer_context_t my_timer_context;//OK on init

    led_rgb_t leds;

    my_pid_t pid_controller;//OK on init

    drv8825_t one_driver;

    mpu_reader_t *my_reader; //OK on init
    uint8_t mpu_task_counter;
    SemaphoreHandle_t ctrl_sync_sem; //OK on init
    mpu_values_t mpu_val;
    uint32_t version_read; 
    uint16_t target_angle;   

    SemaphoreHandle_t config_mutex;
    control_config_t config;

} controller_ctx_t;

//extern portMUX_TYPE mpu_lock;

void controller_task(void *pvParam);

#endif
