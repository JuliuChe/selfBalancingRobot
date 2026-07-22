#ifndef CONTROLLER_EVENTS_H
#define CONTROLLER_EVENTS_H

#include "esp_err.h"

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

typedef struct {
    ctrl_event_t type;         // Type d’événement
    esp_err_t err_code;        // Code d’erreur éventuel
    const char* msg;
    // Optionnel : timestamp, meta, etc.
} ctrl_event_msg_t;

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

#endif //CONTROLLER_EVENTS_H
