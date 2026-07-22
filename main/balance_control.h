#ifndef BALANCE_CONTROL_H
#define BALANCE_CONTROL_H

#include "pid.h"

typedef struct {
    my_pid_t pid;
    float target_angle;
    float max_output;
} balance_control_t;

void balance_control_init(
    balance_control_t *control,
    float kp,
    float ki,
    float kd,
    float target_angle,
    float max_output
);

float balance_control_compute(
    balance_control_t *control,
    float angle,
    float angular_velocity,
    float dt
);

void balance_control_reset(balance_control_t *control);

#endif