#include <stddef.h>
#include <math.h>

#include "balance_control.h"

void balance_control_init(
    balance_control_t *control,
    float kp,
    float ki,
    float kd,
    float target_angle,
    float max_output
){
  if(control == NULL){
    return;
  }

  pid_init(&control->pid, kp, ki, kd);
  control->target_angle = target_angle;
  control->max_output = max_output;
}

float balance_control_compute(
    balance_control_t *control,
    float angle,
    float angular_velocity,
    float dt
){
  if (control == NULL || dt <= 0.0f) {
    return 0.0f;
  }
  float err = angle-(float)control->target_angle; // Calculate error based on target angle
  float command=pid_kal_compute(&control->pid, err, -angular_velocity, dt);

  return fminf(fmaxf(command, -control->max_output), control->max_output);

}

void balance_control_reset(balance_control_t *control){
 if (control == NULL) {
        return;
    }

  control->pid.integral = 0.0f;
  control->pid.prev_error = 0.0f;

}