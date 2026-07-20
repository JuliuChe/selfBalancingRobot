#include "pid.h"
void pid_init(my_pid_t *pid, float kp, float ki, float kd) {
    pid->kp = kp; pid->ki = ki; pid->kd = kd;
    pid->prev_error = 0;
    pid->integral = 0;
}

float pid_compute(my_pid_t *pid, float error, float dt) {
    pid->integral += error * dt;
    float derivative = (error - pid->prev_error)/dt;
    pid->prev_error = error;
    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}


float pid_kal_compute(my_pid_t* pid, float error, float derivative, float dt){
    pid->integral += error * dt;
   return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}