#ifndef PID
#define PID

typedef struct {
    float kp, ki, kd;
    float prev_error;
    float integral;
} my_pid_t;

void pid_init(my_pid_t* pid, float kp, float ki, float kd);
float pid_compute(my_pid_t* pid, float error, float dt);
float pid_kal_compute(my_pid_t* pid, float error, float derivative, float dt);

#endif