#ifndef KALMANFILTER_H
#define KALMANFILTER_H

typedef struct {
    float xk[3]; // Estimated roll angle
    float P[3][3];    // error covariance
} kalman_filter_t;

void kalman_roll_update(kalman_filter_t* kf, float accel_angle, float angular_gyro, float dt);

#endif