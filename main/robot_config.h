#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "driver/gpio.h"
/* Balance-controller defaults */

#define ROBOT_PID_KP_DEFAULT     20.0f
#define ROBOT_PID_KI_DEFAULT      0.0f
#define ROBOT_PID_KD_DEFAULT      1.0f

#define ROBOT_PID_GAIN_MIN        0.0f
#define ROBOT_PID_GAIN_MAX       50.0f

#define ROBOT_TARGET_ANGLE_DEFAULT 90.0f
#define ROBOT_TARGET_ANGLE_MIN     70.0f
#define ROBOT_TARGET_ANGLE_MAX    110.0f

#define ROBOT_BALANCE_MAX_OUTPUT 1500.0f

/* Safety limits */

#define ROBOT_MIN_BALANCE_ANGLE 20.0f
#define ROBOT_MAX_BALANCE_ANGLE 150.0f

/*Current shared motor-driver configuration*/
// DRV8825 Gpio Pins
#define ROBOT_DRV8825_1_GPIO_M0     GPIO_NUM_11
#define ROBOT_DRV8825_1_GPIO_M1     GPIO_NUM_10
#define ROBOT_DRV8825_1_GPIO_M2     GPIO_NUM_18
#define ROBOT_DRV8825_1_GPIO_STEP   GPIO_NUM_1
#define ROBOT_DRV8825_1_GPIO_DIR    GPIO_NUM_4
#define ROBOT_DRV8825_1_GPIO_EN     GPIO_NUM_NC
#define ROBOT_DRV8825_1_GPIO_SLEEP  GPIO_NUM_NC

// Timing for motor driver
#define ROBOT_DRV8825_1_MAX_STEPS_SEC 1300 //was 1350 steps/s
#define ROBOT_DRV8825_1_MIN_STEPS_SEC 50 //was 50
#define ROBOT_DRV8825_1_PWM_RES (1U*1000U*1000U)
#define ROBOT_DRV8825_MAX_ACCEL    5000.0f

/* I2C Bus Config*/
#define ROBOT_GPIO_SDA GPIO_NUM_6// Port GPIO pour SDA
#define ROBOT_GPIO_SCL GPIO_NUM_7
#define ROBOT_I2C_PORT I2C_NUM_0 // Port I2C utilisé, peut être I2C_NUM_0 ou I2C_NUM_1


/*MPU Config*/
#define ROBOT_MPU_INT_PIN GPIO_NUM_3

#define ROBOT_MPU_FRAME_QUEUE_LENGTH          128U // Size of the queue for MPU6050 data frames

#define ROBOT_MPU_EXPECTED_PERIOD_S           0.004f
#define ROBOT_MPU_MIN_VALID_PERIOD_S          0.001f
#define ROBOT_MPU_MAX_VALID_PERIOD_S          0.020f

#define ROBOT_MPU_INTERRUPT_TIMEOUT_MS        50U
#define ROBOT_MPU_TASK_STOP_TIMEOUT_MS        50U

#define ROBOT_MPU_READER_TASK_STACK_SIZE      4096U
#define ROBOT_MPU_READER_TASK_PRIORITY        5U

#define ROBOT_MPU_PROCESS_TASK_STACK_SIZE     4096U
#define ROBOT_MPU_PROCESS_TASK_PRIORITY       6U



#endif