#ifndef CONTROLLER
#define CONTROLLER


#include "mpu_reader.h"
#include "pid.h"
#include "led_rgb.h"
#include "drv8825.h"
#include "timer_controller.h"





//extern portMUX_TYPE mpu_lock;

void controller_task(void *pvParam);

#endif
