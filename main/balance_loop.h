#ifndef BALANCE_LOOP_H
#define BALANCE_LOOP_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"

#include "mpu_reader.h"
#include "drivebase.h"
#include "balance_control.h"
#include "controller_events.h"
#include "robot_config.h"

typedef struct{
  mpu_reader_t *my_reader;
  SemaphoreHandle_t ctrl_sync_sem; 
  QueueHandle_t ctrl_event_queue; 
  drivebase_t *drivebase;
  mpu_values_t *mpu_val;
  balance_control_t *balance_control;
  portMUX_TYPE *mpu_lock;

} balance_loop_t;

void balance_loop_task(void  *pvParam);

// void balance_loop_init(balance_loop_t* bal);
// void balance_loop_start(balance_loop_t* bal);
// void balance_loop_stop(balance_loop_t* bal);
// void balance_loop_deinit(balance_loop_t* bal);
// bool balance_loop_is_running(balance_loop_t* bal);

#endif