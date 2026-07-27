#include "balance_loop.h"
#include "esp_timer.h"
#include "esp_log.h"

#define TAG "BLNCE_LOOP"

void balance_loop_task(void *pvParam){
  balance_loop_t* bal = (balance_loop_t*)pvParam;
  int64_t prevT = esp_timer_get_time();

  while(1){
    if(xSemaphoreTake(bal->ctrl_sync_sem, pdMS_TO_TICKS(50))!=pdTRUE){
      if(!mpu_reader_is_connected(bal->my_reader)){
        ESP_LOGI(TAG, "Lost connection with accelerometer... Exiting balance loop task");
        ctrl_event_msg_t new_event = { .type = EV_LOST_MPU, .err_code = ESP_OK, .msg=""};
        xQueueSend(bal->ctrl_event_queue, &new_event,(TickType_t) 2);
        break;
      }
      continue;
    }

    float roll;
    float roll_speed;
    taskENTER_CRITICAL(bal->mpu_lock);
      roll=bal->mpu_val->kalman_filter.xk[0]; // Use Kalman filter roll angle
      roll_speed=bal->mpu_val->kalman_filter.xk[1]; // Use Kalman filter roll speed
    taskEXIT_CRITICAL(bal->mpu_lock);

    int64_t currT = esp_timer_get_time();
    float dt = (float)(currT - prevT)/(1000000.0f);
    prevT = currT;

    if(dt<=0.0f || dt>0.05f){continue;}

    if (roll <= ROBOT_MIN_BALANCE_ANGLE || roll >= ROBOT_MAX_BALANCE_ANGLE) {
    drivebase_stop(bal->drivebase);
    ESP_LOGI(TAG, "Robot Lying ... Exiting balance loop task");
    ctrl_event_msg_t event = {.type = EV_STOP_BALANCING,.err_code = ESP_OK, .msg = ""};
    xQueueSend(bal->ctrl_event_queue, &event, 0);
    break;
    }

    float command=balance_control_compute(bal->balance_control, roll, roll_speed, dt);
    esp_err_t ret_bal = drivebase_apply_balance(bal->drivebase, command, dt);
    if(ret_bal != ESP_OK){
      drivebase_stop(bal->drivebase);
      ctrl_event_msg_t event = {.type = EV_ERROR,.err_code = ret_bal, .msg = "Impossible to apply speed command to motors"};
      xQueueSend(bal->ctrl_event_queue, &event, 0);
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  vTaskDelete(NULL);

}