#include "balance_loop.h"
#include "esp_timer.h"
#include "esp_log.h"

#define TAG "BLNCE_LOOP"
static void balance_loop_task(void *pvParam);


esp_err_t balance_loop_init(balance_loop_t* bal, const balance_loop_config_t* config){
  if(!bal || !config) return ESP_ERR_INVALID_ARG;
  if (!config->my_reader || !config->ctrl_sync_sem || !config->ctrl_event_queue || !config->drivebase || !config->mpu_val || !config->balance_control || !config->mpu_lock){
    return ESP_ERR_INVALID_ARG;
  }
  if(bal->state != BALANCE_LOOP_STATE_UNINITIALIZED){
    return ESP_ERR_INVALID_STATE;
  }
  bal->balance_config = *config;
  bal->state = BALANCE_LOOP_STATE_READY;
  bal->task_handle=NULL;
  return ESP_OK;
}

esp_err_t balance_loop_start(balance_loop_t* bal){
  if(!bal) return ESP_ERR_INVALID_ARG;

  if(bal->state!=BALANCE_LOOP_STATE_READY) return ESP_ERR_INVALID_STATE;

  if(bal->task_handle) return ESP_ERR_INVALID_STATE;

  esp_err_t ret = drivebase_start(bal->balance_config.drivebase);
  if(ret!=ESP_OK) return ret;

  bal->state = BALANCE_LOOP_STATE_RUNNING;
  BaseType_t xTaskReturned = xTaskCreate(balance_loop_task, "balance_loop", 4096, bal, 10, &bal->task_handle);
  if(xTaskReturned!=pdPASS){
    bal->task_handle=NULL;
    bal->state = BALANCE_LOOP_STATE_READY;
    drivebase_stop(bal->balance_config.drivebase);
    return ESP_ERR_NO_MEM;
  }
  return ESP_OK;
}


static void balance_loop_task(void *pvParam){
  balance_loop_t* bal = (balance_loop_t*)pvParam;
  int64_t prevT = esp_timer_get_time();

  while(1){
    if(xSemaphoreTake(bal->balance_config.ctrl_sync_sem, pdMS_TO_TICKS(50))!=pdTRUE){
      if(!mpu_reader_is_connected(bal->balance_config.my_reader)){
        ESP_LOGI(TAG, "Lost connection with accelerometer... Exiting balance loop task");
        ctrl_event_msg_t new_event = { .type = EV_LOST_MPU, .err_code = ESP_OK, .msg=""};
        xQueueSend(bal->balance_config.ctrl_event_queue, &new_event,(TickType_t) 2);
        break;
      }
      continue;
    }

    float roll;
    float roll_speed;
    taskENTER_CRITICAL(bal->balance_config.mpu_lock);
      roll=bal->balance_config.mpu_val->kalman_filter.xk[0]; // Use Kalman filter roll angle
      roll_speed=bal->balance_config.mpu_val->kalman_filter.xk[1]; // Use Kalman filter roll speed
    taskEXIT_CRITICAL(bal->balance_config.mpu_lock);

    int64_t currT = esp_timer_get_time();
    float dt = (float)(currT - prevT)/(1000000.0f);
    prevT = currT;

    if(dt<=0.0f || dt>0.05f){continue;}

    if (roll <= ROBOT_MIN_BALANCE_ANGLE || roll >= ROBOT_MAX_BALANCE_ANGLE) {
    drivebase_stop(bal->balance_config.drivebase);
    ESP_LOGI(TAG, "Robot Lying ... Exiting balance loop task");
    ctrl_event_msg_t event = {.type = EV_STOP_BALANCING,.err_code = ESP_OK, .msg = ""};
    xQueueSend(bal->balance_config.ctrl_event_queue, &event, 0);
    break;
    }

    float command=balance_control_compute(bal->balance_config.balance_control, roll, roll_speed, dt);
    esp_err_t ret_bal = drivebase_apply_balance(bal->balance_config.drivebase, command, dt);
    if(ret_bal != ESP_OK){
      drivebase_stop(bal->balance_config.drivebase);
      ctrl_event_msg_t event = {.type = EV_ERROR,.err_code = ret_bal, .msg = "Impossible to apply speed command to motors"};
      xQueueSend(bal->balance_config.ctrl_event_queue, &event, 0);
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  drivebase_stop(bal->balance_config.drivebase);
  bal->state=BALANCE_LOOP_STATE_READY;
  bal->task_handle=NULL;
  vTaskDelete(NULL);

}