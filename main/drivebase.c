#include "drivebase.h"
#include "robot_config.h"

#define TAG "DRIVEBASE"

esp_err_t drivebase_init(drivebase_t *drivebase){
  if(!drivebase){
    return ESP_ERR_INVALID_ARG;
  }
  drv8825_config_t driverConfig = {
    ROBOT_DRV8825_1_GPIO_STEP,
    ROBOT_DRV8825_1_GPIO_DIR,
    ROBOT_DRV8825_1_GPIO_SLEEP,
    ROBOT_DRV8825_1_GPIO_EN,
    ROBOT_DRV8825_1_GPIO_M0,
    ROBOT_DRV8825_1_GPIO_M1,
    ROBOT_DRV8825_1_GPIO_M2,
    ROBOT_DRV8825_1_MAX_STEPS_SEC,
    ROBOT_DRV8825_1_MIN_STEPS_SEC,
    ROBOT_DRV8825_1_INIT_MICROSTEP,
    ROBOT_DRV8825_MAX_ACCEL
  };

  return drv8825_init(
    &drivebase->shared_driver, &driverConfig
   );
}

esp_err_t drivebase_start(drivebase_t *drivebase){
  if(!drivebase){
    return ESP_ERR_INVALID_ARG;
  }
  return drv8825_start(&drivebase->shared_driver);
}

esp_err_t drivebase_stop(drivebase_t *drivebase){
  if(!drivebase){
    return ESP_ERR_INVALID_ARG;
  }
  return drv8825_stop(&drivebase->shared_driver);
}

esp_err_t drivebase_deinit(drivebase_t *drivebase){
  if(!drivebase){
    return ESP_ERR_INVALID_ARG;
  }
  return drv8825_deinit(&drivebase->shared_driver);
}

esp_err_t drivebase_apply_balance(
  drivebase_t *drivebase,
  float balance_command,
  float dt
){
  if(!drivebase){
    return ESP_ERR_INVALID_ARG;
  }

  drv8825_set_target_speed(&drivebase->shared_driver, balance_command);

  esp_err_t ret_updt = drv8825_update(&drivebase->shared_driver, dt);
  if(ret_updt != ESP_OK) return ret_updt;
  return ESP_OK;
}
