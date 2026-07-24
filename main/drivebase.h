#ifndef DRIVEBASE_H
#define DRIVEBASE_H

#include "esp_err.h"
#include "drv8825.h"

typedef struct {
    drv8825_t shared_driver;
} drivebase_t;

esp_err_t drivebase_init(drivebase_t *drivebase);
esp_err_t drivebase_start(drivebase_t *drivebase);
esp_err_t drivebase_stop(drivebase_t *drivebase);
esp_err_t drivebase_deinit(drivebase_t *drivebase);

void drivebase_apply_balance(
    drivebase_t *drivebase,
    float balance_command,
    float dt
);

#endif