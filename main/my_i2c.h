

#include "esp_check.h"
#include "driver/i2c_master.h"



void i2c_init(void);
esp_err_t i2c_delete_bus();

esp_err_t i2c_get_device(uint8_t addr, i2c_master_dev_handle_t* out_dev);
esp_err_t remove_i2c_device(i2c_master_dev_handle_t dev_handle);
esp_err_t i2c_transfer(i2c_master_dev_handle_t dev_handle, const uint8_t *tx_data, size_t tx_len, uint8_t *rx_data, size_t rx_len);
i2c_master_bus_handle_t get_i2c_bus_handle(void);