#include "my_i2c.h"
#include "i2c_devices.h"
#include "robot_config.h"

#include "esp_log.h"
#include "esp_err.h"

#include "esp_system.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define TAG "I2C"


static SemaphoreHandle_t i2c_mutex = NULL;


static void i2c_scan_bus(void) {
    ESP_LOGI(TAG, "🔍 Scan I2C démarré...");
    i2c_master_bus_handle_t bus_handle = NULL;
    bus_handle=get_i2c_bus_handle();
    for (uint8_t i = 0x03; i < 0x78; i++) {
        i2c_master_dev_handle_t dev;
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = i,
            .scl_speed_hz = 100000
        };

        esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev);
        if (err == ESP_OK) {
            uint8_t dummy = 0x00;
            esp_err_t trans = i2c_master_transmit(dev, &dummy, 1, 1000 / portTICK_PERIOD_MS);
            if (trans == ESP_OK) {
                ESP_LOGI(TAG, "✅ Périphérique détecté à 0x%02X", i);
            }
            i2c_master_bus_rm_device(dev);
        }
    }

    ESP_LOGI(TAG, "🔍 Fin du scan I2C.");
};


static void i2c_scan_device(uint8_t i2c_addr) {
    ESP_LOGI(TAG, "🔍 Scan I2C for device 0x%x ...", i2c_addr);
    
    i2c_master_bus_handle_t bus_handle = NULL;
    bus_handle=get_i2c_bus_handle();
    i2c_master_dev_handle_t dev;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr,
        .scl_speed_hz = 100000
    };

    esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev);
    if (err == ESP_OK) {
        uint8_t dummy = 0x00;
        esp_err_t trans = i2c_master_transmit(dev, &dummy, 1, 1000 / portTICK_PERIOD_MS);
        if (trans == ESP_OK) {
            ESP_LOGI(TAG, "✅ Périphérique détecté à 0x%02X", i2c_addr);
        }
        i2c_master_bus_rm_device(dev);
    }
    

    ESP_LOGI(TAG, "🔍 Fin du scan I2C.");
};

static void i2c_init_impl(void){
    i2c_master_bus_handle_t bus_handle = NULL;
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = ROBOT_I2C_PORT, //OK
        .scl_io_num = ROBOT_GPIO_SCL,
        .sda_io_num = ROBOT_GPIO_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    i2c_mutex = xSemaphoreCreateMutex();

    vTaskDelay(pdMS_TO_TICKS(50));

    // Appel du scan I2C (à commenter plus tard si besoin)
    //i2c_scan_bus();
     
    i2c_scan_device(0x68);

    ESP_LOGI(TAG, "I2C Bus initialized with handle %p", bus_handle);

};


void i2c_init(void){
    i2c_init_impl();
}

i2c_master_bus_handle_t get_i2c_bus_handle(void) {
    i2c_master_bus_handle_t my_handle;
    i2c_port_num_t i2c_port = ROBOT_I2C_PORT; // Port I2C utilisé, peut être I2C_NUM_0 ou I2C_NUM_1
    
    esp_err_t ret=i2c_master_get_bus_handle(i2c_port, &my_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get I2C bus handle: %s", esp_err_to_name(ret));
        return NULL;
    }
    return my_handle;
}



//TODO : use for debug only : see i2c_transfer
void log_tx_data(const uint8_t *data, size_t len) {
    ESP_LOGI(TAG, "In i2c_transfer : tx_data (%d bytes):", (int)len);
    for (size_t i = 0; i < len; ++i) {
        printf(" %02X", data[i]);  // %02X = hexadécimal sur 2 chiffres
    }
    printf("\n");
};

esp_err_t i2c_transfer(i2c_master_dev_handle_t dev_handle, const uint8_t *tx_data, size_t tx_len, uint8_t *rx_data, size_t rx_len){
    if (!i2c_mutex) return ESP_ERR_NOT_FOUND;
    if(!dev_handle) {
       ESP_LOGI(TAG, "In i2c_transfer : dev_handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    i2c_master_bus_handle_t bus_handle = NULL;
    bus_handle=get_i2c_bus_handle();
    if(!bus_handle){
        ESP_LOGI(TAG, "In i2c_transfer : bus_handle is NULL");
        return ESP_ERR_NOT_FOUND;
    }

    //log_tx_data(tx_data, tx_len);
    esp_err_t res = ESP_FAIL;
    if(xSemaphoreTake(i2c_mutex, portMAX_DELAY)==pdTRUE){
        if(!rx_data){
            res =i2c_master_transmit(dev_handle, tx_data, tx_len,-1);
        } else {
            res =i2c_master_transmit_receive(dev_handle, tx_data, tx_len, rx_data, rx_len, -1);        
        }
        xSemaphoreGive(i2c_mutex);
    };
    return res;
};

esp_err_t i2c_delete_bus(){
    i2c_master_bus_handle_t bus_handle = NULL;
    bus_handle=get_i2c_bus_handle();
    if(!bus_handle) return ESP_FAIL;
    esp_err_t res = i2c_del_master_bus(bus_handle);
    bus_handle = NULL;
    if(i2c_mutex){
        vSemaphoreDelete(i2c_mutex);
        i2c_mutex = NULL;
    }
    return res;
};



esp_err_t i2c_add_device(uint8_t addr, i2c_master_dev_handle_t* out_dev) {
    i2c_master_bus_handle_t bus_handle = NULL;
    bus_handle=get_i2c_bus_handle();
    if(!bus_handle) return ESP_FAIL;
    
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 100000
    };
    return i2c_master_bus_add_device(bus_handle, &dev_cfg, out_dev);
};


esp_err_t remove_i2c_device(i2c_master_dev_handle_t dev_handle) {
    if (!i2c_mutex) return ESP_ERR_NOT_FOUND;
    if(!dev_handle) {
       ESP_LOGI(TAG, "In remov_i2c_device : dev_handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    i2c_master_bus_handle_t bus_handle = NULL;
    bus_handle=get_i2c_bus_handle();
    if(!bus_handle){
        ESP_LOGI(TAG, "In remov_i2c_device : bus_handle is NULL");
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t res = ESP_FAIL;
    if(xSemaphoreTake(i2c_mutex, portMAX_DELAY)==pdTRUE){
        res = i2c_master_bus_rm_device(dev_handle);
        xSemaphoreGive(i2c_mutex);
    };
    return res;
};
