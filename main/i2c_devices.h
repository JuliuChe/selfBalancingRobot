//======================
// Adresses I2C des devices
//======================
#define I2C_MPU6050_ADDRESS 0x68 // Adresse I2C du MPU6050

//======================
// Ports GPIO i2C de l'esp32
//======================
#define GPIO_SDA GPIO_NUM_6// Port GPIO pour SDA
#define GPIO_SCL GPIO_NUM_7
#define I2C_PORT I2C_NUM_0 // Port I2C utilisé, peut être I2C_NUM_0 ou I2C_NUM_1 
//======================
// Macros pour les handles i2C
//======================
#include "driver/i2c_master.h"
#include "esp_log.h"