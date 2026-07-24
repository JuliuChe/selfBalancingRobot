#pragma once
#include "mpu6050.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "kalmanfilter.h"

#define MPU_FRAME_SIZE 12 // Size of the MPU6050 data frame (6 bytes for accelerometer + 6 bytes for gyroscope)

typedef struct {
    float gyro_sensitivity;
    float acce_sensitivity;
}sensor_sensitivity_t;

typedef struct {
    volatile uint32_t version;
    mpu6050_acce_value_t acce_value;
    mpu6050_gyro_value_t gyro_value;
    // complimentary_angle_t angle_values;
    kalman_filter_t kalman_filter; // Kalman filter for roll angle
} mpu_values_t;

typedef enum{
    GYRO_NO_CAL=0, // No calibration
    GYRO_CALIBRATE, // Calibrate gyroscope
} gyro_calibration_t;

typedef struct{
    gyro_calibration_t gyro; // Gyroscope calibration mode
    mpu6050_acce_fs_t acce_fs; // Accelerometer full scale range
    mpu6050_gyro_fs_t gyro_fs; // Gyroscope full scale range
    mpu6050_sampling_config_t sampling_config; // Sampling configuration    
}mpu_config_t;


typedef enum {
    FIFO_SOURCE_ACCELEROMETER = 0x08, // Accelerometer data
    FIFO_SOURCE_GYROSCOPE_X = 0x40, // Gyroscope data
    FIFO_SOURCE_GYROSCOPE_Y = 0x20,
    FIFO_SOURCE_GYROSCOPE_Z = 0x10,
    FIFO_SOURCE_TEMPERATURE = 0x80, // Temperature data
    FIFO_SOURCE_GYRO_ACC = FIFO_SOURCE_ACCELEROMETER | FIFO_SOURCE_GYROSCOPE_X | FIFO_SOURCE_GYROSCOPE_Y | FIFO_SOURCE_GYROSCOPE_Z, // All sources except temperature
    FIFO_SOURCE_ALL = FIFO_SOURCE_GYRO_ACC | FIFO_SOURCE_TEMPERATURE // All sources including temperature
} fifo_sources_t;

// Déclaration opaque
typedef struct mpu_reader mpu_reader_t;

// Crée/init toutes les ressources (pas encore les tâches)
mpu_reader_t* mpu_reader_create(void);

// Activate the sensor and configure it (FS, sampling rate, DLPF, get sensitivities, calibrate gyro etc.)
esp_err_t mpu_reader_config(mpu_reader_t* reader, mpu_config_t* config);

/**
 * @brief Configure interrupts on the sensor (pin, sources etc.) and create the queue for FIFO data frames.
 * @param : reader must have .int_pin set to a valid GPIO number and .en_sensors set to the desired FIFO sources
 * @return
 *      - ESP_OK Success
 *      - ESP_ERR_INVALID_ARG A parameter is NULL or not valid
 *      - ESP_FAIL Failed to enable interrupt sources on mpu6050
**/ 
esp_err_t mpu_reader_int_config(mpu_reader_t* reader, gpio_num_t int_pin, fifo_sources_t sources);

// Launches the tasks to read the sensor data and process it
// The tasks will run in the background and fill the queues with data
esp_err_t mpu_reader_start(mpu_reader_t* reader);

// Demande l’arrêt propre (flag), attend/force la fin des tâches et libère les queues/tâches
esp_err_t mpu_reader_stop(mpu_reader_t* reader);

// Libère le handle capteur, toute ressource dynamique finale
void mpu_reader_deinit(mpu_reader_t* reader);

//Set output buffer and mutex
esp_err_t mpu_reader_set_output_buffer(mpu_reader_t* reader, mpu_values_t* output_values,  SemaphoreHandle_t* sem, portMUX_TYPE* out_lock);

//Restart mpu after mpu_task suspended on mpu error
esp_err_t mpu_reader_resume(mpu_reader_t* reader);

//Check if mpu is still connected or if an error occurred and is now disconnected
bool mpu_reader_is_connected(mpu_reader_t* reader);

//TODO : TO BE DELETED
// Accès thread-safe à la dernière valeur mesurée sans seqLock
bool mpu_reader_get_latest(mpu_reader_t* reader, mpu_values_t* out);


// /**
//  * @brief Check if new MPU data has been written since last check.
//  *
//  * @param reader Pointer to reader instance.
//  * @param last_version_seen Pointer to version seen during last read.
//  * @return true if new data is available, false otherwise.
//  */
// bool mpu_reader_has_new_data(mpu_reader_t* reader, uint32_t* last_version_seen);

// bool mpu_reader_try_get_latest(mpu_reader_t* reader, uint32_t* last_version_seen, mpu_values_t* out); 


