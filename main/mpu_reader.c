
#include "mpu_reader.h"
#include "robot_config.h"

#include "i2c_devices.h"
#include "my_i2c.h"
#include "esp_timer.h"

#include <math.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

static const char *TAG = "MPU_READER";

// #define MPU_FRAME_SIZE  12 // Size of the MPU6050 data frame (6 bytes for accelerometer + 6 bytes for gyroscope)
#define FIFO_MAX_SIZE   1024U // Maximum size of the FIFO buffer
#define PERF_LOG_INTERVAL_SAMPLES 1000U

// Définis un bit pour l’arrêt
#define MPU_TASK_STOPPED_BIT   BIT0
#define MPU_PROC_TASK_STOPPED_BIT BIT1

#define RAD_TO_DEG      57.29577951f /*!< Radians to degrees */


static uint32_t counter = 0;
static uint32_t counter_out =0;

//Internal structure to be used within mpu_reader.c
struct mpu_reader {
       // To inject roll, pitch, accX,Y,Z and gyroX,Y,Z values
    portMUX_TYPE* output_lock;
    SemaphoreHandle_t* mpu_sem;
    mpu_values_t* output_values;


    mpu6050_handle_t mpu6050;

    // To be used for interrupt config
    size_t fifo_frame_size; // Size of the FIFO frame based on enabled sensors
    QueueHandle_t mpu_frame_queue;


    //started tasks in mpu_reader_start()
    TaskHandle_t mpu_task_handle;
    TaskHandle_t mpu_processing_handle;
    EventGroupHandle_t stop_event_group;
    volatile bool running;
    volatile bool connected;


    //Private variables
    sensor_sensitivity_t sensitivity; //OK KEEP
    mpu_values_t values;

};

//i2c bus must be instanciated before calling init in this API

static size_t get_fifo_frame_size(fifo_sources_t fifo_config) {
    size_t size = 0;
    // Accelerometer : 6 bytes (X, Y, Z)
    if (fifo_config & FIFO_SOURCE_ACCELEROMETER) {
        size += 3U*sizeof(int16_t);
    }
    // Gyro X, Y, Z : 2 bytes chacun
    if (fifo_config & FIFO_SOURCE_GYROSCOPE_X) {
        size += sizeof(int16_t);
    }
    if (fifo_config & FIFO_SOURCE_GYROSCOPE_Y) {
        size += sizeof(int16_t);
    }
    if (fifo_config & FIFO_SOURCE_GYROSCOPE_Z) {
        size += sizeof(int16_t);
    }
    // Temp : 2 bytes
    if (fifo_config & FIFO_SOURCE_TEMPERATURE) {
        size += sizeof(int16_t);
    }
    return size;
}

esp_err_t get_sensitivities(mpu6050_handle_t sensor, sensor_sensitivity_t* sensor_sensitivity){
    esp_err_t ret;

    ret = mpu6050_get_gyro_sensitivity(sensor, &sensor_sensitivity->gyro_sensitivity );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get gyro sensitivity: %s", esp_err_to_name(ret));
        return ret;
    }


    ret = mpu6050_get_acce_sensitivity(sensor, &sensor_sensitivity->acce_sensitivity);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get accelerometer sensitivity: %s", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

void mpu_values_add(mpu_values_t* out, const mpu_values_t* in) {
    out->acce_value.acce_x += in->acce_value.acce_x;
    out->acce_value.acce_y += in->acce_value.acce_y;
    out->acce_value.acce_z += in->acce_value.acce_z;

    out->gyro_value.gyro_x += in->gyro_value.gyro_x;
    out->gyro_value.gyro_y += in->gyro_value.gyro_y;
    out->gyro_value.gyro_z += in->gyro_value.gyro_z;

    // out->angle_values.roll  += in->angle_values.roll;
    // out->angle_values.pitch += in->angle_values.pitch;
}

void mpu_values_average(mpu_values_t* out, mpu_values_t* accum, uint8_t nb){
    out->acce_value.acce_x = accum->acce_value.acce_x/((float)nb);
    out->acce_value.acce_y = accum->acce_value.acce_y/((float)nb);
    out->acce_value.acce_z = accum->acce_value.acce_z/((float)nb);

    out->gyro_value.gyro_x = accum->gyro_value.gyro_x/((float)nb);
    out->gyro_value.gyro_y = accum->gyro_value.gyro_y/((float)nb);
    out->gyro_value.gyro_z = accum->gyro_value.gyro_z/((float)nb);

    // out->angle_values.roll  = accum->angle_values.roll/((float)nb);
    // out->angle_values.pitch = accum->angle_values.pitch/((float)nb);
}

static void IRAM_ATTR mpu_reader_isr_handler(void* arg)
{
    mpu_reader_t* reader = (mpu_reader_t*)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (reader && reader->mpu_task_handle) {
        // Notify the MPU task that an interrupt has occurred
        // This will wake up the task if it is blocked on ulTaskNotifyTake()
        // or will set the notification value if it is not blocked.
        vTaskNotifyGiveFromISR(reader->mpu_task_handle, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
    
}

float get_delta_time_s() {
    static struct timeval last = {0};
    struct timeval now;

    gettimeofday(&now, NULL);

    float dt = ROBOT_MPU_EXPECTED_PERIOD_S;  // valeur par défaut
    if (last.tv_sec != 0 || last.tv_usec != 0) {
        time_t sec_diff = now.tv_sec - last.tv_sec;
        suseconds_t usec_diff = now.tv_usec - last.tv_usec;
        dt = sec_diff + usec_diff / 1e6f;

        // Protection contre des valeurs aberrantes (ex: après suspend/reboot)
        if (dt < ROBOT_MPU_MIN_VALID_PERIOD_S || dt > ROBOT_MPU_MAX_VALID_PERIOD_S) {
            dt = 0.004f;
        }
    }

    last = now;  // mise à jour pour le prochain appel
    return dt;
}


void mpu_task(void *pvParameters) {
    mpu_reader_t *reader=(mpu_reader_t *)pvParameters;
    reader->mpu_task_handle = xTaskGetCurrentTaskHandle();
    esp_err_t ret;
       //6. Register ISR handler
        //6.1 Install ISR service
    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "ISR service install failed: %s", esp_err_to_name(isr_err));
        return;
    }
        //6.2 Register the ISR handler
    ret = mpu6050_register_isr(reader->mpu6050, mpu_reader_isr_handler, reader);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register ISR: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "MPU6050 ready");
    
    
    uint8_t status;
    mpu6050_get_interrupt_status(reader->mpu6050, &status);
    ESP_LOGI(TAG, "Interrupt Status %d", status);
    ret=mpu6050_reset_fifo_stack(reader->mpu6050);
    if( ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset FIFO stack: %s", esp_err_to_name(ret));
        return;
    } else {
        reader->connected=true;
        ESP_LOGI(TAG, "FIFO stack reset successfully");
    }
    uint8_t fifo_data[reader->fifo_frame_size]; // Buffer to hold FIFO data

    while (reader->running ) {
  
        uint8_t notify = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ROBOT_MPU_INTERRUPT_TIMEOUT_MS));

        if (notify == 0) {
        // Timer a expiré, PAS d'interruption reçue
            ESP_LOGW(TAG, "Timeout: aucune interruption reçue pendant %u ms", (unsigned)ROBOT_MPU_INTERRUPT_TIMEOUT_MS);
            mpu6050_get_interrupt_status(reader->mpu6050, &status);
            reader->connected=false;
            vTaskSuspend(NULL);
            continue;
        }

        if(mpu6050_get_interrupt_status(reader->mpu6050, &status) == ESP_OK) {

            //If FIFO overflow interrupt is received, reset FIFO stack and wait for next value (exit current loop and wait for next interrupt)
            if(mpu6050_is_fifo_overflow_interrupt(status)) {
                ESP_LOGW(TAG, "FIFO overflow interrupt received");
                // Reset FIFO stack 
                ret = mpu6050_reset_fifo_stack(reader->mpu6050);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to reset FIFO stack: %s", esp_err_to_name(ret));
                }
                mpu6050_get_interrupt_status(reader->mpu6050, &status);
                continue; 
            } 

            //If FIFO data ready interrupt is received, empty the FIFO buffer
            if(mpu6050_is_data_ready_interrupt(status)) {
                // Data ready interrupt received, read FIFO data
                size_t actual_len = 0;
                ret = mpu6050_get_fifo_count(reader->mpu6050, &actual_len);
                if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to get FIFO count: %s", esp_err_to_name(ret));

                // Check if the actual length is a multiple of the FIFO frame size. If not, reset FIFO stack and wait for next interrupt
                if (actual_len % sizeof(fifo_data)!=0 || actual_len > FIFO_MAX_SIZE) {
                    ESP_LOGW(TAG, "FIFO data length mismatch, bytes to read. Length of FIFO queue : %d", actual_len);
                    mpu6050_reset_fifo_stack(reader->mpu6050);
                    mpu6050_get_interrupt_status(reader->mpu6050, &status);
                    continue; 
                }

                while(actual_len>= reader->fifo_frame_size) {
                    // Read FIFO data
                    size_t read_len = reader->fifo_frame_size;
                    ret = mpu6050_read_fifo(reader->mpu6050, fifo_data, &read_len);
                    if (ret != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to read FIFO data: %s", esp_err_to_name(ret));
                        break; // Exit the loop on error
                    }

                    // Push non bloquant: si la queue est pleine, on DROP (mieux que bloquer)
                    if (xQueueSend(reader->mpu_frame_queue, fifo_data, 0) != pdTRUE) {
                        ESP_LOGW(TAG, "Queue full, dropping frame");
                    }
                    actual_len -= read_len; // Decrease the remaining length
                }
                // ret = mpu6050_read_fifo(reader->mpu6050, fifo_data, &actual_len);
                // if (ret == ESP_OK) {
                //     xQueueSend(reader->mpu_frame_queue, fifo_data, (TickType_t) 2); // Process FIFO data as needed
                // }
            }
        }
        //vTaskDelay(pdMS_TO_TICKS(1)); 
    }
    xEventGroupSetBits(reader->stop_event_group, MPU_TASK_STOPPED_BIT);
    vTaskDelete(NULL); // Delete the task when done
}

void mpu_processing_task(void* arg) {
    mpu_reader_t *reader=(mpu_reader_t *)arg;
    reader->mpu_processing_handle = xTaskGetCurrentTaskHandle();

    const mpu6050_gyro_value_t* cal = mpu6050_get_gyro_calibrations();
    uint8_t frame[reader->fifo_frame_size]; // Buffer to hold FIFO data
    //mpu_values_t accum_values;
    //uint8_t k=0;
    //uint8_t idx=1;

    
    // float exec=0.0f;
    float accel_roll_angle;

    while (reader->running ) {
        if (xQueueReceive(reader->mpu_frame_queue, frame, portMAX_DELAY)) {

            //esp_err_t ret;
            int64_t start = esp_timer_get_time();
            reader->values.acce_value.acce_x = ((int16_t)((frame[0] << 8) | frame[1]))/((float)(reader->sensitivity.acce_sensitivity));
            reader->values.acce_value.acce_y = ((int16_t)((frame[2] << 8) | frame[3]))/((float)(reader->sensitivity.acce_sensitivity));
            reader->values.acce_value.acce_z = ((int16_t)((frame[4] << 8) | frame[5]))/((float)(reader->sensitivity.acce_sensitivity));
            reader->values.gyro_value.gyro_x = ((int16_t)((frame[6] << 8) | frame[7]))/((float)(reader->sensitivity.gyro_sensitivity));
            reader->values.gyro_value.gyro_y = ((int16_t)((frame[8] << 8) | frame[9]))/((float)(reader->sensitivity.gyro_sensitivity));
            reader->values.gyro_value.gyro_z = ((int16_t)((frame[10] << 8) | frame[11]))/((float)(reader->sensitivity.gyro_sensitivity));
            
            int64_t t2 = esp_timer_get_time();
            
            // Traitement (filtrage, calcul roll/pitch, log...)
            //ret = mpu6050_complimentory_filter(reader->mpu6050, &reader->values.acce_value, &reader->values.gyro_value, &reader->values.angle_values);
            // if(ret!=ESP_OK) {
            //     ESP_LOGE(TAG, "Failed to apply complimentary filter: %s", esp_err_to_name(ret));
            //     continue;
            // } 
            accel_roll_angle=atan2f(-reader->values.acce_value.acce_y, reader->values.acce_value.acce_z) * RAD_TO_DEG;
        

            reader->values.gyro_value.gyro_x-=cal->gyro_x;
            reader->values.gyro_value.gyro_y-=cal->gyro_y;
            reader->values.gyro_value.gyro_z-=cal->gyro_z;

            //Uncomment to log the values
            //     k+=1;
            // if(k==3){
            //     k=0;
            //     ESP_LOGI(TAG, "Accel Roll Angle: %.2f, Angular gyro : %.2f", accel_roll_angle,reader->values.gyro_value.gyro_x );
            // }


            int64_t t3 = esp_timer_get_time();
            float dt=get_delta_time_s();
            kalman_roll_update(&reader->values.kalman_filter, accel_roll_angle,  reader->values.gyro_value.gyro_x, dt); // Assuming a fixed dt of 0.004 seconds (250 Hz)
            int64_t t4 = esp_timer_get_time();
            
            counter++;

            if ((counter % PERF_LOG_INTERVAL_SAMPLES) == 0) {
                ESP_LOGI("MPU_PROCESSING", "Getting values : %.2f, Atan compute: %.2f, kalman filtering: %.2f", (float)(t2-start), (float)(t3-t2), (float)(t4-t3));
            }

        }
        int64_t t5 = esp_timer_get_time();
        int64_t t6 = 0;
        if (reader->output_values && reader->mpu_sem) {
            taskENTER_CRITICAL(reader->output_lock);
            reader->output_values->version++;
            reader->output_values->acce_value = reader->values.acce_value;
            reader->output_values->gyro_value = reader->values.gyro_value;
            reader->output_values->kalman_filter = reader->values.kalman_filter;
            reader->output_values->version++;
            taskEXIT_CRITICAL(reader->output_lock);
            t6 = esp_timer_get_time();
            xSemaphoreGive((*reader->mpu_sem));
        }
        int64_t t7 = esp_timer_get_time();

        counter_out++;

        if ((counter_out % PERF_LOG_INTERVAL_SAMPLES) == 0) {
                ESP_LOGI("MPU_PROCESSING", "Copy values in loop : %.2f, Give back semaphore : %.2f", (float)(t6-t5), (float)(t7-t6));
            }
    }
    xEventGroupSetBits(reader->stop_event_group, MPU_PROC_TASK_STOPPED_BIT);
    vTaskDelete(NULL); // Delete the task when done
}




//API "Public functions - getters and setters"
mpu_reader_t* mpu_reader_create(void) {
    mpu_reader_t* reader = calloc(1, sizeof(mpu_reader_t));
    if (!reader) return NULL;

    reader->mpu6050 = mpu6050_create(ROBOT_I2C_PORT, MPU6050_I2C_ADDRESS);
    if (!reader->mpu6050) {
        free(reader);
        return NULL;
    }
    reader->running = false;   
    return reader;
}

esp_err_t mpu_reader_config(mpu_reader_t* reader, mpu_config_t* config) {

    esp_err_t ret;
    if (!reader->mpu6050) return ESP_FAIL;

    // 1. Wake up sensor
    ret = mpu6050_wake_up(reader->mpu6050);
    if (ret != ESP_OK) return ret;

    //2. Configure the sensor (FS, sampling rate, DLPF etc.)
    ret = mpu6050_config(reader->mpu6050, config->acce_fs, config->gyro_fs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = mpu6050_config_sampling(reader->mpu6050, &config->sampling_config);
    if( ret == ESP_OK) {
        ESP_LOGI(TAG, "Sampling configuration set successfully with sampling frequency: %d Hz",
                 config->sampling_config.sampling_freq);
    } else {
        ESP_LOGE(TAG, "Failed to configure sampling: %s", esp_err_to_name(ret));
        return ret;
    }

     // 3. Calibration (optional)
    if(config->gyro == GYRO_CALIBRATE) {
        ret = calibrate_gyro(reader->mpu6050);
        if (ret != ESP_OK) return ret;
    }

    // 4. Get the sensitivities of the sensors
    ret = get_sensitivities(reader->mpu6050, &reader->sensitivity);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get sensor sensitivities: %s", esp_err_to_name(ret));
        return ret;
    }
  
    return ESP_OK;

}

esp_err_t mpu_reader_int_config(mpu_reader_t* reader, gpio_num_t int_pin, fifo_sources_t sources) {

    esp_err_t ret; 
    //1. Configure the interrupt behaviour & GPIO pin
    mpu6050_int_config_t int_config = {
        .interrupt_pin = int_pin,
        .active_level = INTERRUPT_PIN_ACTIVE_LOW, // Active low
        .pin_mode = INTERRUPT_PIN_OPEN_DRAIN , // Open drain mode
        .interrupt_latch = INTERRUPT_LATCH_UNTIL_CLEARED, // Latch until cleared
        .interrupt_clear_behavior = INTERRUPT_CLEAR_ON_STATUS_READ // Clear on any read
    };
    ret=mpu6050_config_interrupts(reader->mpu6050, &int_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to config interrupt behaviour and setup GPIO : %s", esp_err_to_name(ret));
        return ret;
    }

    //2. Enable the Interrupts sources in the MPU6050
    uint8_t interrupt_sources = MPU6050_DATA_RDY_INT_BIT |
    MPU6050_FIFO_OVERFLOW_INT_BIT; // Enable data ready and fifo overflow interrupts
    ret = mpu6050_enable_interrupts(reader->mpu6050, interrupt_sources);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable interrupt sources: %s", esp_err_to_name(ret));
        return ret;
    }

    //3. enable FIFO stack
    ret=mpu6050_enable_fifo(reader->mpu6050, sources);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable FIFO stack: %s", esp_err_to_name(ret));
        return ret;
    }

    // 4. Configure the FIFO frame size based on enabled sensors
    reader->fifo_frame_size = get_fifo_frame_size(sources);
    
    //5. Create the queue for MPU6050 data frames
    reader->mpu_frame_queue = xQueueCreate(ROBOT_MPU_FRAME_QUEUE_LENGTH, reader->fifo_frame_size);
    if (!reader->mpu_frame_queue) {
        mpu6050_delete(reader->mpu6050);
        return ESP_FAIL;
    }
    

    return ESP_OK;
}

esp_err_t mpu_reader_start(mpu_reader_t* reader){
    BaseType_t ret;

    reader->stop_event_group = xEventGroupCreate();
    if(!reader->stop_event_group){
        ESP_LOGE(TAG, "Failed to create Event Group");
        return ESP_FAIL;
    }
    
    // 1. Crée la tâche de lecture
    ret = xTaskCreate(mpu_task, "mpu_task", ROBOT_MPU_READER_TASK_STACK_SIZE, reader, ROBOT_MPU_READER_TASK_PRIORITY, &reader->mpu_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create MPU task");
        return ESP_FAIL;
    }
    reader->running = true;
    reader->connected = false;
    ret =  xTaskCreate(mpu_processing_task, "mpu_processing_task", ROBOT_MPU_PROCESS_TASK_STACK_SIZE, reader, ROBOT_MPU_PROCESS_TASK_PRIORITY, &reader->mpu_processing_handle);
        if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create MPU Processing task");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t mpu_reader_stop(mpu_reader_t* reader){
    if (!reader || !reader->running) {
        ESP_LOGW(TAG, "MPU reader is not running or already stopped");
        return ESP_OK; // Nothing to stop
    }
    reader->running = false;
     // Attends que les deux tâches aient signalé leur arrêt (timeout 50 ms)
    EventBits_t bits = xEventGroupWaitBits(reader->stop_event_group, (MPU_TASK_STOPPED_BIT | MPU_PROC_TASK_STOPPED_BIT),pdTRUE, pdTRUE, pdMS_TO_TICKS(ROBOT_MPU_TASK_STOP_TIMEOUT_MS)); 
    if ((bits & (MPU_TASK_STOPPED_BIT | MPU_PROC_TASK_STOPPED_BIT)) != (MPU_TASK_STOPPED_BIT | MPU_PROC_TASK_STOPPED_BIT)) {
        ESP_LOGE(TAG, "Timeout waiting for MPU tasks to stop! Forcing vTaskDelete");
        if((bits & MPU_TASK_STOPPED_BIT) == MPU_TASK_STOPPED_BIT){
            vTaskDelete(reader->mpu_task_handle);
        }
        if((bits & MPU_PROC_TASK_STOPPED_BIT) == MPU_PROC_TASK_STOPPED_BIT){
            vTaskDelete(reader->mpu_processing_handle);
        }
    } else { 
        ESP_LOGI(TAG, "MPU tasks stopped cleanly.");
    }

    // Deinitialize the MPU reader
    mpu_reader_deinit(reader);

    vEventGroupDelete(reader->stop_event_group);
    reader->stop_event_group = NULL;
    free(reader);

    return ESP_OK;
}

void mpu_reader_deinit(mpu_reader_t* reader){
    if (reader->mpu_frame_queue) {
        vQueueDelete(reader->mpu_frame_queue);
        reader->mpu_frame_queue = NULL;
    }
    if (reader->mpu6050) {
        mpu6050_delete(reader->mpu6050);
        reader->mpu6050 = NULL;
    }
    reader->mpu_task_handle = NULL;
    reader->mpu_processing_handle = NULL;

    reader->output_values = NULL;
    reader->mpu_sem = NULL;
    reader->output_lock = NULL;
}

//Set output buffer and mutex addresses
esp_err_t mpu_reader_set_output_buffer(mpu_reader_t* reader, mpu_values_t* output_values,  SemaphoreHandle_t* sem, portMUX_TYPE* out_lock){
    if (!reader || !output_values || !sem) {
        return ESP_ERR_INVALID_ARG;
    }
    reader->output_values=output_values;
    reader->mpu_sem=sem;
    reader->output_lock = out_lock;
    return ESP_OK;
}

bool mpu_reader_is_connected(mpu_reader_t* reader) {
    //Check if reader is not null and is connected
    return reader && reader->connected;
}


// Exemple d’API pour relancer
esp_err_t mpu_reader_resume(mpu_reader_t* reader) {
    if (!reader || reader->mpu6050 == NULL || !reader->mpu_task_handle) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "Preparing MPU task resume");

    /*
     * Supprimer les anciennes trames logicielles.
     */
    if (reader->mpu_frame_queue != NULL) {
        xQueueReset(reader->mpu_frame_queue);
    }

    /*
     * Réinitialiser la FIFO matérielle avant de réveiller la tâche.
     */
    esp_err_t err = mpu6050_reset_fifo_stack(reader->mpu6050);

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Failed to reset FIFO before resume: %s",
            esp_err_to_name(err)
        );

        reader->connected = false;
        return err;
    }

    /*
     * Lire INT_STATUS afin d'effacer une interruption latched
     * éventuellement restée active.
     */
    uint8_t status = 0;

    err = mpu6050_get_interrupt_status(
        reader->mpu6050,
        &status
    );

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Failed to clear interrupt status before resume: %s",
            esp_err_to_name(err)
        );

        reader->connected = false;
        return err;
    }

    ESP_LOGI(
        TAG,
        "Interrupt status before resume: 0x%02X",
        status
    );

    /*
     * Supprimer une éventuelle ancienne notification de tâche.
     */
    xTaskNotifyStateClear(reader->mpu_task_handle);

    /*
     * Ne déclarer connected=true qu'après réception
     * effective d'une nouvelle mesure.
     */
    reader->connected = false;

    if (eTaskGetState(reader->mpu_task_handle) == eSuspended) {
        ESP_LOGI(TAG, "Resuming suspended MPU task...");
        vTaskResume(reader->mpu_task_handle);
    }
    return ESP_OK;
}