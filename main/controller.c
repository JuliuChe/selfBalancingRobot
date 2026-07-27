#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include <math.h>

#include "my_i2c.h"
#include "mpu_reader.h"
#include "led_rgb.h"
#include "drivebase.h"
#include "balance_loop.h"

#include "robot_config.h"
#include "controller.h"
#include "controller_events.h"
#include "balance_control.h"
#include "timer_controller.h"



#define CTRL_QUEUE_SIZE 32 // Size of the queue for MPU6050 data frames
#define MAX_MPU_RESTART 10

#define TAG "CTRLER"


typedef enum {
    CTRL_INIT,  
    CTRL_CONFIG,
    CTRL_MPU_CALIBRATION, 
    CTRL_START_MPU, 
    CTRL_LOST_MPU,
    CTRL_BALANCING,
    CTRL_ERROR,
    CTRL_STOP,
    CTRL_EXIT
} ctrl_state_t;



// STRUCTURE OF GLOBAL CONTEXT
    typedef struct {
        QueueHandle_t ctrl_event_queue; //OK on init
        TimerHandle_t my_timer; //OK on int
        timer_event_context_t my_timer_context;//OK on init

        led_rgb_t leds;

        balance_control_t balance_control;

        drivebase_t drivebase;
        balance_loop_t balancer;
        mpu_reader_t *my_reader; //OK on init
        uint8_t mpu_task_counter;
        SemaphoreHandle_t ctrl_sync_sem; //OK on init
        mpu_values_t mpu_val;
        uint32_t version_read;
    } controller_ctx_t;
    //Led colors for states 
    rgb_t init_col={64, 0,0}; // Initial color for LED RGB
    rgb_t config_col={85, 25,0}; // Color for configuration state
    rgb_t ready_col={0, 64,0}; // Color for calibration state
    rgb_t error_col={32, 0,64}; // Color for error state
    static portMUX_TYPE mpu_lock=portMUX_INITIALIZER_UNLOCKED;

    static char msg_buf[64]; // Buffer for event messages
    

// Handler prototype
 //Functions processing a state
typedef ctrl_state_t (*ctrl_state_func_t)(controller_ctx_t*, ctrl_event_msg_t*);

typedef struct {
    ctrl_state_t state;
    ctrl_state_func_t handler;
} ctrl_state_handler_t;


//State handlers for each state
static ctrl_state_t ctrl_state_init(controller_ctx_t* ctx, ctrl_event_msg_t* event){
        ESP_LOGI(TAG, "State CTRL_INIT, Event :  %s", ctrl_event_to_str(event->type));
        ctrl_state_t next_state = CTRL_INIT;
        ctrl_event_msg_t new_event = { .type = EV_INIT_DONE, .err_code = ESP_OK, .msg=""};
    switch (event->type) {
        case EV_INIT:

            // Initialisation du contexte, des ressources, etc.
            ESP_LOGI(TAG, "Initialisation des ressources...");
            // Initialisation des ressources
                // capteur mpu6050...
            i2c_init();
            ctx->my_reader =mpu_reader_create();
            if (!ctx->my_reader) {
                new_event = (ctrl_event_msg_t){EV_ERROR, ESP_ERR_INVALID_ARG, "In CTRL_INIT, failed to initialize MPU reader"};
                next_state = CTRL_ERROR;
                break;
            } 

            //Create Semaphore for synchronized access to output values
            ctx->ctrl_sync_sem=xSemaphoreCreateBinary();
            if(!ctx->ctrl_sync_sem){
                new_event = (ctrl_event_msg_t){EV_ERROR, ESP_ERR_INVALID_ARG, "In CTRL_INIT, failed to create MPU semaphore"};
                next_state = CTRL_ERROR;
                break;
            }

            esp_err_t ret=mpu_reader_set_output_buffer(ctx->my_reader, &ctx->mpu_val,  &ctx->ctrl_sync_sem, &mpu_lock);
            if (ret!=ESP_OK) {
                new_event = (ctrl_event_msg_t){EV_ERROR, ESP_ERR_INVALID_ARG, "In CTRL_INIT, failed to initialize MPU reader"};
                next_state = CTRL_ERROR;
                break;
            }


            //Init balancing - See defines to adjust Kp, Ki and Kd of pid controler
            balance_control_init(&ctx->balance_control, ROBOT_PID_KP_DEFAULT, ROBOT_PID_KI_DEFAULT, ROBOT_PID_KD_DEFAULT, ROBOT_TARGET_ANGLE_DEFAULT, ROBOT_BALANCE_MAX_OUTPUT);   

            //Motor init 
            ret = drivebase_init(&ctx->drivebase);
            if (ret!=ESP_OK) {
                new_event = (ctrl_event_msg_t){EV_ERROR, ret, "In CTRL_INIT, failed to initialize motor drivebase"};
                next_state = CTRL_ERROR;
                break;
            }
                // leds
            led_rgb_init(&ctx->leds);    
            ctx->leds.led_rgb_queue=xQueueCreate(CTRL_QUEUE_SIZE, sizeof(rgb_t));
            if (!ctx->leds.led_rgb_queue) {
                new_event = (ctrl_event_msg_t){EV_ERROR, ESP_ERR_NO_MEM, "In CTRL_INIT, Failed to create LED RGB queue"};
                next_state = CTRL_ERROR;
                break;
            }

                //Init balance loop
            balance_loop_config_t config = {
                .my_reader= ctx->my_reader,
                .ctrl_sync_sem= ctx->ctrl_sync_sem,
                .ctrl_event_queue = ctx->ctrl_event_queue,
                .drivebase=&ctx->drivebase,
                .mpu_val=&ctx->mpu_val,
                .balance_control=&ctx->balance_control,
                .mpu_lock=&mpu_lock
            };
            ret = balance_loop_init(&ctx->balancer, &config);
            if (ret!=ESP_OK) {
                new_event = (ctrl_event_msg_t){EV_ERROR, ret, "In CTRL_INIT, failed to initialize balance_loop"};
                next_state = CTRL_ERROR;
                break;
            }
                //Timer to send delayed events
            ctx->my_timer_context.timer_event_queue=ctx->ctrl_event_queue;
            new_event = (ctrl_event_msg_t){EV_INIT_DONE, ret, NULL}; // Set the event to start
            next_state = CTRL_CONFIG;      

            break;

        default:
            snprintf(msg_buf, sizeof(msg_buf), "Unexpected event in CTRL_INIT state: %s", ctrl_event_to_str(event->type));
            new_event = (ctrl_event_msg_t){EV_ERROR, ESP_ERR_INVALID_STATE, msg_buf}; // Set the event to start
            next_state = CTRL_ERROR;
            break;
    }
                
    // Envoi l'événement "init terminé"
    if(ctx->ctrl_event_queue){
        xQueueSend(ctx->ctrl_event_queue, &new_event, 0);
    }

    return next_state;
}


static ctrl_state_t ctrl_state_config(controller_ctx_t* ctx, ctrl_event_msg_t* event){
    ESP_LOGI(TAG, "State CTRL_CONFIG, configuration of MPU6050... Event : %s", ctrl_event_to_str(event->type));
    ctrl_state_t next_state = CTRL_MPU_CALIBRATION;
    ctrl_event_msg_t new_event = { .type = EV_CONFIG_DONE, .err_code = ESP_OK, .msg=""};
    switch(event->type) {   
        case EV_INIT_DONE:

            //Create led blink task
            BaseType_t ret=xTaskCreate(led_rgb_task, "led_task", 2048, &ctx->leds, 5, &ctx->leds.led_task);
            if (ret != pdPASS) {
                    new_event = (ctrl_event_msg_t){EV_ERROR, ESP_ERR_NO_MEM, "In CTRL_CONFIG, Failed to create LED RGB Task"};
                    next_state = CTRL_ERROR;
                    xQueueSend(ctx->ctrl_event_queue, &new_event,(TickType_t) 2);
                    break;
            }

            // Set initial color
            if(xQueueSend(ctx->leds.led_rgb_queue, &init_col, (TickType_t) 2)!= pdPASS ){
                new_event = (ctrl_event_msg_t){EV_ERROR, ESP_ERR_INVALID_RESPONSE, "In CTRL_CONFIG, Failed to send color to LED RGB Queue"};
                next_state = CTRL_ERROR;
                xQueueSend(ctx->ctrl_event_queue, &new_event,(TickType_t) 2);
                break;
            }

            next_state = CTRL_MPU_CALIBRATION;
            ctx->my_timer_context.event_to_send=(ctrl_event_msg_t){EV_CONFIG_DONE, ESP_OK, NULL}; // Set the event to send when the timer expires
            // Start the timer for LED to blink for 5 seconds
            esp_err_t tmr_start = timer_ctrl_start(ctx->my_timer_context, 5000, NULL);
            if(tmr_start!=ESP_OK){
                 new_event = (ctrl_event_msg_t){EV_ERROR, ESP_ERR_INVALID_RESPONSE, "In CTRL_CONFIG, Failed to start the timer..."};
                next_state = CTRL_ERROR;
                xQueueSend(ctx->ctrl_event_queue, &new_event,(TickType_t) 2);
                break;
            }
            break;

        default:
            snprintf(msg_buf, sizeof(msg_buf), "Unexpected event in CTRL_CONFIG state: %s", ctrl_event_to_str(event->type));
            new_event = (ctrl_event_msg_t){EV_ERROR, ESP_ERR_INVALID_STATE, msg_buf}; // Set the event to start
            next_state = CTRL_ERROR;
            xQueueSend(ctx->ctrl_event_queue, &new_event,(TickType_t) 2);
            break;
    }
          
    return next_state;
}

static ctrl_state_t ctrl_state_calib_mpu(controller_ctx_t* ctx, ctrl_event_msg_t* event){
    ESP_LOGI(TAG, "State CTRL_MPU_CALIBRATION, calibration of MPU6050.., Event : %s", ctrl_event_to_str(event->type));
    ctrl_state_t next_state = CTRL_START_MPU;
    ctrl_event_msg_t new_event = { .type = EV_CALIB_DONE, .err_code = ESP_OK, .msg=""};
    switch(event->type) {
        case EV_CONFIG_DONE:
            //Calibrate and configurte mpu_6050
            mpu_config_t mpu_config = {
                .gyro = GYRO_CALIBRATE, // Gyroscope calibration mode
                .acce_fs = ACCE_FS_8G, // Accelerometer full scale range
                .gyro_fs = GYRO_FS_500DPS, // Gyroscope full scale range
                .sampling_config = {
                    .smplrt_div = 4, // No division, full sampling rate
                    .dlpf_cfg = MPU6050_DLPF_BW_20HZ // Low-pass filter bandwidth of 42 Hz
                }
            };
            esp_err_t ret = mpu_reader_config(ctx->my_reader, &mpu_config);
            if(ret != ESP_OK) {
                new_event = (ctrl_event_msg_t){EV_ERROR, ret, "State CALIB_MPU, Failed to configure MPU6050"};
                next_state = CTRL_ERROR;
                break;
            }

            // Configure interrupts
            ret = mpu_reader_int_config(ctx->my_reader, ROBOT_MPU_INT_PIN,FIFO_SOURCE_GYRO_ACC );
            if( ret != ESP_OK) {
                new_event = (ctrl_event_msg_t){EV_ERROR, ret, "State CALIB_MPU, Failed to configure interrupts on MPU6050"};
                next_state = CTRL_ERROR;
                break;
            }
            
            new_event = (ctrl_event_msg_t){EV_CALIB_DONE, ret, NULL};
            next_state = CTRL_START_MPU;
            break;
        default:
            snprintf(msg_buf, sizeof(msg_buf), "Unexpected event in CTRL_CALIB_MPU state: %s", ctrl_event_to_str(event->type));
            new_event = (ctrl_event_msg_t){EV_ERROR, ESP_ERR_INVALID_STATE, msg_buf}; // Set the event to start
            next_state = CTRL_ERROR;
            break;   
    }
    xQueueSend(ctx->ctrl_event_queue, &new_event, (TickType_t) 2);
    return next_state;
}

static ctrl_state_t ctrl_state_start_mpu(controller_ctx_t* ctx, ctrl_event_msg_t* event){
    ctrl_state_t next_state = CTRL_START_MPU;
    ctrl_event_msg_t new_event = { .type = EV_START_BALANCE, .err_code = ESP_OK, .msg=""};
    esp_err_t ret;
    switch(event->type) {
        case EV_ROBOT_LYING: 
            xQueueSend(ctx->leds.led_rgb_queue, &config_col, (TickType_t) 2); // Set config color (orange)
            next_state = CTRL_START_MPU; // Stay in the same state to wait for MPU data 
            new_event = (ctrl_event_msg_t){EV_MPU_STARTED, ESP_OK, NULL};
            xQueueSend(ctx->ctrl_event_queue, &new_event, (TickType_t) 2);
            break;
        case EV_CALIB_DONE:
            ESP_LOGI(TAG, "State CTRL_START_MPU, Start task for MPU6050.., Event : %s", ctrl_event_to_str(event->type));
            //Set LED color to Orange (config_col)
            if(xQueueSend(ctx->leds.led_rgb_queue, &config_col, (TickType_t) 2)!= pdPASS ){
                new_event = (ctrl_event_msg_t){EV_ERROR, ESP_ERR_INVALID_RESPONSE, "In CTRL_CONFIG, Failed to send color to LED RGB Queue"};
                next_state = CTRL_ERROR;
                break;
            }
            //Start reader and its tasks
            if (!ctx->my_reader) {
                new_event = (ctrl_event_msg_t){EV_ERROR, ESP_ERR_INVALID_STATE, "In CTRL_START_MPU, MPU reader not initialized"};
                next_state = CTRL_ERROR;
                break;
            }
            ret= mpu_reader_start(ctx->my_reader);
            if (ret != ESP_OK) {
                new_event = (ctrl_event_msg_t){EV_ERROR, ret, "In CTRL_START_MPU Failed to start MPU reader"};
                next_state = CTRL_ERROR;
            } 
            new_event = (ctrl_event_msg_t){EV_MPU_STARTED, ESP_OK, NULL}; // Set the event to start
            next_state = CTRL_START_MPU;
            xQueueSend(ctx->ctrl_event_queue, &new_event, (TickType_t) 2);
            break;
        case EV_MPU_STARTED:
            ESP_LOGI(TAG, "MPU reader started successfully");
            next_state = CTRL_START_MPU; // Stay in the same state to wait for MPU data 
            new_event = (ctrl_event_msg_t){EV_MPU_DATA_READY, ESP_OK, NULL }; // Set the event to start
            xQueueSend(ctx->ctrl_event_queue, &new_event, (TickType_t) 2);
            //Start timer to stop the FSM in case robot stays on the ground for more than 30s
            ctx->my_timer_context.event_to_send=(ctrl_event_msg_t){EV_ERROR, ESP_OK, "Timeout on Start_MPU"}; // Set the event to send when the timer expires
            timer_ctrl_start(ctx->my_timer_context, 30000, &ctx->my_timer);
            break;
        case EV_MPU_DATA_READY:
            uint32_t start_ver=0, end_ver=0;
            float roll=0, pitch=0;
            if(!mpu_reader_is_connected(ctx->my_reader)){
                next_state = CTRL_LOST_MPU;
                new_event = (ctrl_event_msg_t){EV_LOST_MPU, ESP_FAIL, "In CTRL_START_MPU Failed to connect to MPU reader"};
                xQueueSend(ctx->ctrl_event_queue, &new_event, (TickType_t) 5);
                break;
            } 
            //Restart counter of reconnection attempts
            ctx->mpu_task_counter=0;
            if(xSemaphoreTake(ctx->ctrl_sync_sem, pdMS_TO_TICKS(20))==pdTRUE){
                taskENTER_CRITICAL(&mpu_lock);
                start_ver = ctx->mpu_val.version;
                roll = ctx->mpu_val.kalman_filter.xk[0]; // Use Kalman filter roll angle
                end_ver = ctx->mpu_val.version;
                taskEXIT_CRITICAL(&mpu_lock);
                ESP_LOGI(TAG, "Output buffer updated, version: %lu", (long unsigned int)end_ver);
                
                if((start_ver != end_ver) || (start_ver & 1)){
                    new_event = (ctrl_event_msg_t){EV_MPU_DATA_READY, ESP_OK, NULL}; // Set the event to start
                    next_state =CTRL_START_MPU;
                    xQueueSend(ctx->ctrl_event_queue, &new_event, (TickType_t) 2);
                    break;
                }
                ctx->version_read=end_ver;
            }

            if(roll > ROBOT_MIN_BALANCE_ANGLE && roll < ROBOT_MAX_BALANCE_ANGLE){
                timer_ctrl_cancel(ctx->my_timer);
                ESP_LOGI(TAG, "Thershold over 20° Roll : %.2f, Pitch %.2f", roll, pitch);
                new_event = (ctrl_event_msg_t){EV_START_BALANCE, ESP_OK, NULL}; // Set the event to start
                next_state =CTRL_BALANCING;
                //TODO: Instrad, launch the event with a delay (i.e. 300ms) to let the MPU reader update the values
                xQueueSend(ctx->ctrl_event_queue, &new_event, (TickType_t) 2);
                break;
            } 

            next_state = CTRL_START_MPU;
            ctx->my_timer_context.event_to_send=(ctrl_event_msg_t){EV_MPU_DATA_READY, ESP_OK, NULL}; // Set the event to send when the timer expires
            ret = timer_ctrl_start(ctx->my_timer_context, 1000, NULL);
            if(ret!=ESP_OK){
                new_event = (ctrl_event_msg_t){EV_ERROR, ESP_ERR_INVALID_RESPONSE, "In CTRL_CONFIG, Failed to start the timer..."};
                next_state = CTRL_ERROR;
                xQueueSend(ctx->ctrl_event_queue, &new_event,(TickType_t) 2);
                break;
            }
        break;

        default:
            snprintf(msg_buf, sizeof(msg_buf), "Unexpected event in CTRL_START_MPU state: %s", ctrl_event_to_str(event->type));
            new_event = (ctrl_event_msg_t){EV_ERROR, ESP_ERR_INVALID_STATE, msg_buf }; // Set the event to start
            next_state = CTRL_ERROR;
            xQueueSend(ctx->ctrl_event_queue, &new_event, (TickType_t) 2);
        break;
    }             
    return next_state;
}

static ctrl_state_t ctrl_state_lost_mpu(controller_ctx_t* ctx, ctrl_event_msg_t* event){
    ctrl_state_t next_state = CTRL_START_MPU;
    ctrl_event_msg_t new_event = { .type = EV_START_BALANCE, .err_code = ESP_OK, .msg=""};
    esp_err_t ret;
    switch(event->type) {
        case  EV_LOST_MPU:
                next_state = CTRL_START_MPU;
                ret=mpu_reader_resume(ctx->my_reader);
                if(ret!=ESP_OK){
                    new_event = (ctrl_event_msg_t){EV_ERROR, ret, "In CTRL_START_MPU Failed to restart MPU reader"};
                    next_state = CTRL_ERROR;
                    xQueueSend(ctx->ctrl_event_queue, &new_event, (TickType_t) 5);
                    break;
                }

                ctx->mpu_task_counter++;
                if(ctx->mpu_task_counter==MAX_MPU_RESTART){
                    new_event = (ctrl_event_msg_t){EV_ERROR, ret, "In CTRL_START_MPU Failed to restart MPU reader"};
                    next_state = CTRL_ERROR;
                    xQueueSend(ctx->ctrl_event_queue, &new_event, (TickType_t) 5);
                    break;
                }


                // Start the timer to give time to reconnect 1 second
                ctx->my_timer_context.event_to_send=(ctrl_event_msg_t){EV_MPU_DATA_READY, ESP_OK, NULL}; // Set the event to send when the timer expires
                ret = timer_ctrl_start(ctx->my_timer_context, 500, NULL);
                if(ret!=ESP_OK){
                    new_event = (ctrl_event_msg_t){EV_ERROR, ESP_ERR_INVALID_RESPONSE, "In CTRL_CONFIG, Failed to start the timer..."};
                    next_state = CTRL_ERROR;
                    xQueueSend(ctx->ctrl_event_queue, &new_event,(TickType_t) 2);
                    break;
                }
            break;

        default:
            snprintf(msg_buf, sizeof(msg_buf), "Unexpected event in CTRL_LOST_MPU state: %s", ctrl_event_to_str(event->type));
            new_event = (ctrl_event_msg_t){EV_ERROR, ESP_ERR_INVALID_STATE, msg_buf }; // Set the event to start
            next_state = CTRL_ERROR;
            xQueueSend(ctx->ctrl_event_queue, &new_event, (TickType_t) 2);
            break;
        }
        return next_state;

}


static ctrl_state_t ctrl_state_balancing(controller_ctx_t* ctx, ctrl_event_msg_t* event){
    ctrl_event_msg_t new_event={.type=EV_NONE, .err_code=ESP_OK, .msg=NULL};
    ctrl_state_t next_state = CTRL_BALANCING;

     switch(event->type) {
        case EV_START_BALANCE:
            xQueueSend(ctx->leds.led_rgb_queue, &ready_col, (TickType_t) 2); 
            ESP_LOGI(TAG, "State CTRL_BALANCING, Apply speed to motor, Event : %s", ctrl_event_to_str(event->type));
            next_state=CTRL_BALANCING;

            esp_err_t ret = balance_loop_start(&ctx->balancer);
            if(ret!=ESP_OK){
                new_event = (ctrl_event_msg_t){EV_ERROR, ret, "In CTRL_BALANCING, failed to start the balance loop task"};
                next_state = CTRL_ERROR;
                xQueueSend(ctx->ctrl_event_queue, &new_event, (TickType_t) 2);
                break;
            }
            break;
        case EV_STOP_BALANCING:
            next_state = CTRL_START_MPU;
            new_event = (ctrl_event_msg_t){EV_ROBOT_LYING, ESP_OK, NULL};
    
            //TODO Deal with error from stop method
            drivebase_stop(&ctx->drivebase);
            xQueueSend(ctx->ctrl_event_queue, &new_event, (TickType_t) 2);
            
            break;

        case EV_LOST_MPU:
            next_state = CTRL_LOST_MPU;
            new_event = (ctrl_event_msg_t){EV_LOST_MPU, ESP_OK, NULL};
            //TODO Deal with error from stop method
            drivebase_stop(&ctx->drivebase);
            xQueueSend(ctx->ctrl_event_queue, &new_event, (TickType_t) 2);
            break;
        case EV_ERROR:
            next_state = CTRL_ERROR;
            new_event = (ctrl_event_msg_t){EV_ERROR, event->err_code, event->msg};
            xQueueSend(ctx->ctrl_event_queue, &new_event, (TickType_t) 2);
            break;
        default:
            snprintf(msg_buf, sizeof(msg_buf), "Unexpected event in CTRL_NEW_MEASUREMNT state: %s", ctrl_event_to_str(event->type));
            new_event = (ctrl_event_msg_t){EV_ERROR, ESP_ERR_INVALID_STATE, msg_buf };
            next_state = CTRL_ERROR;
            xQueueSend(ctx->ctrl_event_queue, &new_event, (TickType_t) 2);
            break;
    }
    return next_state;
}



static ctrl_state_t ctrl_state_error(controller_ctx_t* ctx, ctrl_event_msg_t* event){
    ESP_LOGE(TAG, "Error occurred!");
    ctrl_state_t next_state = CTRL_STOP;
    // Handle error state
    switch(event->type) {
        case EV_ERROR:
            if (event->err_code != ESP_OK) {
                ESP_LOGE(TAG, "Error code: %s, %s", esp_err_to_name(event->err_code), event->msg ? event->msg : "No message");
            } else {
                ESP_LOGE(TAG, "An error event occurred with error code : %s", esp_err_to_name(event->err_code));
            }
            break;
        default:
            ESP_LOGE(TAG, "Unexpected event in CTRL_ERROR state: %s", ctrl_event_to_str(event->type));
            break;
    }
    xQueueSend(ctx->leds.led_rgb_queue, &error_col, (TickType_t) 2); // Set error color purple
    
    ctx->my_timer_context.event_to_send=(ctrl_event_msg_t){EV_STOP, ESP_OK, NULL}; // Set the event to send when the timer expires
    esp_err_t  ret = timer_ctrl_start(ctx->my_timer_context, 5000, NULL);
    if(ret!=ESP_OK){
        ctrl_event_msg_t new_event=(ctrl_event_msg_t){EV_STOP, ret, NULL};
        xQueueSend(ctx->ctrl_event_queue, &new_event, (TickType_t) 5);
    }

    //xTimerStart(ctx->my_timer,0); // Start the timer for LED to blink for 5 seconds
    return next_state;
}

static ctrl_state_t ctrl_state_stop(controller_ctx_t* ctx, ctrl_event_msg_t* event){

     ctrl_state_t next_state = CTRL_STOP;
    if(event->type == EV_STOP){
        ESP_LOGI(TAG, "Stopping controller...");
             //Free the reader's memory
        if (ctx->my_reader) {
            mpu_reader_stop(ctx->my_reader); // Stop the MPU reader
            if(ctx->ctrl_event_queue){
                ESP_LOGI(TAG, "Deleting user queue...");
                vQueueDelete(ctx->ctrl_event_queue); // Delete the user queue
                ctx->ctrl_event_queue =NULL; 
            }
            
            ctx->my_reader = NULL; // Clear the pointer
        } else {
            free(ctx->my_reader); // Free the memory if my_reader is NULL
        }

        //DeInit motor controller
        //TODO Deal with error from deinit method
        //Deinit stops and deinit pointers
         drivebase_deinit(&ctx->drivebase);

        //Stops the leds
        ctx->leds.running = false; // Set the LED task running flag to false
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay to ensure the LED task stops

        //Stops the TImer
        timer_ctrl_stop_all();
        ctx->my_timer = NULL; // Clear the timer pointer
        ESP_LOGI(TAG, "Controller stopped.");
    
        next_state= CTRL_EXIT;
    }
    return next_state;
}


static const ctrl_state_handler_t state_handlers[] = {
    { CTRL_INIT,           ctrl_state_init },
    { CTRL_CONFIG,         ctrl_state_config },
    { CTRL_MPU_CALIBRATION, ctrl_state_calib_mpu},
    { CTRL_START_MPU,      ctrl_state_start_mpu },
    { CTRL_LOST_MPU,        ctrl_state_lost_mpu},
    //{ CTRL_BALANCING_INIT,      ctrl_state_balancing_init},
    //{ CTRL_NEW_MEASUREMENT, ctrl_state_new_meas},
    { CTRL_BALANCING, ctrl_state_balancing},
    { CTRL_ERROR,          ctrl_state_error },
    { CTRL_STOP,           ctrl_state_stop },
    // Ajoute tes autres états ici
};

#define NUM_HANDLERS (sizeof(state_handlers)/sizeof(state_handlers[0]))

// === 6. Boucle principale de la FSM ===

void controller_task(void *pvParam){
    controller_ctx_t ctx = {0}; // Zero-initialisation

    // Initialiser le contexte ici si besoin (queues, etc.)
    ctx.ctrl_event_queue = xQueueCreate(CTRL_QUEUE_SIZE, sizeof(ctrl_event_msg_t));

    //If no queue, don't send an event and straight to ERROR
    if (!ctx.ctrl_event_queue){
        ESP_LOGE(TAG, "Failed to create event queue (to store event from controller FSM)");
        vTaskDelete(NULL);
        return;
    } 




    ctrl_state_t state = CTRL_INIT;
    ctrl_event_msg_t event = {EV_INIT, ESP_OK, NULL};
    xQueueSend(ctx.ctrl_event_queue, &event, portMAX_DELAY);


    while(1) {
        // Attendre un événement
        xQueueReceive(ctx.ctrl_event_queue, &event, portMAX_DELAY);
         bool handled = false;
        // Chercher et appeler le bon handler d'état
                  
            for (size_t i=0; i<NUM_HANDLERS; ++i) {
                if (state_handlers[i].state == state) {
                    state = state_handlers[i].handler(&ctx, &event);
                    handled = true;
                    break;
                }
            }
            if (!handled) {
                ESP_LOGE(TAG, "Etat inconnu: %d", state);
                state = CTRL_ERROR;
            }
            if (state == CTRL_EXIT) {
                ESP_LOGI(TAG, "FSM terminé, suppression de la tâche...");
                break; // quitte le while(1)
            }

    }
    vTaskDelete(NULL);
}

