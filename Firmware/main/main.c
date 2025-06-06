/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "stepper_motor_encoder.h"
#include "driver/ledc.h"
#include "esp_err.h"



#include "esp_system.h"
#include "driver/uart.h"

#include <inttypes.h>
#include "nvs_flash.h"
#include "nvs.h"

#include "cJSON.h"
#include "esp_log.h"


//QUEUE HANDLING
#include "freertos/queue.h"
#define QUEUE_SIZE 10    // Store up to 10 messages  
#define MESSAGE_SIZE 128 // Max message length  

static QueueHandle_t uart2_queue;  // Queue for UART2 messages 


// Motor 1
#define MOTOR1_STEP_MOTOR_GPIO_EN       14
#define MOTOR1_STEP_MOTOR_GPIO_DIR      12
#define MOTOR1_STEP_MOTOR_GPIO_STEP     13
#define MOTOR1_STEP_MOTOR_ENABLE_LEVEL  0 // DRV8825 is enabled on low level
#define MOTOR1_STEP_MOTOR_DISABLE_LEVEL 1 // DRV8825 is enabled on low level
#define MOTOR1_STEP_MOTOR_SPIN_DIR_CLOCKWISE 0
#define MOTOR1_STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE !MOTOR1_STEP_MOTOR_SPIN_DIR_CLOCKWISE
#define MOTOR1_STEP_MOTOR_RESOLUTION_HZ 1000000 // 1MHz resolution

// Motor 2
#define MOTOR2_STEP_MOTOR_GPIO_EN       6
#define MOTOR2_STEP_MOTOR_GPIO_DIR      4
#define MOTOR2_STEP_MOTOR_GPIO_STEP     5
#define MOTOR2_STEP_MOTOR_ENABLE_LEVEL  0 // DRV8825 is enabled on low level
#define MOTOR2_STEP_MOTOR_DISABLE_LEVEL 1 // DRV8825 is enabled on low level

#define MOTOR2_STEP_MOTOR_SPIN_DIR_CLOCKWISE 1
#define MOTOR2_STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE !MOTOR2_STEP_MOTOR_SPIN_DIR_CLOCKWISE
#define MOTOR2_STEP_MOTOR_RESOLUTION_HZ 1000000 // 1MHz resolution


#define TXD1_PIN (GPIO_NUM_7)
#define RXD1_PIN (GPIO_NUM_8)
#define TXD2_PIN (GPIO_NUM_15)
#define RXD2_PIN (GPIO_NUM_16)
#define LEDC_OUTPUT_IO1          (2)
#define LEDC_OUTPUT_IO2          (3)

#define EN_PIN1 (GPIO_NUM_36)
#define EN_PIN2 (GPIO_NUM_37)

#define STOP 0
#define RUN  1

#define FORWORD    2
#define BACKWORD    3

#define RIGHT 4
#define LEFT 5

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (5) // Define the output GPIO
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (0) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (200) // Frequency in Hertz. Set frequency at 4 kHz

#define LEDC_TEST_CH_NUM       (2)
// uint32_t DutyCycle[LEDC_TEST_CH_NUM] = {1024,2048,1024,2048};
// uint32_t DutyCycle[LEDC_TEST_CH_NUM] = {512,1024,2048,3072,4096,5120,6144,7168};
volatile int32_t DutyCycle = 1024;

typedef struct {
	float RollFactor;    
	float PitchFactor;   
	float YawFactor;     
	float ThrottleFactor;   
	float ForwardFactor;   
	float LateralFactor;
} variable;

//====================================================================================================================
//Motor #    	 Roll Factor    Pitch Factor    Yaw Factor     Throttle Factor    Forward Factor    Lateral Factor  

float fmotor1[6]={	0,			0,			    1,  			0,					1,					0};
float fmotor2[6]={	0,			0,				-1,				0,					1,					0};
//=====================================================================================================================
volatile double motorOut[2]= {0,0};
volatile double motorPulsOut[2]= {0,0};
double motorPrvPulsOut[2]= {0,0};

float motorDir[2]= {0,0};
float motorPrevDir[2]= {0,0};
double setpoint[6]={0,0,0,0,0,0},output[6]={0,0,0,0,0,0},pwmoutput[6]={0,0,0,0,0,0} ;
int position[3]= {0,0,0};


#define RX_BUF_SIZE        256
#define LINE_BUF_SIZE      256
// static const int TX_BUF_SIZE = 8;
char buffer[17];
char responseData[1024];
volatile int32_t SPEED = 200;
volatile bool armed = false;

volatile int cmdFlag = 1, state = 0,previousState = 0;;

volatile bool flagMotor1RUN = 0,flagMotor2RUN = 0,errorFlag = 0,flagCamRUN = 0,serialFlag = 0, accelFlag = 1;

static const char *TAG = "example";
static const char *MOTOR_TAG = "MOTOR";
rmt_channel_handle_t motor1_chan = NULL;
rmt_channel_handle_t motor2_chan = NULL;
nvs_handle_t my_handle;

int sendData(const char* logName, const char* data);
int sendData1(const char* logName, const char* data);

void motorInit(void);
void motorRunFwd(void);
void motorRunBwd(void);
void motor1RunFwd(void);
void motor2RunFwd(void);
void motorSTOP(void);
void saveSPEED(void);
void saveBRIGHT(void);
#define axes_size 8
double array_axes[axes_size]={};

#include "cJSON.h"
#include "esp_log.h"

#define threshold = 0.1;
// Example JSON string received
const char *received_json = "{\"op\": \"publish\", \"topic\": \"/joystick_input\", \"msg\": {\"header\": {\"stamp\": {\"sec\": 1729144145, \"nanosec\": 169086574}, \"frame_id\": \"\"}, \"axes\": [0.0, 0.0, 0.0, -0.49803921580314636], \"buttons\": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}}";


void updateMotor(void)
{   
    int i,j;
 
    setpoint [2] = array_axes[2];
    setpoint [4] = array_axes[1];
    motorOut[0] = 0;
    motorOut[1] = 0;
    for(j=0; j<6; j++)
	{
		motorOut[0]+=fmotor1[j]*setpoint[j];
       
	}


    ESP_LOGI("MotorOut", "Out: %lf", motorOut[0]);
	for(j=0; j<6; j++)
	{
		motorOut[1]+=fmotor2[j]*setpoint[j];
       
	}

    ESP_LOGI("MotorOut", "Out: %lf", motorOut[1]);
    if(motorOut[0]>0)
    {
        motorPulsOut[0]=(int32_t)(motorOut[0]*600);
        ESP_LOGI("1FORWORD", "speed: %lf", motorPulsOut[0]);
        motorDir[0] = FORWORD;
    }
    else
    {
        motorPulsOut[0]=(int32_t)(-1*motorOut[0]*600);
        ESP_LOGI("1BACKWORD", "speed: %lf", motorPulsOut[0]);
        motorDir[0] = BACKWORD;
    }
    if(motorOut[1]>0)
    {
        motorPulsOut[1]=(int32_t)(motorOut[1]*600);
        ESP_LOGI("2FORWORD", "speed: %lf", motorPulsOut[1]);
        motorDir[1] = FORWORD;
    }
    else
    {
        motorPulsOut[1]=(int32_t)(-1*motorOut[1]*600);
        ESP_LOGI("2BACKWORD", "speed: %lf", motorPulsOut[1]);
        motorDir[1] = BACKWORD;
    }
    if (motorPulsOut[0] >600)
        motorPulsOut[0] =600;
    if (motorPulsOut[1] >600)
        motorPulsOut[1] =600;
    

}

// Function to decode JSON string and extract joystick data
void decode_joystick_json(const char *json_string)
{
    // Parse the JSON string into a cJSON object
    cJSON *root = cJSON_Parse(json_string);
    if (root == NULL) {
        ESP_LOGE("Joystick", "Error parsing JSON");
        return;
    }

    // Extract the topic field
    cJSON *topic = cJSON_GetObjectItem(root, "topic");
    if (topic && cJSON_IsString(topic)) {
        ESP_LOGD("Joystick", "Topic: %s", topic->valuestring);
        
        // Check if the topic is "/joystick_input"
        if (strcmp(topic->valuestring, "/joystick_input") != 0) {
            ESP_LOGD("Joystick", "Received message on an unexpected topic: %s", topic->valuestring);
            cJSON_Delete(root);
            return;  // Exit function if topic does not match
        }
    } else {
        ESP_LOGE("Joystick", "No valid topic found");
        cJSON_Delete(root);
        return;
    }

    // Extract the "msg" object
    cJSON *msg = cJSON_GetObjectItem(root, "msg");
    if (msg == NULL) {
        ESP_LOGE("Joystick", "No 'msg' object found.");
        cJSON_Delete(root);
        return;
    }

    // Extract header (timestamp and frame_id)
    cJSON *header = cJSON_GetObjectItem(msg, "header");
    if (header) {
        cJSON *stamp = cJSON_GetObjectItem(header, "stamp");
        if (stamp) {
            cJSON *sec = cJSON_GetObjectItem(stamp, "sec");
            cJSON *nanosec = cJSON_GetObjectItem(stamp, "nanosec");
            if (sec && cJSON_IsNumber(sec) && nanosec && cJSON_IsNumber(nanosec)) {
                ESP_LOGD("Joystick", "Timestamp: %d sec, %d nanosec", sec->valueint, nanosec->valueint);
            }
        }
        cJSON *frame_id = cJSON_GetObjectItem(header, "frame_id");
        if (frame_id && cJSON_IsString(frame_id)) {
            ESP_LOGD("Joystick", "Frame ID: %s", frame_id->valuestring);
        }
    }

    // Extract axes array
    cJSON *axes = cJSON_GetObjectItem(msg, "axes");
    if (axes && cJSON_IsArray(axes)) {
        // int axes_size = cJSON_GetArraySize(axes);
        ESP_LOGD("Joystick", "Axes:");
        for (int i = 0; i < axes_size; i++) {
            cJSON *axis_value = cJSON_GetArrayItem(axes, i);
            if (axis_value && cJSON_IsNumber(axis_value)) {
                // ESP_LOGI("Joystick", "  Axis %d: %f", i, axis_value->valuedouble);
                array_axes[i]=axis_value->valuedouble;
                ESP_LOGI("Joystick", "  Axis %d: %f", i, array_axes[i]);
            }
        }
    }

    // Extract buttons array
    cJSON *buttons = cJSON_GetObjectItem(msg, "buttons");
    if (buttons && cJSON_IsArray(buttons)) {
        int buttons_size = cJSON_GetArraySize(buttons);
        ESP_LOGD("Joystick", "Buttons:");
        for (int i = 0; i < buttons_size; i++) {
            cJSON *button_value = cJSON_GetArrayItem(buttons, i);
            if (button_value && cJSON_IsNumber(button_value)) {
                ESP_LOGD("Joystick", "  Button %d: %d", i, button_value->valueint);
            }
        }
    }
    serialFlag=1;
    updateMotor();
    // Clean up and free the memory used by the cJSON object
    cJSON_Delete(root);
}




void initUART(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
}
void initUART1(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD1_PIN, RXD1_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}
void initUART2(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TXD2_PIN, RXD2_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_0, data, len);
    // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}
int sendData1(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}
// int sendData1(const char* logName, unsigned const char* data,const int len)
// {
//     // const int len = strlen(data);
//     const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
//     // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
//     return txBytes;
// }
int sendData2(const char* logName, unsigned const char* data,const int len)
{
    // const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_2, data, len);
    // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}
static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        sendData(TX_TASK_TAG, "Hello world");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

static int uart_read_line(uart_port_t uart_num, uint8_t *data, size_t max_len, int timeout_ms) {
    size_t len = 0;
    uint8_t byte;
    
    while (len < max_len - 1) {  // Reserve space for null terminator
        int read_len = uart_read_bytes(uart_num, &byte, 1, pdMS_TO_TICKS(timeout_ms));
        
        if (read_len > 0) {
            if (byte == '\n') {  // End of line detected
                break;
            }
            data[len++] = byte;
        } else {
            // Timeout or no data
            break;
        }
    }
    data[len] = '\0';  // Null-terminate the string
    return len;
}




static void rx_task(void *arg) {
    static const char *RX_TASK_TAG = "CONFIGURE";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);

    uint8_t *line_data = (uint8_t *) malloc(LINE_BUF_SIZE + 1);
    if (line_data == NULL) {
        ESP_LOGE(RX_TASK_TAG, "Failed to allocate memory for line buffer");
        return;
    }

    while (1) {
        int line_len = uart_read_line(UART_NUM_0, line_data, LINE_BUF_SIZE, 200);

        if (line_len > 0) {
            ESP_LOGI(RX_TASK_TAG, "Read line: '%s'", line_data);
            char dataArray [1024];
            memcpy(dataArray,line_data, line_len);

            // Process the line data as needed
            // Example: Decode JSON or process the command
            decode_joystick_json((char *)dataArray);

        }
        //  vTaskDelay(10/ portTICK_PERIOD_MS);
    }
    free(line_data);

}

static void rx_task1(void *arg) {
    static const char *RX_TASK_TAG = "CONFIGURE";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);

    uint8_t *line_data = (uint8_t *) malloc(LINE_BUF_SIZE + 1);
    if (line_data == NULL) {
        ESP_LOGE(RX_TASK_TAG, "Failed to allocate memory for line buffer");
        return;
    }

    while (1) {
        int line_len = uart_read_line(UART_NUM_1, line_data, LINE_BUF_SIZE, 200);

        if (line_len > 0) {
            ESP_LOGI(RX_TASK_TAG, "Read line: '%s'", line_data);
            char dataArray [1024];
            memcpy(dataArray,line_data, line_len);

            // Process the line data as needed
            // Example: Decode JSON or process the command
            decode_joystick_json((char *)dataArray);
        }
        //  vTaskDelay(10/ portTICK_PERIOD_MS);
    }
    free(line_data);

    
}

static void rx_task2(void *arg) {  
    static const char *RX_TASK_TAG = "UART2_RX";  
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);  

    uint8_t *line_data = (uint8_t *) malloc(LINE_BUF_SIZE + 1);  
    if (line_data == NULL) {  
        ESP_LOGE(RX_TASK_TAG, "Failed to allocate memory for line buffer");  
        return;  
    }  

    while (1) {  
        int line_len = uart_read_line(UART_NUM_2, line_data, LINE_BUF_SIZE, 200);  

        if (line_len > 0) {  
            ESP_LOGI(RX_TASK_TAG, "Received from UART2: '%s'", line_data);  

            // Create a buffer to store message in the queue  
            char *msg = malloc(line_len + 1);  
            if (msg) {  
                memcpy(msg, line_data, line_len);  
                msg[line_len] = '\0';  // Null-terminate the string  

                // Send the message to the queue (wait up to 100ms if full)  
                if (xQueueSend(uart2_queue, &msg, pdMS_TO_TICKS(100)) != pdPASS) {  
                    ESP_LOGE(RX_TASK_TAG, "Failed to enqueue message");  
                    free(msg);  
                }  
            }  
        }  
    }  
    free(line_data);  
}



static void uart2_queue_processor_task(void *arg) {  
    static const char *TASK_TAG = "UART2_QUEUE_PROCESSOR";  
    esp_log_level_set(TASK_TAG, ESP_LOG_INFO);  

    char *msg;  

    while (1) {  
        // Wait indefinitely for a message in the queue  
        if (xQueueReceive(uart2_queue, &msg, portMAX_DELAY)) {  
            ESP_LOGI(TASK_TAG, "Processing UART2 message: '%s'", msg);  

            // Transmit this message through UART0 or any other destination  
            sendData(TASK_TAG, (unsigned char *)msg, strlen(msg));  

            // Free the message buffer after transmission  
            free(msg);  
        }  
    }  
}

void pwmLEDC(void *arg)
{
    
    int ch;
    int32_t CurrentDuty = 0;

    // Set the LEDC peripheral configuration
        // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
        {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO1 ,
        .duty           = LEDC_DUTY, // Set duty to 0%
        .hpoint         = 0
        },        
        {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_1,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO2 ,
        .duty           = LEDC_DUTY, // Set duty to 0%
        .hpoint         = 0
        }        
    };
    ESP_LOGI(TAG, "Init LED ");
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }
    ESP_LOGI(TAG, "Setting LED duty cycle");
    //   for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
    //         ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, ledc_channel[ch].channel, DutyCycle));
    //         // Update duty to apply the new value
    //         ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, ledc_channel[ch].channel));

    //   } 
    CurrentDuty=DutyCycle;
    while (true){
        if (DutyCycle!=CurrentDuty)
        {
            for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, ledc_channel[ch].channel, DutyCycle));
            // Update duty to apply the new value
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, ledc_channel[ch].channel));
            }
            CurrentDuty= DutyCycle;
        }
            vTaskDelay(10/ portTICK_PERIOD_MS);
            // ESP_LOGI(TAG, "LEDC task");

    }
}



void saveSPEED()
{
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        // printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        // printf("Done\n");

    err = nvs_set_i32(my_handle, "SPEED",SPEED);
    // printf((err != ESP_OK) ? "Failed!\n" : "Done\n");


    // printf("Committing updates in NVS ... ");
    err = nvs_commit(my_handle);
    // printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

    // Close
    nvs_close(my_handle);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    }
}
void saveBRIGHT()
{
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        // printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        // printf("Done\n");

    err = nvs_set_i32(my_handle, "BRIGHT",DutyCycle);
    // printf((err != ESP_OK) ? "Failed!\n" : "Done\n");


    // printf("Committing updates in NVS ... ");
    err = nvs_commit(my_handle);
    // printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

    // Close
    nvs_close(my_handle);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    }
}




static void stepper_motor_run(void *pvParameter)
{

    // ===========Init stepper motor ==========================================

        const static uint32_t accel_samples = 100;
        const static uint32_t decel_samples = 100;

        
        ESP_LOGI(TAG, "Initialize EN + DIR GPIO for Motor 1");
        gpio_config_t motor_en_dir_gpio_config = {
            .mode = GPIO_MODE_OUTPUT,
            .intr_type = GPIO_INTR_DISABLE,
            .pin_bit_mask = ((1ULL << MOTOR1_STEP_MOTOR_GPIO_DIR) | (1ULL << MOTOR1_STEP_MOTOR_GPIO_EN)|(1ULL << MOTOR2_STEP_MOTOR_GPIO_DIR)),
        };
        ESP_ERROR_CHECK(gpio_config(&motor_en_dir_gpio_config));


        ESP_LOGI(TAG, "Create RMT TX channel for Motor 1");
        rmt_tx_channel_config_t motor1_tx_chan_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .gpio_num = MOTOR1_STEP_MOTOR_GPIO_STEP,
            .mem_block_symbols = 64,
            .resolution_hz = MOTOR1_STEP_MOTOR_RESOLUTION_HZ,
            .trans_queue_depth = 10,
        };
        ESP_ERROR_CHECK(rmt_new_tx_channel(&motor1_tx_chan_config, &motor1_chan));

        ESP_LOGI(TAG, "Create RMT TX channel for Motor 2");
        rmt_tx_channel_config_t motor2_tx_chan_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .gpio_num = MOTOR2_STEP_MOTOR_GPIO_STEP,
            .mem_block_symbols = 64,
            .resolution_hz = MOTOR2_STEP_MOTOR_RESOLUTION_HZ,
            .trans_queue_depth = 10,
        };
        ESP_ERROR_CHECK(rmt_new_tx_channel(&motor2_tx_chan_config, &motor2_chan));

        // ESP_LOGI(TAG, "Enable Motor 1");
        // gpio_set_level(MOTOR1_STEP_MOTOR_GPIO_EN, MOTOR1_STEP_MOTOR_ENABLE_LEVEL);

        // ESP_LOGI(TAG, "Enable Motor 2");
        // gpio_set_level(MOTOR2_STEP_MOTOR_GPIO_EN, MOTOR2_STEP_MOTOR_ENABLE_LEVEL);

        ESP_LOGI(TAG, "Create motor encoders for Motor 1");
        stepper_motor_curve_encoder_config_t motor1_accel_encoder_config = {
            .resolution = MOTOR1_STEP_MOTOR_RESOLUTION_HZ,
            .sample_points = 10,
            .start_freq_hz = 0,
            .end_freq_hz = 100,
        };
        rmt_encoder_handle_t motor1_accel_encoder = NULL;
        ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&motor1_accel_encoder_config, &motor1_accel_encoder));

        stepper_motor_uniform_encoder_config_t motor1_uniform_encoder_config = {
            .resolution = MOTOR1_STEP_MOTOR_RESOLUTION_HZ,
        };
        rmt_encoder_handle_t motor1_uniform_encoder = NULL;
        ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&motor1_uniform_encoder_config, &motor1_uniform_encoder));

        stepper_motor_curve_encoder_config_t motor1_decel_encoder_config = {
            .resolution = MOTOR1_STEP_MOTOR_RESOLUTION_HZ,
            .sample_points = 10,
            .start_freq_hz = 100,
            .end_freq_hz = 0,
        };
        rmt_encoder_handle_t motor1_decel_encoder = NULL;
        ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&motor1_decel_encoder_config, &motor1_decel_encoder));

        ESP_LOGI(TAG, "Create motor encoders for Motor 2");
        stepper_motor_curve_encoder_config_t motor2_accel_encoder_config = {
            .resolution = MOTOR2_STEP_MOTOR_RESOLUTION_HZ,
            .sample_points = 20,
            .start_freq_hz = 0,
            .end_freq_hz = 100,
        };
        rmt_encoder_handle_t motor2_accel_encoder = NULL;
        ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&motor2_accel_encoder_config, &motor2_accel_encoder));

        stepper_motor_uniform_encoder_config_t motor2_uniform_encoder_config = {
            .resolution = MOTOR2_STEP_MOTOR_RESOLUTION_HZ,
        };
        rmt_encoder_handle_t motor2_uniform_encoder = NULL;
        ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&motor2_uniform_encoder_config, &motor2_uniform_encoder));

        stepper_motor_curve_encoder_config_t motor2_decel_encoder_config = {
            .resolution = MOTOR2_STEP_MOTOR_RESOLUTION_HZ,
            .sample_points = 20,
            .start_freq_hz = 100,
            .end_freq_hz = 0,
        };
        rmt_encoder_handle_t motor2_decel_encoder = NULL;
        ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&motor2_decel_encoder_config, &motor2_decel_encoder));

        ESP_LOGI(TAG, "Enable RMT channels");
        ESP_ERROR_CHECK(rmt_enable(motor1_chan));
        ESP_ERROR_CHECK(rmt_enable(motor2_chan));

        ESP_LOGI(TAG, "Spin motors for 6000 steps: 500 accel + 5000 uniform + 500 decel");
        rmt_transmit_config_t tx_config = {
            .loop_count = 0,
        };
    //=========================================================================
    //==================wait for serial events============================================


    while(true) 
    {   
        int gain = 1;
        uint32_t uniform_speed1_hz = (uint32_t)(gain * motorPulsOut[0]);
        uint32_t uniform_speed2_hz = (uint32_t)(gain * motorPulsOut[1]);
        // ESP_LOGI(TAG, "main loop");
        if((motorPrevDir[0] != motorDir[0])||(motorPrevDir[1] != motorDir[1]))
        {

            if((1==flagMotor1RUN)&&(1==flagMotor2RUN))
            {
            motor1_decel_encoder_config.start_freq_hz = (uint32_t)(gain * motorPrvPulsOut[0]);
            motor2_decel_encoder_config.start_freq_hz = (uint32_t)(gain * motorPrvPulsOut[1]);
            ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&motor1_decel_encoder_config, &motor1_decel_encoder));
            ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&motor2_decel_encoder_config, &motor2_decel_encoder));
            }
            tx_config.loop_count = 0;
            // if(1==flagMotor1RUN)
            // {
            ESP_ERROR_CHECK(rmt_transmit(motor1_chan, motor1_decel_encoder, &decel_samples, sizeof(decel_samples), &tx_config));
            ESP_ERROR_CHECK(rmt_transmit(motor2_chan, motor2_decel_encoder, &decel_samples, sizeof(decel_samples), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor2_chan, -1));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor1_chan, -1));
            vTaskDelay(100 / portTICK_PERIOD_MS);
            // }          
            ESP_LOGI(TAG, "Stop motors");
            tx_config.loop_count = 0;
            if (FORWORD == motorDir[0])
            {
                ESP_LOGI(MOTOR_TAG, "Motor1 Moving Forword");
                ESP_LOGI(MOTOR_TAG, "Set spin direction for Motor 1");
                gpio_set_level(MOTOR1_STEP_MOTOR_GPIO_DIR, MOTOR1_STEP_MOTOR_SPIN_DIR_CLOCKWISE);
            }
            if (BACKWORD == motorDir[0])
            {
                ESP_LOGI(MOTOR_TAG, "MMotor1 Moving Backword");
                ESP_LOGI(MOTOR_TAG, "Set spin direction for Motor 1");
                gpio_set_level(MOTOR1_STEP_MOTOR_GPIO_DIR, MOTOR1_STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE);
            }
            if (FORWORD == motorDir[1])
            {
                ESP_LOGI(MOTOR_TAG, "Motor2 Moving Forword");
                ESP_LOGI(MOTOR_TAG, "Set spin direction for Motor 2");
                gpio_set_level(MOTOR2_STEP_MOTOR_GPIO_DIR, MOTOR2_STEP_MOTOR_SPIN_DIR_CLOCKWISE);
            }
            if (BACKWORD == motorDir[1])
            {
                ESP_LOGI(MOTOR_TAG, "Motor2 Moving Backword");
                ESP_LOGI(MOTOR_TAG, "Set spin direction for Motor 2");
                gpio_set_level(MOTOR2_STEP_MOTOR_GPIO_DIR, MOTOR2_STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE );
            }
            if((1==flagMotor1RUN)&&(1==flagMotor2RUN))
            {            
            motor1_accel_encoder_config.end_freq_hz = (uint32_t)(gain * motorPulsOut[0]);
            motor2_accel_encoder_config.end_freq_hz = (uint32_t)(gain * motorPulsOut[1]);
            ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&motor1_accel_encoder_config, &motor1_decel_encoder));
            ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&motor2_accel_encoder_config, &motor2_decel_encoder));
            }
            ESP_LOGI(TAG, "set direction");
            ESP_ERROR_CHECK(rmt_transmit(motor1_chan, motor1_accel_encoder, &accel_samples, sizeof(accel_samples), &tx_config));
            flagMotor1RUN=1;
            ESP_ERROR_CHECK(rmt_transmit(motor2_chan, motor2_accel_encoder, &accel_samples, sizeof(accel_samples), &tx_config));
            flagMotor2RUN=1;
            ESP_LOGI(TAG, "run motors ");
            motorPrevDir[0] = motorDir[0];
            motorPrevDir[1] = motorDir[1];

            motorPrvPulsOut[0]= motorPulsOut[0];
            motorPrvPulsOut[1]= motorPulsOut[1];
        }
        // ESP_LOGI(TAG, "continue previous state");
        tx_config.loop_count = 1;
        if(5<uniform_speed1_hz)
        {
            ESP_ERROR_CHECK(rmt_transmit(motor1_chan, motor1_uniform_encoder, &uniform_speed1_hz, sizeof(uniform_speed1_hz), &tx_config));
        }
        if(5<uniform_speed2_hz)
        {        
            ESP_ERROR_CHECK(rmt_transmit(motor2_chan, motor2_uniform_encoder, &uniform_speed2_hz, sizeof(uniform_speed2_hz), &tx_config));
        }
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor1_chan, -1));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor2_chan, -1));

        // motorPulsOut[0] = 0;
        // motorPulsOut[1] = 0;

    }

}




// static void stepper_motor_run(void *pvParameter)
// {

//     // ===========Init stepper motor ==========================================

//         const static uint32_t accel_samples = 100;
//         const static uint32_t decel_samples = 100;

        
//         ESP_LOGI(TAG, "Initialize EN + DIR GPIO for Motor 1");
//         gpio_config_t motor_en_dir_gpio_config = {
//             .mode = GPIO_MODE_OUTPUT,
//             .intr_type = GPIO_INTR_DISABLE,
//             .pin_bit_mask = ((1ULL << MOTOR1_STEP_MOTOR_GPIO_DIR) | (1ULL << MOTOR1_STEP_MOTOR_GPIO_EN)|(1ULL << MOTOR2_STEP_MOTOR_GPIO_DIR)),
//         };
//         ESP_ERROR_CHECK(gpio_config(&motor_en_dir_gpio_config));


//         ESP_LOGI(TAG, "Create RMT TX channel for Motor 1");
//         rmt_tx_channel_config_t motor1_tx_chan_config = {
//             .clk_src = RMT_CLK_SRC_DEFAULT,
//             .gpio_num = MOTOR1_STEP_MOTOR_GPIO_STEP,
//             .mem_block_symbols = 64,
//             .resolution_hz = MOTOR1_STEP_MOTOR_RESOLUTION_HZ,
//             .trans_queue_depth = 10,
//         };
//         ESP_ERROR_CHECK(rmt_new_tx_channel(&motor1_tx_chan_config, &motor1_chan));

//         ESP_LOGI(TAG, "Create RMT TX channel for Motor 2");
//         rmt_tx_channel_config_t motor2_tx_chan_config = {
//             .clk_src = RMT_CLK_SRC_DEFAULT,
//             .gpio_num = MOTOR2_STEP_MOTOR_GPIO_STEP,
//             .mem_block_symbols = 64,
//             .resolution_hz = MOTOR2_STEP_MOTOR_RESOLUTION_HZ,
//             .trans_queue_depth = 10,
//         };
//         ESP_ERROR_CHECK(rmt_new_tx_channel(&motor2_tx_chan_config, &motor2_chan));

//         // ESP_LOGI(TAG, "Enable Motor 1");
//         // gpio_set_level(MOTOR1_STEP_MOTOR_GPIO_EN, MOTOR1_STEP_MOTOR_ENABLE_LEVEL);

//         // ESP_LOGI(TAG, "Enable Motor 2");
//         // gpio_set_level(MOTOR2_STEP_MOTOR_GPIO_EN, MOTOR2_STEP_MOTOR_ENABLE_LEVEL);

//         ESP_LOGI(TAG, "Create motor encoders for Motor 1");
//         stepper_motor_curve_encoder_config_t motor1_accel_encoder_config = {
//             .resolution = MOTOR1_STEP_MOTOR_RESOLUTION_HZ,
//             .sample_points = 100,
//             .start_freq_hz = 0,
//             .end_freq_hz = 100,
//         };
//         rmt_encoder_handle_t motor1_accel_encoder = NULL;
//         ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&motor1_accel_encoder_config, &motor1_accel_encoder));

//         stepper_motor_uniform_encoder_config_t motor1_uniform_encoder_config = {
//             .resolution = MOTOR1_STEP_MOTOR_RESOLUTION_HZ,
//         };
//         rmt_encoder_handle_t motor1_uniform_encoder = NULL;
//         ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&motor1_uniform_encoder_config, &motor1_uniform_encoder));

//         stepper_motor_curve_encoder_config_t motor1_decel_encoder_config = {
//             .resolution = MOTOR1_STEP_MOTOR_RESOLUTION_HZ,
//             .sample_points = 100,
//             .start_freq_hz = 100,
//             .end_freq_hz = 0,
//         };
//         rmt_encoder_handle_t motor1_decel_encoder = NULL;
//         ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&motor1_decel_encoder_config, &motor1_decel_encoder));

//         ESP_LOGI(TAG, "Create motor encoders for Motor 2");
//         stepper_motor_curve_encoder_config_t motor2_accel_encoder_config = {
//             .resolution = MOTOR2_STEP_MOTOR_RESOLUTION_HZ,
//             .sample_points = 100,
//             .start_freq_hz = 0,
//             .end_freq_hz = 100,
//         };
//         rmt_encoder_handle_t motor2_accel_encoder = NULL;
//         ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&motor2_accel_encoder_config, &motor2_accel_encoder));

//         stepper_motor_uniform_encoder_config_t motor2_uniform_encoder_config = {
//             .resolution = MOTOR2_STEP_MOTOR_RESOLUTION_HZ,
//         };
//         rmt_encoder_handle_t motor2_uniform_encoder = NULL;
//         ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&motor2_uniform_encoder_config, &motor2_uniform_encoder));

//         stepper_motor_curve_encoder_config_t motor2_decel_encoder_config = {
//             .resolution = MOTOR2_STEP_MOTOR_RESOLUTION_HZ,
//             .sample_points = 100,
//             .start_freq_hz = 100,
//             .end_freq_hz = 0,
//         };
//         rmt_encoder_handle_t motor2_decel_encoder = NULL;
//         ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&motor2_decel_encoder_config, &motor2_decel_encoder));

//         ESP_LOGI(TAG, "Enable RMT channels");
//         ESP_ERROR_CHECK(rmt_enable(motor1_chan));
//         ESP_ERROR_CHECK(rmt_enable(motor2_chan));

//         ESP_LOGI(TAG, "Spin motors for 6000 steps: 500 accel + 5000 uniform + 500 decel");
//         rmt_transmit_config_t tx_config = {
//             .loop_count = 0,
//         };
//     //=========================================================================
//     //==================wait for serial events============================================


//     while(true) 
//     {   
//         int gain = 1;
//         uint32_t uniform_speed1_hz = (uint32_t)(gain * motorPulsOut[0]);
//         uint32_t uniform_speed2_hz = (uint32_t)(gain * motorPulsOut[1]);
//         // ESP_LOGI(TAG, "main loop");
//         if((motorPrevDir[0] != motorDir[0])||(motorPrevDir[1] != motorDir[1]))
//         {
//             tx_config.loop_count = 0;
//             // if(1==flagMotor1RUN)
//             // {
//             ESP_ERROR_CHECK(rmt_transmit(motor1_chan, motor1_decel_encoder, &decel_samples, sizeof(decel_samples), &tx_config));
//             ESP_ERROR_CHECK(rmt_transmit(motor2_chan, motor2_decel_encoder, &decel_samples, sizeof(decel_samples), &tx_config));
//             ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor2_chan, -1));
//             ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor1_chan, -1));
//             vTaskDelay(100 / portTICK_PERIOD_MS);
//             // }          
//             ESP_LOGI(TAG, "Stop motors");
//             tx_config.loop_count = 0;
//             if (FORWORD == motorDir[0])
//             {
//                 ESP_LOGI(MOTOR_TAG, "Motor1 Moving Forword");
//                 ESP_LOGI(MOTOR_TAG, "Set spin direction for Motor 1");
//                 gpio_set_level(MOTOR1_STEP_MOTOR_GPIO_DIR, MOTOR1_STEP_MOTOR_SPIN_DIR_CLOCKWISE);
//             }
//             if (BACKWORD == motorDir[0])
//             {
//                 ESP_LOGI(MOTOR_TAG, "MMotor1 Moving Backword");
//                 ESP_LOGI(MOTOR_TAG, "Set spin direction for Motor 1");
//                 gpio_set_level(MOTOR1_STEP_MOTOR_GPIO_DIR, MOTOR1_STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE);
//             }
//             if (FORWORD == motorDir[1])
//             {
//                 ESP_LOGI(MOTOR_TAG, "Motor2 Moving Forword");
//                 ESP_LOGI(MOTOR_TAG, "Set spin direction for Motor 2");
//                 gpio_set_level(MOTOR2_STEP_MOTOR_GPIO_DIR, MOTOR2_STEP_MOTOR_SPIN_DIR_CLOCKWISE);
//             }
//             if (BACKWORD == motorDir[1])
//             {
//                 ESP_LOGI(MOTOR_TAG, "Motor2 Moving Backword");
//                 ESP_LOGI(MOTOR_TAG, "Set spin direction for Motor 2");
//                 gpio_set_level(MOTOR2_STEP_MOTOR_GPIO_DIR, MOTOR2_STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE );
//             }
            
//             ESP_LOGI(TAG, "set direction");
//             ESP_ERROR_CHECK(rmt_transmit(motor1_chan, motor1_accel_encoder, &accel_samples, sizeof(accel_samples), &tx_config));
//             flagMotor1RUN=1;
//             ESP_ERROR_CHECK(rmt_transmit(motor2_chan, motor2_accel_encoder, &accel_samples, sizeof(accel_samples), &tx_config));
//             flagMotor2RUN=1;
//             ESP_LOGI(TAG, "run motors ");
//             motorPrevDir[0] = motorDir[0];
//             motorPrevDir[1] = motorDir[1];
//         }
//         // ESP_LOGI(TAG, "continue previous state");
//         tx_config.loop_count = 1;
//         if(0!=uniform_speed1_hz)
//         {
//             ESP_ERROR_CHECK(rmt_transmit(motor1_chan, motor1_uniform_encoder, &uniform_speed1_hz, sizeof(uniform_speed1_hz), &tx_config));
//         }
//         if(0!=uniform_speed2_hz)
//         {        
//             ESP_ERROR_CHECK(rmt_transmit(motor2_chan, motor2_uniform_encoder, &uniform_speed2_hz, sizeof(uniform_speed2_hz), &tx_config));
//         }
//         ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor1_chan, -1));
//         ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor2_chan, -1));

//         // motorPulsOut[0] = 0;
//         // motorPulsOut[1] = 0;

//     }

// }

void initPeripheral(void)
{

    initUART();
    initUART1();
    initUART2();
}


  


void initStorage(void)
{
 // Initialize NVS==============================================
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG,"Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG,"Done\n");
        err = nvs_get_i32(my_handle, "SPEED", &SPEED);
        switch (err) {
            case ESP_OK:
                ESP_LOGI(TAG,"Done\n");
                ESP_LOGI(TAG,"SPEED = %ld\n", SPEED);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                ESP_LOGE(TAG,"The value SPEED is not initialized yet!\n");
                break;
            default :
                ESP_LOGE(TAG,"Error (%s) reading!\n", esp_err_to_name(err));
        }
        // err = nvs_get_i32(my_handle, "Brightness", &DutyCycle);
        // switch (err) {
        //     case ESP_OK:
        //         ESP_LOGI(TAG,"Done\n");
        //         ESP_LOGI(TAG,"Dutycycle = %ld\n", DutyCycle);
        //         break;
        //     case ESP_ERR_NVS_NOT_FOUND:
        //         ESP_LOGE(TAG,"The value is not initialized yet!\n");
        //         break;
        //     default :
        //         ESP_LOGE(TAG,"Error (%s) reading!\n", esp_err_to_name(err));
        // }
        //  vTaskDelay(100 / portTICK_PERIOD_MS);


        vTaskDelay(100 / portTICK_PERIOD_MS);
            // Close
        nvs_close(my_handle);
    }
}

void app_main(void)
{
    
   
    esp_err_t res;
    initPeripheral();

    initStorage();
      
    // Create the queue for UART2 messages  
      uart2_queue = xQueueCreate(QUEUE_SIZE, sizeof(char *));  
      if (uart2_queue == NULL) {  
          ESP_LOGE("MAIN", "Failed to create UART2 queue");  
          return;  
      }  
   

    // initPeripheral();
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 8, NULL, configMAX_PRIORITIES - 2, NULL);
    // xTaskCreate(tx_task, "uart_tx_task", 1024 * 8, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(rx_task1, "uart_rx_task", 1024 * 8, NULL, configMAX_PRIORITIES - 2, NULL);
    
    xTaskCreate(pwmLEDC, "pwmLEDC_task", 1024 * 4, NULL, configMAX_PRIORITIES - 3, NULL);
    
    xTaskCreate(rx_task2, "uart2_rx_task", 1024 * 8, NULL, configMAX_PRIORITIES - 2, NULL);  
    xTaskCreate(uart2_queue_processor_task, "uart2_queue_processor", 1024 * 8, NULL, configMAX_PRIORITIES - 1, NULL);  
    
    
    // xTaskCreate(tx_task2, "uart_tx_task", 1024 * 8, NULL, configMAX_PRIORITIES - 2, NULL);

    xTaskCreate(stepper_motor_run, "stepper_motor_run",  1024 * 4, NULL, configMAX_PRIORITIES - 1, NULL);
    // xTaskCreate(motorRunFwd, "motorRunFwd",  1024 * 4, NULL, configMAX_PRIORITIES - 1, NULL);
    ESP_LOGI(TAG, "Initialization DONE");

    while (1) {
        // ESP_LOGD(TAG, "Looping");
        // vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

