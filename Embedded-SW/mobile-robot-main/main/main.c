#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/queue.h"

// Include source files for Mobile Robot here!
#include "led_blink.c"
#include "motor_control.c"
#include "esp_now.c"
#include "ultrasonic_measure.c"

// Setup any Queues here!
// Motor Signal Queues
static QueueHandle_t right_encoder_queue;
static QueueHandle_t left_encoder_queue;

// Ultrasonic Sensor Data Queues
static QueueHandle_t ultrasonic_left_queue;
static QueueHandle_t ultrasonic_center_queue;
static QueueHandle_t ultrasonic_right_queue;

// State Variable Queues
static QueueHandle_t x_position_queue;
static QueueHandle_t y_position_queue;
static QueueHandle_t theta_position_queue;

// Write RTOS Callback functions here! (The RTOS task can also be defined in your .c file)

// RTOS Task #1 - Blink LED
void vLed_blink_task()
{
    // This is a basic RTOS task that blinks the addressible LED every 200 ms!
    configure_led();
    while (1)
    {
        blink_led();                        // Blink the LED
        //ESP_LOGI(LED_TAG, "Blinked the LED!");
        s_led_state = !s_led_state;
        vTaskDelay(pdMS_TO_TICKS(200));     // Block the task for 200 ms
    }
}

// RTOS Task #2 - Measure Encoders
void vMeasure_Encoders()
{
    // Setup the rotary encoders
    encoder_setup();

    // Initialize the pulse counters
    int right_current_pulse_cnt = 0;
    int left_current_pulse_cnt = 0;
    
    while (1)
    {
        // Measure the Encoder Values
        pcnt_unit_get_count(pcnt_unit_right, &right_current_pulse_cnt);
        pcnt_unit_get_count(pcnt_unit_left, &left_current_pulse_cnt);

        // Send measured values to the respective Queues
        xQueueSend(right_encoder_queue, (void*)&right_current_pulse_cnt, 10);
        xQueueSend(left_encoder_queue, (void*)&left_current_pulse_cnt, 10);

        // ESP_LOGI(TAG_MOTOR, "LEFT: %d", left_current_pulse_cnt);
        // ESP_LOGI(TAG_MOTOR, "RIGHT: %d", right_current_pulse_cnt);

        // Estimate state
        estimate_state((double) left_current_pulse_cnt, (double) right_current_pulse_cnt);
        
        ESP_LOGI(TAG_MOTOR, "X: %f", x);
        ESP_LOGI(TAG_MOTOR, "Y: %f", y);
        ESP_LOGI(TAG_MOTOR, "T: %f", theta);
    
        // Clear the pulse count
        pcnt_unit_clear_count(pcnt_unit_right);
        pcnt_unit_clear_count(pcnt_unit_left);

        // Send updated state to Queues
        xQueueSend(x_position_queue, (void*)&x, 10);
        xQueueSend(y_position_queue, (void*)&y, 10);
        xQueueSend(theta_position_queue, (void*)&theta, 10);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// RTOS Task #3 - Measure Ultrasonic Sensors
void vMeasure_Ultrasonic()
{
    ultrasonic_setup();

    while (1)
    {
        float distance1;
        esp_err_t res1 = ultrasonic_measure(&sensor1, MAX_DISTANCE_CM, &distance1);
        if (res1 != ESP_OK)
            handle_error(res1);
        else
            printf("Distance1: %0.04f cm\n", distance1 * 100);

        float distance2;
        esp_err_t res2 = ultrasonic_measure(&sensor2, MAX_DISTANCE_CM, &distance2);
        if (res2 != ESP_OK)
            handle_error(res1);
        else
            printf("Distance2: %0.04f cm\n", distance2 * 100);

        float distance3;
        esp_err_t res3 = ultrasonic_measure(&sensor3, MAX_DISTANCE_CM, &distance3);
        if (res3 != ESP_OK)
            handle_error(res1);
        else
            printf("Distance3: %0.04f cm\n", distance3 * 100);

        printf("-----------------\n");

        xQueueSend(ultrasonic_left_queue, (void*)&distance1, 10);
        xQueueSend(ultrasonic_center_queue, (void*)&distance2, 10);
        xQueueSend(ultrasonic_right_queue, (void*)&distance3, 10);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// RTOS Task #4 - Motor PID Control
void vMotor_PID_Control()
{
    // Setup Motors
    motor_setup();
    bdc_motor_forward(left_motor);
    bdc_motor_forward(right_motor);

    // Sets up PID controller parameters
    pid_ctrl_parameter_t pid_parameters = {
        .kp = 1.6,
        .ki = 0.4,
        .kd = 0.2,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output = BDC_MCPWM_DUTY_TICK_MAX - 1,
        .min_output = -(BDC_MCPWM_DUTY_TICK_MAX - 1),
        .max_integral = 1000,
        .min_integral = -1000,
    };

    pid_ctrl_config_t pid_config = {
        .init_param = pid_parameters,
    };

    // Create PID controllers for left and right wheels assigns them to the control context
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &right_pid_ctrl));
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &left_pid_ctrl));

    // Initialize Measured Pulse counts
    int right_motor_pulse_cnt = 0;
    int left_motor_pulse_cnt = 0;

    // Initialize Motor Setpoints
    int left_motor_setpoint = 30;
    int right_motor_setpoint = 30;

    while (1)
    {
        // Receives info from queues if available
        xQueueReceive(right_encoder_queue, (void *) &right_motor_pulse_cnt, 0);
        xQueueReceive(left_encoder_queue, (void *) &left_motor_pulse_cnt, 0);
        ESP_LOGI(TAG_MOTOR, "L: %d R: %d", left_motor_pulse_cnt, right_motor_pulse_cnt);
        
        // Calculate Left Wheel Error and PID output
        float left_error = left_motor_setpoint - left_motor_pulse_cnt;
        float left_speed = 0;
        pid_compute(left_pid_ctrl, left_error, &left_speed);
        if(left_speed > 0)
        {
            bdc_motor_forward(left_motor);
        } else {
            bdc_motor_reverse(left_motor);
            left_speed = -left_speed;
        }

        // Calculate Right Wheel Error and PID output
        float right_error = right_motor_setpoint - right_motor_pulse_cnt;
        float right_speed = 0;
        pid_compute(right_pid_ctrl, right_error, &right_speed);
        if(right_speed > 0)
        {
            bdc_motor_forward(right_motor);
        } else {
            bdc_motor_reverse(right_motor);
            right_speed = -right_speed;
        }

        // Apply Inputs
        bdc_motor_set_speed(right_motor, (uint32_t)right_speed);
        bdc_motor_set_speed(left_motor, (uint32_t)left_speed);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
}

// RTOS Task #5 - ESPNOW
void vESP_NOW()
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    // Run the wifi init
    wifi_init();

    // Register Callback functions
    ESP_ERROR_CHECK(esp_now_init());
    esp_now_register_recv_cb(data_recieve_cb);
    esp_now_register_send_cb(data_send_cb);

    // Setup peer info type and register
    memset(&dongle_info, 0, sizeof(dongle_info));
    memcpy(dongle_info.peer_addr, s_usb_dongle_address, 6);
    dongle_info.encrypt = false;
    dongle_info.channel = 0;

    // Init loop
    int retry_counter = 0;
    int max_retries = 5;
    while(esp_now_add_peer(&dongle_info) != ESP_OK)
    {
        if (retry_counter > max_retries)
        {
            ESP_LOGE(ESP_NOW_TAG,"Maximum peer add retries exceeded");
            while (1);
        }
        ESP_LOGW(ESP_NOW_TAG,"ESP NOW Failed to add peer! Retrying . . .");
        vTaskDelay(pdMS_TO_TICKS(5000));
        retry_counter++;
    }

    // Setup local variables for info to send over ESP_NOW
    int w_r = 0.0;
    int w_l = 0.0;

    float ultrasonic_left = 0.0; 
    float ultrasonic_center = 0.0;
    float ultrasonic_right = 0.0;

    u_int8_t counter = 0;
    while (1)
    {
        // Recieve updated info from Queues
        //xQueueReceive(x_position_queue, (void*)&x, 0);
        //xQueueReceive(y_position_queue, (void*)&y, 0);
        //xQueueReceive(theta_position_queue, (void*)&theta, 0);

        // Debug
        ESP_LOGI(ESP_NOW_TAG, "X: %f", x);
        ESP_LOGI(ESP_NOW_TAG, "Y: %f", y);
        ESP_LOGI(ESP_NOW_TAG, "T: %f", theta);

        // Copy into send structure
        memcpy(&state_data.x_position, &x, sizeof(x));
        memcpy(&state_data.y_position, &y, sizeof(y));
        memcpy(&state_data.theta, &theta, sizeof(theta));
        memcpy(&state_data.w_left, &w_l, sizeof(w_l));
        memcpy(&state_data.w_right, &w_r, sizeof(w_r));
        memcpy(&state_data.ultrasonic_left, &ultrasonic_left, sizeof(ultrasonic_left));
        memcpy(&state_data.ultrasonic_center, &ultrasonic_center, sizeof(ultrasonic_center));
        memcpy(&state_data.ultrasonic_right, &ultrasonic_right, sizeof(ultrasonic_right));
        memcpy(&state_data.counter, &counter, sizeof(counter));

        // Send the Data
        esp_now_send(s_usb_dongle_address, (u_int8_t * )&state_data, sizeof(state_data));
        counter++;
        ESP_LOGI(ESP_NOW_TAG, "Counter Value = %i", counter);
        vTaskDelay(pdMS_TO_TICKS(ESP_NOW_RATE));
    }
     
}

// Main function entry point here:
void app_main(void)
{
    // Prints out the MAC Address
    unsigned char mac[6] = {0};
    esp_read_mac(mac,ESP_MAC_WIFI_STA);
    ESP_LOGI(ESP_NOW_TAG, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
    
    // Create Queues using xQueueCreate:
    // Parameters: | Number of values that can be stored in a queue | size in bytes of each variable the queue takes |
    right_encoder_queue = xQueueCreate(5, sizeof(int));
    left_encoder_queue = xQueueCreate(5, sizeof(int));

    x_position_queue = xQueueCreate(5, sizeof(double));
    y_position_queue = xQueueCreate(5, sizeof(double));
    theta_position_queue = xQueueCreate(5, sizeof(double));

    ultrasonic_left_queue = xQueueCreate(5, sizeof(float));
    ultrasonic_center_queue = xQueueCreate(5, sizeof(float));
    ultrasonic_right_queue = xQueueCreate(5, sizeof(float));

    // Create RTOS Tasks here using xTaskCreate:
    // Parameters: | Task callback function | Task Name | Memory Assigned to Task | Parameters to pass into the task | Priority | Task Handle
    xTaskCreate(vLed_blink_task, "Status LED", 4096, NULL, 1, NULL);
    xTaskCreate(vMeasure_Encoders, "Encoder Measurement", 4096, NULL, 10, NULL); 
    //xTaskCreate(vMeasure_Ultrasonic, "Ultrasonic Sensor Measurement", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    //xTaskCreate(vMotor_PID_Control, "Motor CL Controller", 8192, NULL, 10, NULL);
    xTaskCreate(vESP_NOW, "ESP NOW Wireless Coms", 8192, NULL, 2, NULL);
}