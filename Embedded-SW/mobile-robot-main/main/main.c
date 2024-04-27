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

// Define parameters
#define debug_statements 1

// Setup any Queues here!
// Odometry Queues
static QueueHandle_t x_position_queue;
static QueueHandle_t y_position_queue;
static QueueHandle_t theta_queue;

// Ultrasonic Sensor Data Queues
static QueueHandle_t ultrasonic_left_queue;
static QueueHandle_t ultrasonic_center_queue;
static QueueHandle_t ultrasonic_right_queue;

// Motor Command Queue
static QueueHandle_t robot_cmd_queue;

// ESP NOW Data Recieved Callback Function
static void data_recieve_cb(const uint8_t *mac_address, uint8_t *incomingData, int length)
{
    
    memcpy(&robot_cmd, incomingData, sizeof(robot_cmd));

    // Send to motor command queue
    xQueueSend(robot_cmd_queue, (void*)&robot_cmd, 5);

    if(debug_statements)
    {
        ESP_LOGI(ESP_NOW_TAG,"Data recieved:");
        ESP_LOGI(ESP_NOW_TAG,"w_r : %i",robot_cmd.w_right_cmd);
        ESP_LOGI(ESP_NOW_TAG,"w_l : %i",robot_cmd.w_left_cmd);
    }
    
}

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
        estimate_state((float) left_current_pulse_cnt, (float) right_current_pulse_cnt);
        
        //ESP_LOGI(TAG_MOTOR, "X: %f", x);
        //ESP_LOGI(TAG_MOTOR, "Y: %f", y);
        //ESP_LOGI(TAG_MOTOR, "T: %f", theta);
    
        // Clear the pulse count
        pcnt_unit_clear_count(pcnt_unit_right);
        pcnt_unit_clear_count(pcnt_unit_left);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// RTOS Task #3 - Measure Ultrasonic Sensors
void vMeasure_Ultrasonic()
{
    ultrasonic_setup();

    while (1)
    {
        uint32_t distance1;
        esp_err_t res1 = ultrasonic_measure_cm(&sensor1, MAX_DISTANCE_CM, &distance1);
        if (res1 != ESP_OK)
            handle_error(res1);
        else
            printf("Distance1: %lu cm\n", distance1);

        uint32_t distance2;
        esp_err_t res2 = ultrasonic_measure_cm(&sensor2, MAX_DISTANCE_CM, &distance2);
        if (res2 != ESP_OK)
            handle_error(res1);
        else
            printf("Distance2: %lu cm\n", distance2);

        uint32_t distance3;
        esp_err_t res3 = ultrasonic_measure_cm(&sensor3, MAX_DISTANCE_CM, &distance3);
        if (res3 != ESP_OK)
            handle_error(res1);
        else
            printf("Distance3: %lu cm\n", distance3);

        printf("-----------------\n");

        // Cast to uint_8
        u_int8_t distance1_tx = (uint8_t)distance1;
        u_int8_t distance2_tx = (uint8_t)distance2;
        u_int8_t distance3_tx = (uint8_t)distance3;

        xQueueSend(ultrasonic_left_queue, (void*)&distance1_tx, 10);
        xQueueSend(ultrasonic_center_queue, (void*)&distance2_tx, 10);
        xQueueSend(ultrasonic_right_queue, (void*)&distance3_tx, 10);

        vTaskDelay(pdMS_TO_TICKS(1000));
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
        .kp = 10.0,
        .ki = 0.0,
        .kd = 0.3,
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

    // Initialize Motor Setpoints
    mobile_robot_command_t cmd = {0};
    int notReceived = 0;

    while (1)
    {

        // Receives info from queues if available
        xQueueReceive(right_encoder_queue, (void *) &right_motor_pulse_cnt, 0);
        xQueueReceive(left_encoder_queue, (void *) &left_motor_pulse_cnt, 0);

        if (xQueueReceive(robot_cmd_queue, (void *)&cmd, 0) != pdTRUE) {
            notReceived += 1;
            if (notReceived > 10) {
                ESP_LOGI(TAG_MOTOR, "Queue hasn't received anything for %d times, stop the robot!", notReceived);
                cmd.w_left_cmd = 0;
                cmd.w_right_cmd = 0;
            }
        } else {
           notReceived = 0; 
        }

        if(debug_statements) {
            ESP_LOGI(TAG_MOTOR, "L_cmd: %d R_cmd: %d",cmd.w_left_cmd, cmd.w_right_cmd);
            ESP_LOGI(TAG_MOTOR, "    L: %d     R: %d", left_motor_pulse_cnt, right_motor_pulse_cnt);
        } 
                        
        // Calculate Left Wheel Error and PID output
        float left_error = cmd.w_left_cmd - left_motor_pulse_cnt;
        float left_speed = 0;
        pid_compute(left_pid_ctrl, left_error, &left_speed);
        left_speed += compute_feedforward(cmd.w_left_cmd);
        if(left_speed > 0)
        {
            bdc_motor_forward(left_motor);
        } else {
            bdc_motor_reverse(left_motor);
            left_speed = -left_speed;
        }

        // Calculate Right Wheel Error and PID output
        float right_error = cmd.w_right_cmd - right_motor_pulse_cnt;
        float right_speed = 0;
        pid_compute(right_pid_ctrl, right_error, &right_speed);
        right_speed += compute_feedforward(cmd.w_right_cmd);
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
    int x_tx; 
    int y_tx; 
    int theta_tx;

    uint8_t ultrasonic_left = 0; 
    uint8_t ultrasonic_center = 0;
    uint8_t ultrasonic_right = 0;

    u_int8_t counter = 0;
    while (1)
    {
        // Update Odometry
        x_tx = (int) 1000*x;
        y_tx = (int) 1000*y;
        theta_tx = (int) 1000*theta;

        if (debug_statements)
        {
           // Debug
            ESP_LOGI(ESP_NOW_TAG, "X: %f", x);
            ESP_LOGI(ESP_NOW_TAG, "Y: %f", y);
            ESP_LOGI(ESP_NOW_TAG, "T: %f", theta);
            ESP_LOGI(ESP_NOW_TAG, "X: %d", x_tx);
            ESP_LOGI(ESP_NOW_TAG, "Y: %d", y_tx);
            ESP_LOGI(ESP_NOW_TAG, "T: %d", theta_tx);
        }

        


        // If we do not recieve a value from the Queue, we reset the sensor value to zero
        if(xQueueReceive(ultrasonic_left_queue, (void*)&ultrasonic_left, 0) != pdTRUE)
        {
            ultrasonic_left = 0;
        }

        if(xQueueReceive(ultrasonic_center_queue, (void*)&ultrasonic_center, 0) != pdTRUE)
        {
            ultrasonic_center = 0;
        }

        if(xQueueReceive(ultrasonic_right_queue, (void*)&ultrasonic_right, 0) != pdTRUE)
        {
            ultrasonic_right = 0;
        }

        // Copy into send structure
        memcpy(&state_data.x_position, &x_tx, sizeof(x_tx));
        memcpy(&state_data.y_position, &y_tx, sizeof(y_tx));
        memcpy(&state_data.theta, &theta_tx, sizeof(theta_tx));
        memcpy(&state_data.w_left, &left_motor_pulse_cnt, sizeof(left_motor_pulse_cnt));
        memcpy(&state_data.w_right, &right_motor_pulse_cnt, sizeof(right_motor_pulse_cnt));
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
    x_position_queue = xQueueCreate(5, sizeof(int));
    y_position_queue = xQueueCreate(5, sizeof(int));
    theta_queue = xQueueCreate(5, sizeof(int));

    right_encoder_queue = xQueueCreate(5, sizeof(int8_t));
    left_encoder_queue = xQueueCreate(5, sizeof(int8_t));

    ultrasonic_left_queue = xQueueCreate(5, sizeof(u_int8_t));
    ultrasonic_center_queue = xQueueCreate(5, sizeof(u_int8_t));
    ultrasonic_right_queue = xQueueCreate(5, sizeof(u_int8_t));

    robot_cmd_queue = xQueueCreate(10, sizeof(mobile_robot_command_t));

    // Create RTOS Tasks here using xTaskCreate:
    // Parameters: | Task callback function | Task Name | Memory Assigned to Task | Parameters to pass into the task | Priority | Task Handle
    xTaskCreate(vLed_blink_task, "Status LED", 4096, NULL, 1, NULL);
    xTaskCreate(vMeasure_Encoders, "Encoder Measurement", 4096, NULL, 10, NULL); 
    xTaskCreate(vMeasure_Ultrasonic, "Ultrasonic Sensor Measurement", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(vMotor_PID_Control, "Motor CL Controller", 8192, NULL, 10, NULL);
    xTaskCreate(vESP_NOW, "ESP NOW Wireless Coms", 8192, NULL, 2, NULL);
    //xTaskCreate(vMotor_Ramp, "Open Loop Motor Test", 4096, NULL, 2, NULL);
}