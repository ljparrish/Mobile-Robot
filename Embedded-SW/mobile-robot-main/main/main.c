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

// Setup any Queues here!
static QueueHandle_t right_encoder_queue;
static QueueHandle_t left_encoder_queue;

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

        // Clear the pulse count
        pcnt_unit_clear_count(pcnt_unit_right);
        pcnt_unit_clear_count(pcnt_unit_left);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// RTOS Task #3 - Measure Ultrasonic Sensors
void vMeasure_Ultrasonic()
{
    while (1)
    {
        /* code */
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

// Main function entry point here:
void app_main(void)
{
    // Create Queues using xQueueCreate:
    // Parameters: | Number of values that can be stored in a queue | size in bytes of each variable the queue takes |
    right_encoder_queue = xQueueCreate(5, sizeof(int));
    left_encoder_queue = xQueueCreate(5, sizeof(int));

    // Create RTOS Tasks here using xTaskCreate:
    // Parameters: | Task callback function | Task Name | Memory Assigned to Task | Parameters to pass into the task | Priority | Task Handle
    xTaskCreate(vLed_blink_task, "Status LED", 4096, NULL, 1, NULL);
    //xTaskCreate(vMotor_Ramp, "Motor Control", 4096, NULL, 1, NULL);
    xTaskCreate(vMeasure_Encoders, "Encoder Measurement", 4096, NULL, 1, NULL);
    xTaskCreate(vMotor_PID_Control, "Motor CL Controller", 8192, NULL, 1, NULL);
}