#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"
#include "freertos/queue.h"

#include "include/mobile_robot_pins.h"

static const char *TAG_MOTOR = "Motor_Control";

// Motor Configuration
#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             10000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks

// Motor Handles
bdc_motor_handle_t left_motor = NULL;
bdc_motor_handle_t right_motor = NULL;

// Encoder Handles
pcnt_unit_handle_t pcnt_unit_left = NULL;
pcnt_unit_handle_t pcnt_unit_right = NULL;

// PID Controller Handles
pid_ctrl_block_handle_t left_pid_ctrl = NULL;
pid_ctrl_block_handle_t right_pid_ctrl = NULL;

// Encoder Signal Queues
static QueueHandle_t right_encoder_queue;
static QueueHandle_t left_encoder_queue;

// Encoder Configuration
#define MOTOR_PCNT_HIGH_LIMIT 10000
#define MOTOR_PCNT_LOW_LIMIT  -10000

// Macros for State Estimation
#define PI 3.14159265
#define WHEEL_BASE 0.2
#define WHEEL_DIAMETER 0.06
#define TICKS_PER_REVOLUTION 1120
#define SCALING_FACTOR_TRANSLATION 1.25
#define SCALING_FACTOR_ROTATION 1.00

float x = 1.0;
float y = 2.5;
float theta = 0.0;

// Initialize Measured Pulse counts
int8_t right_motor_pulse_cnt = 0;
int8_t left_motor_pulse_cnt = 0;

// PID Configuration
#define BDC_PID_LOOP_PERIOD_MS        10   // calculate the motor speed every 10ms
#define BDC_PID_EXPECT_SPEED          20  // expected motor speed, in the pulses counted by the rotary encoder

// Motor Parameters for feedforward
#define A0 580.0
#define A1 5.88

typedef struct {
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    int report_pulses;
} motor_control_context_t;

void motor_setup()
{
    // Create Right Motor Object
    bdc_motor_config_t motor_config_right = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = MOTOR_R_A,
        .pwmb_gpio_num = MOTOR_R_B,
    };

    // Create Left Motor Object
    bdc_motor_config_t motor_config_left = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = MOTOR_L_B, // NOTE: A and B Phases are flipped intentionally! This means that the motor will rotate forward in the context of the robot
        .pwmb_gpio_num = MOTOR_L_A,
    };

    // Configure mcpwm
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };

    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config_left, &mcpwm_config, &left_motor));
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config_right, &mcpwm_config, &right_motor));
    ESP_LOGI(TAG_MOTOR, "Left and Right Motors setup successful");

    ESP_ERROR_CHECK(bdc_motor_enable(left_motor));
    ESP_ERROR_CHECK(bdc_motor_enable(right_motor));
    ESP_LOGI(TAG_MOTOR, "Motors Enabled Successfully");
}

void encoder_setup()
{
    // Sets the Pulse Counter config
    pcnt_unit_config_t unit_config = {
        .high_limit = MOTOR_PCNT_HIGH_LIMIT,
        .low_limit = MOTOR_PCNT_LOW_LIMIT,
        .flags.accum_count = 1,
    };

    // Creates new pulse counters for the left and right
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit_left));
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit_right));

    // Setup left channel (NOTE: A and B are fipped intentionally to account for the way the motor is connected on the robot!)
    pcnt_chan_config_t chan_a_left_config = {
        .edge_gpio_num = ENC_L_B,
        .level_gpio_num = ENC_L_A,
    };

    pcnt_channel_handle_t pcnt_chan_a_left = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_left, &chan_a_left_config, &pcnt_chan_a_left));
    pcnt_chan_config_t chan_b_left_config = {
        .edge_gpio_num = ENC_L_A,
        .level_gpio_num = ENC_L_B,
    };

    pcnt_channel_handle_t pcnt_chan_b_left = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_left, &chan_b_left_config, &pcnt_chan_b_left));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a_left, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a_left, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b_left, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b_left, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Setup right channel
    pcnt_chan_config_t chan_a_right_config = {
        .edge_gpio_num = ENC_R_A,
        .level_gpio_num = ENC_R_B,
    };

    pcnt_channel_handle_t pcnt_chan_a_right = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_right, &chan_a_right_config, &pcnt_chan_a_right));
    pcnt_chan_config_t chan_b_right_config = {
        .edge_gpio_num = ENC_R_B,
        .level_gpio_num = ENC_R_A,
    };

    pcnt_channel_handle_t pcnt_chan_b_right = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_right, &chan_b_right_config, &pcnt_chan_b_right));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a_right, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a_right, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b_right, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b_right, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Enable Counters
    // Left
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_left));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_left));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_left));

    // Right
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_right));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_right));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_right));
}

void estimate_state(float encoder_left, float encoder_right) {

    float dist_l = (encoder_left/TICKS_PER_REVOLUTION) * (PI * WHEEL_DIAMETER);
    float dist_r = (encoder_right/TICKS_PER_REVOLUTION) * (PI * WHEEL_DIAMETER);
    float d = SCALING_FACTOR_TRANSLATION * (dist_l+dist_r)/2;

    float delta_theta = SCALING_FACTOR_ROTATION * (dist_r-dist_l)/WHEEL_BASE;

    x += d * cos(theta+delta_theta/2);
    y += d * sin(theta+delta_theta/2);
    theta += delta_theta;
}

float compute_feedforward(int8_t cmd_vel)
{
    float u = 0;
    if (cmd_vel > 0)
    {
        u = A0 + (float)cmd_vel*A1;
    } 
    if (cmd_vel < 0)
    {
        u = -A0 + (float)cmd_vel*A1; 
    }
    return u;
}

void vMotor_Routine()
{
    motor_setup();
    bdc_motor_set_speed(left_motor, 398);
    bdc_motor_set_speed(right_motor, 398);
    while (1)
    {
        ESP_LOGI(TAG_MOTOR, "Motors forward");
        bdc_motor_forward(left_motor);
        bdc_motor_forward(right_motor);

        vTaskDelay(pdMS_TO_TICKS(5000));

        ESP_LOGI(TAG_MOTOR, "Motors coasting");
        bdc_motor_coast(left_motor);
        bdc_motor_coast(right_motor);

        vTaskDelay(pdMS_TO_TICKS(5000));

        ESP_LOGI(TAG_MOTOR, "Motors reverse");
        bdc_motor_reverse(left_motor);
        bdc_motor_reverse(right_motor);

        vTaskDelay(pdMS_TO_TICKS(5000));

        ESP_LOGI(TAG_MOTOR, "Motors braking");
        bdc_motor_brake(left_motor);
        bdc_motor_brake(right_motor);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    
}

void vMotor_Ramp()
{
    motor_setup();
    ESP_LOGI(TAG_MOTOR, "Motors forward");
    bdc_motor_forward(left_motor);
    bdc_motor_forward(right_motor);
    int count = BDC_MCPWM_DUTY_TICK_MAX - 1;

    int8_t w_l;
    int8_t w_r;
    int s_print;
    // TODO: Add start button here to begin

    // Begin motor open loop test
    xQueueReset(right_encoder_queue);
    xQueueReset(left_encoder_queue);
    while (1)
    {
        for (size_t i = 0; i < count; i++)
        {
            bdc_motor_set_speed(left_motor,i);
            bdc_motor_set_speed(right_motor,i);
            // Receives info from queues if available
            xQueueReceive(right_encoder_queue, (void *) &w_l, pdTICKS_TO_MS(10));
            xQueueReceive(left_encoder_queue, (void *) &w_r, pdTICKS_TO_MS(10));

            // Print info to the terminal
            ESP_LOGI(TAG_MOTOR, "S:%d L:%d R:%d",i, w_l, w_r);
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        for (size_t j = count; j > 0; j--)
        {
            bdc_motor_set_speed(left_motor,j);
            bdc_motor_set_speed(right_motor,j);
            // Receives info from queues if available
            xQueueReceive(right_encoder_queue, (void *) &w_l, pdTICKS_TO_MS(10));
            xQueueReceive(left_encoder_queue, (void *) &w_r, pdTICKS_TO_MS(10));

            // Print info to the terminal
            ESP_LOGI(TAG_MOTOR, "S:%d L:%d R:%d",j, w_l, w_r);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void vMotor_Open_Loop()
{
    motor_setup();
    bdc_motor_forward(left_motor);
    bdc_motor_forward(right_motor);
    int count = BDC_MCPWM_DUTY_TICK_MAX - 1;
    //float profile[] = {0.0, 0.7, 0.5, -0.2, 0.4, 1.0, -0.4, 0.2, -0.7, -1.0, -0.3, 0.8, 0.0};
    int interval = 1000;
    int profile[] = {0, 200, 400, 500, 600, 700, 800, 900, 500, 600, 550, 400, 0, -200, -400, -500, -550, -600, -650, -700, -800, -650, -900, -850, 200, 400, 600, 999, 800, 750, 200, 0, -400, -600, -800, -700, -999, -750 -300, 0};
    u_int32_t signal;

    while (1)
    {
        int8_t w_l;
        int8_t w_r;
        int s_print;
        // TODO: Add start button here to begin

        // Begin motor open loop test
        xQueueReset(right_encoder_queue);
        xQueueReset(left_encoder_queue);
        for (size_t pidx = 0; pidx < sizeof(profile)/sizeof(int); pidx++)
        {
            signal = (u_int32_t) abs(profile[pidx]);
            s_print = profile[pidx];
            if (profile[pidx] > 0)
            {
                bdc_motor_forward(left_motor);
                bdc_motor_forward(right_motor);
            } else {
                bdc_motor_reverse(left_motor);
                bdc_motor_reverse(right_motor);
            }
            // Set the motor speeds
            bdc_motor_set_speed(left_motor,signal);
            bdc_motor_set_speed(right_motor,signal);

            for (size_t i = 0; i < 100; i++)
            {
                // Receives info from queues if available
                xQueueReceive(right_encoder_queue, (void *) &w_l, pdTICKS_TO_MS(10));
                xQueueReceive(left_encoder_queue, (void *) &w_r, pdTICKS_TO_MS(10));

                // Print info to the terminal
                ESP_LOGI(TAG_MOTOR, "S:%d L:%d R:%d",s_print, w_l, w_r);
            }
            
        }
        vTaskDelay(pdTICKS_TO_MS(10));
    }
    
}