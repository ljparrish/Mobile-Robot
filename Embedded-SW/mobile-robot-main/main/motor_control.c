#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"

#include "include/mobile_robot_pins.h"

static const char *TAG_MOTOR = "Motor_Control";

// Motor Configuration
#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             500    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks

// Motor Handles
bdc_motor_handle_t left_motor = NULL;
bdc_motor_handle_t right_motor = NULL;

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
        .pwma_gpio_num = MOTOR_L_A,
        .pwmb_gpio_num = MOTOR_L_B,
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
    while (1)
    {
        for (size_t i = 0; i < count; i++)
        {
            bdc_motor_set_speed(left_motor,i);
            bdc_motor_set_speed(right_motor,i);
            ESP_LOGI(TAG_MOTOR, "Speed = %i", i);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}