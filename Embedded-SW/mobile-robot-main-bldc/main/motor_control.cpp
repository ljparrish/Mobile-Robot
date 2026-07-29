#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/queue.h"

#include "esp_hal_bldc_6pwm.h"
#include "BLDCMotor.h"
#include "mt6701.h"

#include "include/motor_go_axis_pinout.h"

BLDCMotor motor_0 = BLDCMotor(11);
BLDCDriver6PWM motor_driver_0 = BLDCDriver6PWM(MOTOR_0_UH, MOTOR_0_UL, MOTOR_0_VH, MOTOR_0_VL, MOTOR_0_WH, MOTOR_O_WL);
MT6701 encoder_0 = MT6701(SPI2_HOST, ENC_SCL, ENC_SDA, GPIO_NUM_45, ENC_0_CS);

static const char *MOTOR_CTRL_TAG = "MOTOR_CTRL";

float motor_cmd = 0.0;

void motor_setup()
{
    // Call function to initialize motor parameters
    motor_driver_0.voltage_power_supply = 17.0;
    motor_driver_0.voltage_limit = 16.0;
    motor_driver_0.init();

    motor_0.linkDriver(&motor_driver_0);
    motor_0.velocity_limit = 200.0;
    motor_0.voltage_limit = 8.0;

    //motor_0.phase_resistance = 9.5;
    //motor_0.phase_inductance = 0.002;

    motor_0.current_limit = 1.0;

    encoder_0.init();
    motor_0.linkSensor(&encoder_0);

    motor_0.controller = MotionControlType::velocity;
    motor_0.torque_controller = TorqueControlType::voltage;
    motor_0.foc_modulation = FOCModulationType::SpaceVectorPWM;
    
    motor_0.PID_velocity.P = 0.2;
    motor_0.PID_velocity.I = 20.0;
    motor_0.PID_velocity.D = 0.01;

    motor_0.PID_velocity.output_ramp = 2500;
    motor_0.voltage_sensor_align = 2;
    motor_0.LPF_velocity.Tf = 0.05;
    
    motor_0.init();
    motor_0.initFOC();
    ESP_LOGI(MOTOR_CTRL_TAG, "Motor FOC Init Completed! Motor Status : %x", motor_0.motor_status);
}

void vMotor_Step_Test(void * param)
{
    float setpoints[] = {0.5, 5.0, 10.0, 25.0, 50.0, 10.0, 100.0, 50.0};
    int idx = 0;
    while (idx < sizeof(setpoints) / sizeof(setpoints[0]))
    {
        motor_cmd = setpoints[idx];
        idx++;
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    motor_cmd = 0.0;
    vTaskDelete(NULL);
}

void vMotor_OL_Ramp_Test(void * param)
{
    motor_cmd = 0;

    // Ramp Up
    while (motor_cmd < motor_0.velocity_limit)
    {
        motor_cmd++;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    // Ramp Down
    while (motor_cmd > 0)
    {
        motor_cmd--;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    motor_cmd = 0;
    vTaskDelete(NULL);
}

void vMotor_KV_Calc(void * params)
{
    motor_0.controller = MotionControlType::torque;
    motor_0.torque_controller = TorqueControlType::voltage;

    motor_cmd = 1.0; // Set motor speed to 1 volt
    vTaskDelay(1000); // Allow motor to reach steady state
    float sum_readings = 0.0;
    int num_readings = 0;
    while (num_readings < 100)
    {
        sum_readings = ++motor_0.shaft_velocity;
        num_readings++;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    motor_cmd = 0.0;
    ESP_LOGI(MOTOR_CTRL_TAG, "Motor KV Calculation Completed! Measured Motor KV : %f [rpm/V]", sum_readings / num_readings * 10.0);
    vTaskDelete(NULL);
}

void vMotor_Position_CL_Test(void * params)
{
    motor_0.controller = MotionControlType::angle;
    motor_0.torque_controller = TorqueControlType::voltage;

    motor_0.PID_velocity.P = 3.0;
    motor_0.PID_velocity.I = 0;
    motor_0.PID_velocity.D = 0.01;

    motor_0.P_angle.P = 15;

    float setpoints[] = {0.0, PI/2, PI, PI*3/2, 0.0, PI/4, 5.3, 4.2, 0.0, PI*10, 0.0};
    int idx = 0;
    while (idx < sizeof(setpoints) / sizeof(setpoints[0]))
    {
        motor_cmd = setpoints[idx];
        idx++;
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    motor_cmd = 0.0;
    vTaskDelete(NULL);
}

void motor_timer_cb(void *param)
{
    gpio_set_level(GPIO_NUM_35, HIGH);
    motor_0.loopFOC();
    motor_0.move(motor_cmd);
    gpio_set_level(GPIO_NUM_35, LOW);
}