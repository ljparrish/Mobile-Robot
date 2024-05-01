#include <stdio.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"
#include <ultrasonic.h>
#include <esp_err.h>

static const char *TAG_ULTRA = "Ultra_Measure";

#define MAX_DISTANCE_CM 400 // 4m max

// Macros for each ultrasonic sensor
ultrasonic_sensor_t sensor1 = {
    .trigger_pin = GPIO_NUM_6,
    .echo_pin = GPIO_NUM_13};
ultrasonic_sensor_t sensor2 = {
    .trigger_pin = GPIO_NUM_9,
    .echo_pin = GPIO_NUM_36};
ultrasonic_sensor_t sensor3 = {
    .trigger_pin = GPIO_NUM_10,
    .echo_pin = GPIO_NUM_35};

void ultrasonic_setup()
{
    ultrasonic_init(&sensor1);
    ultrasonic_init(&sensor2);
    ultrasonic_init(&sensor3);
}

void handle_error(int res)
{
    ESP_LOGI(TAG_ULTRA, "Error %d: ", res);
    switch (res)
    {
    case ESP_ERR_ULTRASONIC_PING:
        ESP_LOGI(TAG_ULTRA, "Cannot ping (device is in invalid state)");
        break;
    case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
        ESP_LOGI(TAG_ULTRA, "Ping timeout (no device found)\n");
        break;
    case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
        ESP_LOGI(TAG_ULTRA, "Echo timeout (i.e. distance too big)\n");
        break;
    default:
        ESP_LOGI(TAG_ULTRA, "%s\n", esp_err_to_name(res));
    }
}