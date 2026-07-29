#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "include/motor_go_axis_pinout.h"

// RTOS Task #1 - Blink LED
static void vLed_blink_task(void * param)
{
    // This is a basic RTOS task that blinks the addressible LED every 200 ms!
    static uint8_t s_led_state = 0;
    gpio_reset_pin(STATUS_LED);
    gpio_set_direction(STATUS_LED, GPIO_MODE_OUTPUT);
    while (1)
    {
        gpio_set_level(STATUS_LED, s_led_state);
        s_led_state = !s_led_state;
        vTaskDelay(pdMS_TO_TICKS(200)); // Block the task for 200 ms
    }
}