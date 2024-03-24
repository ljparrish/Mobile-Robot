#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

// Include source files for Mobile Robot here!
#include "led_blink.c"
#include "motor_control.c"

// Write RTOS Callback functions here! (The RTOS task can also be defined in your .c file)

// RTOS Task #1 - Blink LED
void vLed_blink_task()
{
    // This is a basic RTOS task that blinks the addressible LED every 200 ms!
    configure_led();
    while (1)
    {
        blink_led();                        // Blink the LED
        ESP_LOGI(LED_TAG, "Blinked the LED!");
        s_led_state = !s_led_state;
        vTaskDelay(pdMS_TO_TICKS(200));     // Block the task for 200 ms
    }
}

// RTOS Task #2 - Measure Encoders
void vMeasure_Encoders()
{
    while (1)
    {
        /* code */
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

// Main function entry point here:
void app_main(void)
{
    // Create RTOS Tasks here using xTaskCreate:
    // Parameters: | Task callback function | Task Name | Memory Assigned to Task | Parameters to pass into the task | Priority | Task Handle
    xTaskCreate(vLed_blink_task, "Status LED", 4096, NULL, 1, NULL);
    xTaskCreate(vMotor_Ramp, "Motor Control", 4096, NULL, 1, NULL);
}