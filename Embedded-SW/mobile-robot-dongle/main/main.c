#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/queue.h"

// Include source files for Mobile Robot Dongle Here!
#include "uart.c"
// Setup any Queues here!

// Write RTOS Callback functions here! (The RTOS task can also be defined in your .c file)

void app_main(void)
{
    // Create RTOS Tasks here using xTaskCreate:
    // Parameters: | Task callback function | Task Name | Memory Assigned to Task | Parameters to pass into the task | Priority | Task Handle
    xTaskCreate(echo_task, "UART Echo", 2048, NULL, 1, NULL);
}
