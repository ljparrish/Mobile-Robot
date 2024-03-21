// // Attempting to set up multitasking structure for reading motor encoder pulses for both wheels

// #include "sdkconfig.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/queue.h"
// #include "esp_log.h"
// #include "driver/pulse_cnt.h"
// #include "driver/gpio.h"
// #include "esp_sleep.h"

// // task being created
// void vLeftWheel()
//     while(1)
//     {


//     }

// void vRightWheel()
//     while(1)
//     {




//     }

// void app_main(void)
// {
// printf("\n Print out the pulse count each wheel for each time step")

// xTaskCreate(vLeftWheel, "Left_Wheel", 2048, NULL, 1, NULL)
// xTaskCreate(vRightWheel, "Right_Wheel", 2048, NULL, 1, NULL)
// }