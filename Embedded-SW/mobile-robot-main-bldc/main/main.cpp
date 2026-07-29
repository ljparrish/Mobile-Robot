#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "driver/spi_master.h"
#include "mt6701.h"

// Include source files for Mobile Robot here!
#include "include/motor_go_axis_pinout.h"
#include "esp_now.c"
#include "led_blink.c"
#include "motor_control.cpp"

// Define parameters
#define debug_statements 1

// Macros
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))

static const char *ENC_READ_TAG = "ENC_READ";

// Setup any Queues here!

// Write RTOS Callback functions here! (The RTOS task can also be defined in your .c file)

void vRead_magnetic_encoders(void * params)
{

    float enc_0_velocity = 0.0;
    float enc_0_angle = 0.0;
    while (1)
    {
        enc_0_velocity = motor_0.shaft_velocity;
        enc_0_angle = motor_0.shaft_angle;
        ESP_LOGI(ENC_READ_TAG, "Encoder Status -- t: %f w: %f", enc_0_angle, enc_0_velocity);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
}

 // Setup Motor Actuation and Encoder Measurement Timer and CB Function
 const esp_timer_create_args_t motor_ctrl_timer_args = {
    .callback = &motor_timer_cb,
    .name = "Motor Control Timer"
};
esp_timer_handle_t motor_timer_handle;

// Main function entry point here:
extern "C" {
void app_main(void)
{
    // Prints out the MAC Address
    unsigned char mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGI(ESP_NOW_TAG, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // Create Queues using xQueueCreate:
    // Parameters: | Number of values that can be stored in a queue | size in bytes of each variable the queue takes |

    // Initialize Encoder PCNTs
    //encoder_setup();

    // Setup Motor Actuation and Encoder Measurement Timer and CB Function

    // Create RTOS Tasks here using xTaskCreate:
    // Parameters: | Task callback function | Task Name | Memory Assigned to Task | Parameters to pass into the task | Priority | Task Handle
    xTaskCreatePinnedToCore(vLed_blink_task, "Status LED", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(vRead_magnetic_encoders, "Read Encoders", 4096, NULL, 1, NULL, 0);
    
    //xTaskCreatePinnedToCore(vESP_NOW, "ESP NOW Wireless Coms", 8192, NULL, 3, NULL, 1);
    motor_setup();
    gpio_reset_pin(GPIO_NUM_35);
    gpio_set_direction(GPIO_NUM_35, GPIO_MODE_OUTPUT);

    //xTaskCreatePinnedToCore(vMotor_Step_Test, "Motor Step Test", 4096, NULL, 1, NULL, 0);
    //xTaskCreatePinnedToCore(vMotor_OL_Ramp_Test, "Motor OL Ramp Test", 4096, NULL, 1, NULL, 0);
    //xTaskCreatePinnedToCore(vMotor_KV_Calc, "Motor KV Calculation", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(vMotor_Position_CL_Test, "Motor CL Position Test", 4096, NULL, 1, NULL, 0);
    
    ESP_ERROR_CHECK(esp_timer_create(&motor_ctrl_timer_args, &motor_timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(motor_timer_handle, 100));

}
}