#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/queue.h"

// Include source files for Mobile Robot Dongle Here!
#include "uart.c"
#include "esp_now.c"
// Setup any Queues here!

// Write RTOS Callback functions here! (The RTOS task can also be defined in your .c file)
// RTOS Task #1 - ESPNOW
void vESP_NOW()
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    // Run the wifi init
    wifi_init();

    // Register Callback functions
    ESP_ERROR_CHECK(esp_now_init());
    esp_now_register_recv_cb(data_recieve_cb);
    esp_now_register_send_cb(data_send_cb);

    // Setup peer info type and register
    memset(&robot_info, 0, sizeof(robot_info));
    memcpy(robot_info.peer_addr, s_mobile_robot_address, 6);
    robot_info.encrypt = false;
    robot_info.channel = 0;

    // Init loop
    int retry_counter = 0;
    int max_retries = 5;
    while(esp_now_add_peer(&robot_info) != ESP_OK)
    {
        if (retry_counter > max_retries)
        {
            ESP_LOGE(ESP_NOW_TAG,"Maximum peer add retries exceeded");
            while (1);
        }
        ESP_LOGW(ESP_NOW_TAG,"ESP NOW Failed to add peer! Retrying . . .");
        vTaskDelay(pdMS_TO_TICKS(5000));
        retry_counter++;
    }

    float counter = 0;
    while (1)
    {
        memcpy(&state_data.x_position, &counter, sizeof(counter));
        esp_now_send(s_mobile_robot_address, (u_int8_t * )&state_data, sizeof(state_data));
        counter++;
        vTaskDelay(pdMS_TO_TICKS(ESP_NOW_RATE));
    }
     
}

void app_main(void)
{
    // Prints out MAC Address
    unsigned char mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGI(ESP_NOW_TAG, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);


    // Create RTOS Tasks here using xTaskCreate:
    // Parameters: | Task callback function | Task Name | Memory Assigned to Task | Parameters to pass into the task | Priority | Task Handle
    // xTaskCreate(echo_task, "UART Echo", 2048, NULL, 1, NULL);
    xTaskCreate(vESP_NOW, "ESP NOW Wireless", 8192, NULL, 10, NULL);
    xTaskCreate(stream_task, "UART Echo", 2048, NULL, 1, NULL);
}

