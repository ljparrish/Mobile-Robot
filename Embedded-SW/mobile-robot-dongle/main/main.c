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

// Define parameters here
#define debug_statements 1 // Use this flag to control console debug statements

// Setup any Queues here!
static QueueHandle_t robot_cmd_queue;
static QueueHandle_t robot_info_queue;

// ESP NOW Data Recieved Callback
static void data_recieve_cb(const uint8_t *mac_address, uint8_t *incomingData, int length)
{
    // Copy recieved data from the buffer
    memcpy(&state_data, incomingData, sizeof(state_data));

    // Send into the UART Queue
    xQueueSend(robot_info_queue, (void*)&state_data, 5);
    if(debug_statements)
    {
        ESP_LOGI(ESP_NOW_TAG,"Data recieved:");
        ESP_LOGI(ESP_NOW_TAG,"w_r : %d", state_data.w_right);
        ESP_LOGI(ESP_NOW_TAG,"w_l : %d", state_data.w_left);
        ESP_LOGI(ESP_NOW_TAG,"u_l : %d", state_data.ultrasonic_left);
        ESP_LOGI(ESP_NOW_TAG,"u_c : %d", state_data.ultrasonic_center);
        ESP_LOGI(ESP_NOW_TAG,"u_r : %d", state_data.ultrasonic_right);
        ESP_LOGI(ESP_NOW_TAG,"u_r : %d", state_data.counter);
    }
}

// Write RTOS Callback functions here! (The RTOS task can also be defined in your .c file)
// RTOS Task #1 - ESPNOW
void vESP_NOW()
{
    // Initialize NVS
    init_nvs();

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

    mobile_robot_command_t cmd;
    while (1)
    {
        // Check to see if we have a command signal to be sent to the robot in the queue
        if (xQueueReceive(robot_cmd_queue, (void*)&cmd, 0) == pdTRUE)
        {
            // Send the data from the queue
            esp_now_send(s_mobile_robot_address, (u_int8_t *)&cmd, sizeof(cmd));
        }

        // Task Delay
        vTaskDelay(pdMS_TO_TICKS(ESP_NOW_RATE));
    }
     
}

// RTOS Task #2 - UART
void vUART_Communication()
{
    // Setup and configure the UART for Data transmission
    uart_setup();

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (1)
    {
        // Check if there are any available bytes to read from the UART
        if (uart_read_bytes(ECHO_UART_PORT_NUM, (const char *) data, sizeof(robot_cmd), 0) == sizeof(mobile_robot_command_t))
        {
            // Number of bytes recieved is equal to the size of robot_cmd (we recieved the number of bytes we expected)
            // Copy the data from the buffer into robot_cmd
            memcpy(&robot_cmd, data, sizeof(robot_cmd));

            // Send the data into the queue
            xQueueSend(robot_cmd_queue, (void*)&robot_cmd, 5);
        }
        
        // Check if there is data in the robot_info_queue, we send it back via UART
        if (xQueueReceive(robot_info_queue, (void*)&robot_info, 0) == pdTRUE)
        {
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char *)&robot_info, sizeof(robot_info));
        }
        
        // Task Delay
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    
}

void app_main(void)
{
    // Prints out MAC Address
    unsigned char mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGI(ESP_NOW_TAG, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);

    // Create Queues using xQueueCreate:
    // Parameters: | Number of values that can be stored in a queue | size in bytes of each variable the queue takes |
    robot_cmd_queue = xQueueCreate(10, sizeof(mobile_robot_command_t));
    robot_info_queue = xQueueCreate(10, sizeof(mobile_robot_state_info_t)); 

    // Create RTOS Tasks here using xTaskCreate:
    // Parameters: | Task callback function | Task Name | Memory Assigned to Task | Parameters to pass into the task | Priority | Task Handle
    // xTaskCreate(echo_task, "UART Echo", 2048, NULL, 1, NULL);
    xTaskCreate(vESP_NOW, "ESP NOW Wireless", 8192, NULL, 10, NULL);
    xTaskCreate(vUART_Communication, "UART Coms", 2048, NULL, 1, NULL);
}

