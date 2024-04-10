
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"

#include "include/esp_now_structures.h"

#define ESPNOW_MAXDELAY 512
#define ESP_NOW_RATE 500

static const char *ESP_NOW_TAG = "ESP_NOW";

// Define the mobile robot's ESP32 MAC Address here!
static uint8_t s_mobile_robot_address[ESP_NOW_ETH_ALEN] = {0x70, 0x04, 0x1D, 0xCD, 0xBC, 0x50};

// Define the USB Dongle ESP32 MAC Address here!
static uint8_t s_usb_dongle_address[ESP_NOW_ETH_ALEN] = {0xAC, 0x0B, 0xFB, 0x68, 0x1F, 0x8C};

// Global Variables
mobile_robot_state_info_t state_data;

esp_now_peer_info_t robot_info;

// WiFi should start before using ESPNOW 
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK( esp_wifi_start());
}

static void data_send_cb(const uint8_t *mac_address, esp_now_send_status_t status)
{
    ESP_LOGI(ESP_NOW_TAG,"Data send status: %i",status);
}

static void data_recieve_cb(const uint8_t *mac_address, uint8_t *incomingData, int length)
{
    ESP_LOGI(ESP_NOW_TAG,"Data recieved:");
    memcpy(&state_data, incomingData, sizeof(state_data));
    ESP_LOGI(ESP_NOW_TAG,"X : %f",state_data.x_position);
    ESP_LOGI(ESP_NOW_TAG,"Y : %f",state_data.y_position);
    ESP_LOGI(ESP_NOW_TAG,"T : %f",state_data.theta);
    ESP_LOGI(ESP_NOW_TAG, "Counter : %i", state_data.counter);
}
