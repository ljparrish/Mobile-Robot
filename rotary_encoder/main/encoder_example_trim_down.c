#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

static const char *TAG = "motor";

#define MOTOR_PCNT_HIGH_LIMIT 10000
#define MOTOR_PCNT_LOW_LIMIT  -10000

#define MOTOR_EC11_GPIO_A 16
#define MOTOR_EC11_GPIO_B 15

void app_main(void)
{
    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = MOTOR_PCNT_HIGH_LIMIT,
        .low_limit = MOTOR_PCNT_LOW_LIMIT,
        .flags.accum_count = 1,
    };

    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = MOTOR_EC11_GPIO_A,
        .level_gpio_num = MOTOR_EC11_GPIO_B,
    };

    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = MOTOR_EC11_GPIO_B,
        .level_gpio_num = MOTOR_EC11_GPIO_A,
    };

    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));



    //pcnt_event_callbacks_t cbs = {
       // .on_reach = example_pcnt_on_reach,
     //  };
  //  QueueHandle_t queue = xQueueCreate(10, sizeof(int));
   // ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, NULL, queue));

    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

#if CONFIG_EXAMPLE_WAKE_UP_LIGHT_SLEEP
    // EC11 channel output high level in normal state, so we set "low level" to wake up the chip
    ESP_ERROR_CHECK(gpio_wakeup_enable(MOTOR_EC11_GPIO_A, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
    ESP_ERROR_CHECK(esp_light_sleep_start());
#endif

 // Report counter value
    int pulse_count = 0;
    // int event_count = 0;
    while (1) {
        //if (xQueueReceive(queue, &event_count, pdMS_TO_TICKS(50))) {
           // ESP_LOGI(TAG, "Watch point event, count: %d", event_count);
       // if {

        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
        ESP_LOGI(TAG, "Pulse count: %d", pulse_count);
        ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
        vTaskDelay(pdMS_TO_TICKS(500));
    //}
    }
}
