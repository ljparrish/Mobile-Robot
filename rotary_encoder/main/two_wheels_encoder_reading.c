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

#define LEFT_EC11_GPIO_A 16
#define LEFT_EC11_GPIO_B 15

#define RIGHT_EC11_GPIO_A 38
#define RIGHT_EC11_GPIO_B 39

void app_main(void)
{
    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = MOTOR_PCNT_HIGH_LIMIT,
        .low_limit = MOTOR_PCNT_LOW_LIMIT,
        .flags.accum_count = 1,
    };

    pcnt_unit_handle_t pcnt_unit_left = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit_left));
    pcnt_unit_handle_t pcnt_unit_right = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit_right));


    // LEFT WHEEL CHANNEL SET UP

    ESP_LOGI(TAG, "install pcnt channels LEFT");
    pcnt_chan_config_t chan_a_left_config = {
        .edge_gpio_num = LEFT_EC11_GPIO_A,
        .level_gpio_num = LEFT_EC11_GPIO_B,
    };

    pcnt_channel_handle_t pcnt_chan_a_left = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_left, &chan_a_left_config, &pcnt_chan_a_left));
    pcnt_chan_config_t chan_b_left_config = {
        .edge_gpio_num = LEFT_EC11_GPIO_B,
        .level_gpio_num = LEFT_EC11_GPIO_A,
    };

    pcnt_channel_handle_t pcnt_chan_b_left = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_left, &chan_b_left_config, &pcnt_chan_b_left));

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels LEFT");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a_left, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a_left, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b_left, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b_left, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // RIGHT WHEEL CHANNEL SET UP

    ESP_LOGI(TAG, "install pcnt channels RIGHT");
    pcnt_chan_config_t chan_a_right_config = {
        .edge_gpio_num = RIGHT_EC11_GPIO_A,
        .level_gpio_num = RIGHT_EC11_GPIO_B,
    };

    pcnt_channel_handle_t pcnt_chan_a_right = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_right, &chan_a_right_config, &pcnt_chan_a_right));
    pcnt_chan_config_t chan_b_right_config = {
        .edge_gpio_num = RIGHT_EC11_GPIO_B,
        .level_gpio_num = RIGHT_EC11_GPIO_A,
    };

    pcnt_channel_handle_t pcnt_chan_b_right = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_right, &chan_b_right_config, &pcnt_chan_b_right));

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels RIGHT");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a_right, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a_right, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b_right, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b_right, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));



    //pcnt_event_callbacks_t cbs = {
       // .on_reach = example_pcnt_on_reach,
     //  };
  //  QueueHandle_t queue = xQueueCreate(10, sizeof(int));
   // ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, NULL, queue));

// LEFT WHEEL

    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_left));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_left));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_left));

// RIGHT WHEEL

    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_right));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_right));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_right));

// #if CONFIG_EXAMPLE_WAKE_UP_LIGHT_SLEEP
//     // EC11 channel output high level in normal state, so we set "low level" to wake up the chip
//     ESP_ERROR_CHECK(gpio_wakeup_enable(LEFT_EC11_GPIO_A, GPIO_INTR_LOW_LEVEL));
//     ESP_ERROR_CHECK(gpio_wakeup_enable(RIGHT_EC11_GPIO_A, GPIO_INTR_LOW_LEVEL));
//     ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
//     ESP_ERROR_CHECK(esp_light_sleep_start());
// #endif

 // Report counter value
    int pulse_count_left = 0;
    int pulse_count_right = 0;
    // int event_count = 0;
    while (1) {
        //if (xQueueReceive(queue, &event_count, pdMS_TO_TICKS(50))) {
           // ESP_LOGI(TAG, "Watch point event, count: %d", event_count);
       // if {

        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit_left, &pulse_count_left));
        ESP_LOGI(TAG, "Pulse count LEFT: %d", pulse_count_left);
        ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_left));

        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit_right, &pulse_count_right));
        ESP_LOGI(TAG, "Pulse count RIGHT: %d", pulse_count_right);
        ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_right));
        vTaskDelay(pdMS_TO_TICKS(500));
    //}
    }
}
