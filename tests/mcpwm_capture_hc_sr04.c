#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_private/esp_clk.h"
#include "driver/mcpwm_cap.h"
#include "driver/gpio.h"
#include "driver/mcpwm_types.h"

#define HC_SR04_TRIG_GPIO_1 GPIO_NUM_10
#define HC_SR04_ECHO_GPIO_1 GPIO_NUM_35

#define HC_SR04_TRIG_GPIO_2 GPIO_NUM_9
#define HC_SR04_ECHO_GPIO_2 GPIO_NUM_36

#define HC_SR04_TRIG_GPIO_3 GPIO_NUM_6
#define HC_SR04_ECHO_GPIO_3 GPIO_NUM_13 

// Tag for logging
const static char *TAG = "MCPWM_ultrasonic";


static bool hc_sr04_echo_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    static uint32_t cap_val_begin_of_sample = 0;
    static uint32_t cap_val_end_of_sample = 0;
    TaskHandle_t task_to_notify = (TaskHandle_t)user_data;
    BaseType_t high_task_wakeup = pdFALSE;

    // calculate the interval in the ISR (Interrupt Service Routine)
    // so that the interval will be always correct even when capture_queue is not handled in time and overflow.
    if (edata->cap_edge == MCPWM_CAP_EDGE_POS) {
        // store the timestamp when positive edge is detected
        cap_val_begin_of_sample = edata->cap_value;
        cap_val_end_of_sample = cap_val_begin_of_sample;
    } else {
        // store the timestamp when negative edge is detected
        cap_val_end_of_sample = edata->cap_value;
        uint32_t tof_ticks = cap_val_end_of_sample - cap_val_begin_of_sample;

        // notify the task to calculate the distance
        // eSetValueWithOverwrite: if the notification value is already set,
        //                         it should be overwritten with the new value (tof_ticks)
        //                         ensures that only the latest value of tof_ticks is sent to the task,
        //                         and any previous notifications with different values are discarded.

        xTaskNotifyFromISR(task_to_notify, tof_ticks, eSetValueWithOverwrite, &high_task_wakeup);
    }

    return high_task_wakeup == pdTRUE;
}

/**
 * @brief generate single pulse on Trig pin to start a new sample
 */

static void gen_trig_output(uint32_t gpio_pin)
{
    gpio_set_level(gpio_pin, 1); // set high
    esp_rom_delay_us(10);        // delay execution for 10 microseconds
    gpio_set_level(gpio_pin, 0); // set low
}


void app_main(void)
{
    ESP_LOGI(TAG, "Install capture timer");
    mcpwm_cap_timer_handle_t cap_timer= NULL;
    mcpwm_capture_timer_config_t cap_conf = {
        .clk_src = MWDT_CLK_SRC_DEFAULT,
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf, &cap_timer));


    ESP_LOGI(TAG, "Install capture channel 1");
    mcpwm_cap_channel_handle_t cap_chan = NULL;
    mcpwm_capture_channel_config_t cap_ch_conf = {
        .gpio_num = HC_SR04_ECHO_GPIO_1,
        .prescale = 1,
        // capture on both edge
        .flags.neg_edge = true,
        .flags.pos_edge = true,
        // pull up internally
        .flags.pull_up = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_conf, &cap_chan));

    ESP_LOGI(TAG, "Install capture channel 2");
    mcpwm_cap_channel_handle_t cap_chan_2 = NULL;
    mcpwm_capture_channel_config_t cap_ch_conf_2 = {
        .gpio_num = HC_SR04_ECHO_GPIO_2,
        .prescale = 1,
        .flags.neg_edge = true,
        .flags.pos_edge = true,
        .flags.pull_up = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_conf_2, &cap_chan_2));

    ESP_LOGI(TAG, "Install capture channel 3");
    mcpwm_cap_channel_handle_t cap_chan_3 = NULL;
    mcpwm_capture_channel_config_t cap_ch_conf_3 = {
        .gpio_num = HC_SR04_ECHO_GPIO_3,
        .prescale = 1,
        .flags.neg_edge = true,
        .flags.pos_edge = true,
        .flags.pull_up = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_conf_3, &cap_chan_3));

    ESP_LOGI(TAG, "Register capture callback");
    // FreeRTOS function to obtain the handle of the currently running task
    TaskHandle_t cur_task = xTaskGetCurrentTaskHandle();
    // mcpwm_capture_event_callbacks_t: a structure provided by the MCPWM driver/library
    // to specify callback functions for various events related to MCPWM capture operations
    mcpwm_capture_event_callbacks_t cbs_1 = {
        .on_cap = hc_sr04_echo_callback,
    };
    // mcpwm_cap_channel_handle_t cap_channel, const mcpwm_capture_event_callbacks_t *cbs, void *user_data
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs_1, cur_task));
    // // mcpwm_cap_channel_handle_t cap_channel, const mcpwm_capture_event_callbacks_t *cbs, void *user_data

    mcpwm_capture_event_callbacks_t cbs_2 = {
        .on_cap = hc_sr04_echo_callback,
    };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan_2, &cbs_2, cur_task));
    mcpwm_capture_event_callbacks_t cbs_3 = {
        .on_cap = hc_sr04_echo_callback,
    };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan_3, &cbs_3, cur_task));

    ESP_LOGI(TAG, "Enable capture channel 1");
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan));
    ESP_LOGI(TAG, "Enable capture channel 2");
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan_2));
    ESP_LOGI(TAG, "Enable capture channel 3");
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan_3)); 
    
    ESP_LOGI(TAG, "Configure Trig pin 1");
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << HC_SR04_TRIG_GPIO_1,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    // drive low by default
    ESP_ERROR_CHECK(gpio_set_level(HC_SR04_TRIG_GPIO_1, 0));

    ESP_LOGI(TAG, "Configure Trig pin 2");
    gpio_config_t io_conf_2 = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << HC_SR04_TRIG_GPIO_2,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf_2));
    // drive low by default
    ESP_ERROR_CHECK(gpio_set_level(HC_SR04_TRIG_GPIO_2, 0));

    ESP_LOGI(TAG, "Configure Trig pin 3");
    gpio_config_t io_conf_3 = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << HC_SR04_TRIG_GPIO_3,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf_3));
    // drive low by default
    ESP_ERROR_CHECK(gpio_set_level(HC_SR04_TRIG_GPIO_3, 0));


    ESP_LOGI(TAG, "Enable and start capture timer");
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));



    uint32_t tof_ticks_1, tof_ticks_2, tof_ticks_3;
    while (1) {

        // trigger the sensor to start a new sample
        gen_trig_output(HC_SR04_TRIG_GPIO_1);
        ESP_LOGI(TAG, "1: Triggerd");
        // wait for echo done signal 0x01 
        // ulBitsToClearOnEntry, ulBitsToClearOnExit, pulNotificationValue, xTicksToWait )
        ESP_LOGI(TAG, "1: tof_ticks: %lu", (unsigned long) tof_ticks_1); 
        if (xTaskNotifyWait(tof_ticks_1, ULONG_MAX, &tof_ticks_1, pdMS_TO_TICKS(1000)) == pdTRUE) {
            float pulse_width_us_1 = tof_ticks_1 * (1000000.0 / esp_clk_apb_freq());
            if (pulse_width_us_1 > 35000) {
                // out of range
                continue;
            }
            // convert the pulse width into measure distance
            float distance_1 = (float) pulse_width_us_1 / 58;
            ESP_LOGI(TAG, "1: Measured distance: %.2fcm", distance_1);
        }
        ESP_LOGI(TAG, "1: tof_ticks: %lu", (unsigned long) tof_ticks_1); 

        ESP_LOGI(TAG, "-----------------"); 
        vTaskDelay(pdMS_TO_TICKS(500));

        // trigger the sensor to start a new sample
        gen_trig_output(HC_SR04_TRIG_GPIO_2);
        ESP_LOGI(TAG, "2: Triggerd");
        // wait for echo done signal
        ESP_LOGI(TAG, "2: tof_ticks: %lu", (unsigned long) tof_ticks_2); 
        if (xTaskNotifyWait(tof_ticks_2, ULONG_MAX, &tof_ticks_2, pdMS_TO_TICKS(1000)) == pdTRUE) {
            float pulse_width_us_2 = tof_ticks_2 * (1000000.0 / esp_clk_apb_freq()); 
            if (pulse_width_us_2 > 35000) {
                // out of range
                continue;
            }
            // convert the pulse width into measure distance
            float distance_2 = (float) pulse_width_us_2 / 58;
            ESP_LOGI(TAG, "2: Measured distance: %.2fcm", distance_2);
        }
        ESP_LOGI(TAG, "2: tof_ticks: %lu", (unsigned long) tof_ticks_2); 

        ESP_LOGI(TAG, "-----------------"); 

        vTaskDelay(pdMS_TO_TICKS(500));

        // trigger the sensor to start a new sample
        gen_trig_output(HC_SR04_TRIG_GPIO_3);
        ESP_LOGI(TAG, "3: Triggerd");
        // wait for echo done signal 0x01 
        // ulBitsToClearOnEntry, ulBitsToClearOnExit, pulNotificationValue, xTicksToWait )
        ESP_LOGI(TAG, "3: tof_ticks: %lu", (unsigned long) tof_ticks_3); 
        if (xTaskNotifyWait(tof_ticks_3, ULONG_MAX, &tof_ticks_3, pdMS_TO_TICKS(1000)) == pdTRUE) {
            float pulse_width_us_3 = tof_ticks_3 * (1000000.0 / esp_clk_apb_freq());
            if (pulse_width_us_3 > 35000) {
                // out of range
                continue;
            }
            // convert the pulse width into measure distance
            float distance_3 = (float) pulse_width_us_3 / 58;
            ESP_LOGI(TAG, "3: Measured distance: %.2fcm", distance_3);
        }
        ESP_LOGI(TAG, "3: tof_ticks: %lu", (unsigned long) tof_ticks_3);

        ESP_LOGI(TAG, "-----------------"); 

        vTaskDelay(pdMS_TO_TICKS(500)); 
    }
}