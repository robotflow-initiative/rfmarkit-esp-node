//
// Created by liyutong on 2024/4/30.
//
#include <stdlib.h>


#include "esp_log.h"

#include "apps.h"
#include "settings.h"
#include "sys.h"
#include "blink.h"

static SemaphoreHandle_t sync_mutex = NULL;

static const char *TAG = "sys.led_status";

/**
 * 1. Fast Flash When Wi-Fi is not connected
 * 2. Normal Breath When Wi-Fi is connected but NTP is not synced / NotDiscovered
 * 3. Slow Flash When Wi-Fi is connected and NTP is synced
 * 4. LED on when is enabled
**/
static led_status_enum get_target_led_status() {
    if (!(g_mcu.state.wifi_connected)) {
        return LED_FAST_FLASH;
    } else {
        if (!(g_mcu.state.ntp_synced) || !(g_mcu.state.discovery_completed)) {
            return LED_FAST_BREATH;
        } else {
            if (g_mcu.state.active) {
                return LED_ON;
            } else {
                return LED_SLOW_BREATH;
            }
        }
    }

}

/**
 * Set LED status according to the target
 * @param target
 * @return
**/
esp_err_t sys_set_led_status(led_status_enum target) {
    switch (target) {
        case LED_ON:
            blink_led_on();
            return ESP_OK;
        case LED_OFF:
            blink_led_off();
            return ESP_OK;
        case LED_FAST_FLASH:
            blink_led_fast_flash();
            return ESP_OK;
        case LED_FAST_BREATH:
            blink_led_start_fast_breath_pattern();
            return ESP_OK;
        case LED_SLOW_BREATH:
            blink_led_start_slow_breath_pattern();
            return ESP_OK;
        case LED_SEQ_ENC:
            blink_start_seq_enc_pattern();
            return ESP_OK;
        default:
            return ESP_FAIL;
    }
}

/**
 * LED status handler
 * @param handler_args
 * @param base
 * @param id
 * @param event_data
**/
void sys_led_status_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data) {
    ESP_LOGD(TAG, "sys_led_status event triggered");
    if (!sync_mutex) {
        sync_mutex = xSemaphoreCreateMutex();
    }
    if (xSemaphoreTake(sync_mutex, portMAX_DELAY) == pdTRUE) {
        led_status_enum target_led_status = get_target_led_status();
        ESP_LOGD(TAG, "target_led_status=%d", target_led_status);
        if (target_led_status == g_mcu.state.led_status) {
            xSemaphoreGive(sync_mutex);
            clear_task_event(EV_TASK_LED_STATUS);
            return;
        }
        sys_set_led_status(target_led_status);

        xSemaphoreGive(sync_mutex);
        clear_task_event(EV_TASK_LED_STATUS);
        return;
    }

}