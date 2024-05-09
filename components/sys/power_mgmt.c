//
// Created by liyutong on 2024/4/30.
//
#include <stdlib.h>

#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_event.h"
#include "esp_attr.h"
#include "esp_log.h"

#include "apps.h"
#include "settings.h"
#include "sys.h"
#include "hi229.h"
#include "blink.h"

static SemaphoreHandle_t sync_mutex = NULL;
RTC_FAST_ATTR power_mgmt_info_t g_power_mgmt_info;

static const char *TAG = "sys.pm";

esp_err_t power_mgmt_init() {
    /** Cancel hold **/
    gpio_hold_dis(CONFIG_IMU_CTRL_PIN);
    gpio_hold_dis(CONFIG_BLINK_PIN);
    gpio_deep_sleep_hold_dis();
    /** Enable gpio hold in deep sleep **/
    gpio_deep_sleep_hold_en();

    ++g_power_mgmt_info.boot_count;
    ESP_LOGI(TAG, "system booted %d times", g_power_mgmt_info.boot_count);

    if (g_power_mgmt_info.initialized) {
        if (g_power_mgmt_info.state == POWER_DEEP_SLEEP) {
            g_power_mgmt_info.state = POWER_WAKEN;
            // Do not change power mode
            g_power_mgmt_info.mutex = xSemaphoreCreateMutex();
            return ESP_OK;
        } else {
            g_power_mgmt_info.state = POWER_UNKNOWN;
            g_power_mgmt_info.mutex = NULL;
            return ESP_FAIL;
        }
    } else {
        g_power_mgmt_info.initialized = true;
        g_power_mgmt_info.state = POWER_NORMAL_BOOT;
        g_power_mgmt_info.mode = POWER_MODE_NORMAL;
        g_power_mgmt_info.mutex = xSemaphoreCreateMutex();
        return ESP_OK;
    }
}

power_mgmt_state_t power_mgmt_wake_up_handler() {
    return POWER_NORMAL_BOOT;
}

esp_err_t power_mgmt_on_enter_standby() {
    g_power_mgmt_info.state = POWER_STANDBY;
    // TODO: Implement standby logic
    return ESP_OK;
}

esp_err_t power_mgmt_on_enter_active() {
    /** TOOD: Turn wifi power save off **/
    return ESP_OK;
}

esp_err_t power_mgmt_on_enter_power_save() {
    /** TOOD: Turn off wifi and BLE if they are on**/
    return ESP_OK;
}

esp_err_t power_mgmt_on_enter_deep_sleep(bool wakeup) {
    ESP_LOGI(TAG, "disabling IMU and led");

    imu_disable(&g_imu);
    g_mcu.state.led_manual = true;
    os_delay_ms(100);

    /** Operate LED to indicate deep sleep **/
    blink_led_on();
    os_delay_ms(2000);
    blink_led_off();

    /** Hold pins in deep sleep **/
    gpio_hold_en(CONFIG_IMU_CTRL_PIN);
    gpio_hold_en(CONFIG_BLINK_PIN);

    if (wakeup) {
        esp_sleep_enable_timer_wakeup(CONFIG_WAKE_UP_TIME_SEC * 1000000LL);
        ESP_LOGI(TAG, "entering deep sleep for %d seconds...", CONFIG_WAKE_UP_TIME_SEC);
    } else {
        ESP_LOGI(TAG, "entering deep sleep forever...");
    }
    esp_sleep_enable_ext0_wakeup(CONFIG_BUTTON_FN_GPIO_PIN, CONFIG_BUTTON_FN_ACTIVE_LEVEL);
    g_power_mgmt_info.state = POWER_DEEP_SLEEP;
    esp_deep_sleep_start();
    return ESP_OK;
}

//static esp_err_t power_mgmt_handle_transition(power_mgmt_transition_t transition) {
//    return ESP_OK;
//}

void sys_power_mgmt_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data) {
    /** TODO: Implement logic**/
    if (!sync_mutex) {
        sync_mutex = xSemaphoreCreateMutex();
    }
    if (xSemaphoreTake(sync_mutex, portMAX_DELAY) == pdTRUE) {
        switch (id) {
            case SYSTEM_POWER_MGMT_EVENT:
                ESP_LOGI(TAG, "power management event triggered for regular update");
                break;
            case SYSTEM_MODE_CHANGE_EVENT:
                ESP_LOGI(TAG, "power management event triggered for mode change");
                break;
            case SYSTEM_OTA_EVENT:
                ESP_LOGI(TAG, "power management event triggered for ota event");
                break;
            default:
                break;
        }
        xSemaphoreGive(sync_mutex);
    }
}


