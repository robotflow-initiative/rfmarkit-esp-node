//
// Created by liyutong on 2024/4/29.
//
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_event.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"

#include "apps.h"
#include "settings.h"
#include "sys.h"

ESP_EVENT_DEFINE_BASE(SYSTEM_EVENTS);

static const char *TAG = "app.system_loop ";

static void discovery_timer_cb(TimerHandle_t xTimer) {
    /** Send discovery packets if Wi-Fi connected **/
    if (g_mcu.state.wifi_connected) {
        esp_event_post_to(g_mcu.system_loop, SYSTEM_EVENTS, SYSTEM_DISCOVERY_EVENT, NULL, 0, portMAX_DELAY);
    }
}

static void time_sync_timer_cb(TimerHandle_t xTimer) {
    /** Sync time (continuous) if Wi-Fi connected **/
    if ((g_mcu.state.wifi_connected && !get_task_event(EV_TASK_TIME_SYNC)) || !g_mcu.state.ntp_synced) {
        set_task_event(EV_TASK_TIME_SYNC);
        esp_event_post_to(g_mcu.system_loop, SYSTEM_EVENTS, SYSTEM_TIME_SYNC_EVENT, NULL, 0, portMAX_DELAY);
    }
}

static void power_mgmt_timer_cb(TimerHandle_t xTimer) {
    /** Power Mgmt Event **/
    if (g_power_mgmt_ctx.state != POWER_SAVE) {
        esp_event_post_to(g_mcu.system_loop, SYSTEM_EVENTS, SYSTEM_POWER_MGMT_EVENT, NULL, 0, portMAX_DELAY);
    }
}

_Noreturn void app_system_loop(void *pvParameters) {
    ESP_LOGI(TAG, "system loop started");

    esp_event_loop_args_t loop_args = {
        .queue_size = 16,
        .task_name = "system_event_loop_task",
        .task_priority = uxTaskPriorityGet(NULL),
        .task_stack_size = 4096,
        .task_core_id = tskNO_AFFINITY,
    };

    /** Register events **/
    ESP_ERROR_CHECK(esp_event_loop_create(&loop_args, &g_mcu.system_loop));
    ESP_ERROR_CHECK(esp_event_handler_register_with(g_mcu.system_loop, SYSTEM_EVENTS, SYSTEM_LED_STATUS_EVENT, sys_led_status_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(g_mcu.system_loop, SYSTEM_EVENTS, SYSTEM_MODE_CHANGE_EVENT, sys_mode_change_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(g_mcu.system_loop, SYSTEM_EVENTS, SYSTEM_POWER_MGMT_EVENT, sys_power_mgmt_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(g_mcu.system_loop, SYSTEM_EVENTS, SYSTEM_MODE_CHANGE_EVENT, sys_power_mgmt_handler, NULL));

    /** Launch timers **/
    g_mcu.timers.power_mgmt_timer = xTimerCreate("power_mgmt_timer", pdMS_TO_TICKS(CONFIG_POWER_MGMT_DURATION_S * 1000), pdTRUE, NULL, power_mgmt_timer_cb);
    if (xTimerStart(g_mcu.timers.power_mgmt_timer, 0) == pdPASS) ESP_LOGI(TAG, "power mgmt timer started");

    while (1) {
        /** wait for events **/
        EventBits_t bits = xEventGroupWaitBits(
            g_mcu.sys_event_group,
            0x00FFFFFF,
            pdTRUE,
            pdFALSE,
            portMAX_DELAY
        );
        ESP_LOGI(TAG, "bits=%x", bits);

        /** Power Management **/
        if (bits & EV_SYS_POWER_MGMT_BIT) {
            ESP_LOGI(TAG, "power management event");
            esp_event_post_to(g_mcu.system_loop, SYSTEM_EVENTS, SYSTEM_POWER_MGMT_EVENT, NULL, 0, portMAX_DELAY);
        }

        /** Mode Change **/
        if (bits & EV_SYS_MODE_CHANGE_BIT) {
            ESP_LOGI(TAG, "mode change event");
            esp_event_post_to(g_mcu.system_loop, SYSTEM_EVENTS, SYSTEM_MODE_CHANGE_EVENT, NULL, 0, portMAX_DELAY);
        }

        /** BLE Connection **/
        if (bits & EV_SYS_BLE_CONN_UPDATE_BIT) {
            ESP_LOGI(TAG, "BLE connection update event");
            esp_event_post_to(g_mcu.system_loop, SYSTEM_EVENTS, SYSTEM_MODE_CHANGE_EVENT, NULL, 0, portMAX_DELAY);
        }

        /** Blink LED Configuration **/
        if (!get_task_event(EV_TASK_LED_STATUS) && !g_mcu.state.led_manual) {
            set_task_event(EV_TASK_LED_STATUS);
            esp_event_post_to(g_mcu.system_loop, SYSTEM_EVENTS, SYSTEM_LED_STATUS_EVENT, NULL, 0, portMAX_DELAY);
        }
    }
}