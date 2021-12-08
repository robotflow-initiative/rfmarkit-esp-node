/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_wifi.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_task_wdt.h"
#include "esp_ota_ops.h"

#include "nvs_flash.h"
#include "soc/soc_caps.h"
#include "soc/rtc.h"
#include "driver/adc.h"
#include "driver/rtc_io.h"

#include "lwip/sockets.h"

#include "apps.h"
#include "blink.h"
#include "settings.h"
#include "imu.h"
#include "device.h"
#include "sys.h"

static const char* TAG = "app_main";

static void init_events() {
    /** Configure Events **/
    xEventGroupClearBits(g_mcu.sys_event_group, TCP_CONNECTED_BIT);
    xEventGroupClearBits(g_mcu.sys_event_group, NTP_SYNCED_BIT);
    xEventGroupSetBits(g_mcu.sys_event_group, IMU_ENABLED_BIT);
    xEventGroupSetBits(g_mcu.sys_event_group, UART_BLOCK_BIT);
}

static void init() { // TODO: Add BLE function
    device_log_chip_info();

    /** Setup up g_sleep_countup **/
    device_init_sleep_countup();

    /** Setup Button **/
    device_button_init(CONFIG_BUTTON_GPIO_PIN);

    /** Initialize NVS **/
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    /** Cancel GPIO hold **/
    device_reset_gpio(CONFIG_IMU_CTRL_PIN);

    /** Init global imu struct g_imu **/
    /** Setup IMU **/
    ESP_LOGI(TAG, "Setting up IMU");
    imu_init(g_imu);

    /** Init Wi-Fi **/
    ret = device_wifi_init_sta();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Cannot connect to AP, entering deep sleep");
        sys_enter_deep_sleep();
    }
    /** Configure wifi tx power **/
    ESP_LOGI(TAG, "Set wifi tx power level: %d", CONFIG_MAX_TX_POWER);
    esp_wifi_set_max_tx_power(CONFIG_MAX_TX_POWER);

    /** Create system event group **/
    g_mcu.sys_event_group = xEventGroupCreate();

    /** Blink init **/
    blink_init();

    init_events();

    if (imu_self_test(&g_imu) != ESP_OK) {
        sys_enter_deep_sleep();
    }

}

void app_main(void) {

    esp_ota_mark_app_valid_cancel_rollback();
    init();

    /** Init queues **/
    QueueHandle_t serial_queue = xQueueCreate(CONFIG_SERIAL_QUEUE_LEN, sizeof(imu_dgram_t));
    ESP_LOGD(TAG, "\nSerial Queue Addr %p\n", serial_queue);

    /** Launch tasks **/
#if CONFIG_MULTI_CORE
    launch_task_multicore(app_time_sync, "app_time_sync", 2560, NULL, 2, time_sync_task, 0x1);
    launch_task_multicore(app_data_client, "app_data_client", 4096,serial_queue, 2, tcp_task, 0x1);
    launch_task_multicore(app_uart_monitor, "app_uart_monitor", 2560,serial_queue, 1, uart_task, 0x0);
    launch_task_multicore(app_controller, "app_controller", 4096, NULL, 1, controller_task, 0x1);
#else
    launch_task(app_time_sync, "app_time_sync", 2560, NULL, 2, time_sync_task);
    launch_task(app_data_client, "app_data_client", 4096, serial_queue, 2, tcp_task);
    launch_task(app_uart_monitor, "app_uart_monitor", 2560, serial_queue, 1, uart_task);
    launch_task(app_controller, "app_controller", 4096, NULL, 1, controller_task);
#endif

    device_log_heap_size();
    device_reset_sleep_countup();

    EventBits_t bits;
    while (1) {
        ESP_LOGI(TAG, "Main loop, g_sleep_countup: %d", g_mcu.sleep_countup);
        device_delay_ms(CONFIG_MAIN_LOOP_COUNT_PERIOD_MS);

        /** If WIFI_FAIL event occurs after init, we have a wifi interrupt. Going to deep sleep (shutdown)**/
        bits = xEventGroupGetBits(g_mcu.wifi_event_group);
        if (bits & WIFI_FAIL_BIT) {
            ESP_LOGI(TAG, "Wi-Fi interrupt, going to deep sleep");
            sys_enter_deep_sleep();
        }

        /** Check other events **/
        bits = xEventGroupGetBits(g_mcu.sys_event_group);
        if ((bits & UART_BLOCK_BIT)) {
            device_incr_sleep_countup(1);
        }

        if (g_mcu.sleep_countup > CONFIG_MAIN_LOOP_MAX_COUNT_NUM) {
            ESP_LOGI(TAG, "Operation Timeout");
            sys_enter_deep_sleep(); // TODO: Replace with WDT
        }
    }
}
