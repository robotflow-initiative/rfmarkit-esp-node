/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
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
#include "esp_ota_ops.h"

#include "nvs_flash.h"
#include "soc/rtc.h"

#include "lwip/sockets.h"

#include "apps.h"
#include "blink.h"
#include "settings.h"
#include "imu.h"
#include "sys.h"

static const char *TAG = "app_main";

static void init() {
    // TODO: Add BLE function
    sys_log_chip_info();

    /** Setup up g_sleep_countup **/
    device_init_sleep_countup();

    /** Setup Button **/
    sys_button_init(CONFIG_BUTTON_GPIO_PIN);

    /** Initialize NVS **/
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    
    sys_load_configuration();


    /** Cancel GPIO hold **/
    sys_reset_gpio(CONFIG_IMU_CTRL_PIN);

    /** During init, test up the led **/
    blink_gpio_on(&g_blink_cfg);
    os_delay_ms(1000);
    blink_gpio_off(&g_blink_cfg);
    os_delay_ms(500);
    blink_gpio_on(&g_blink_cfg);
    os_delay_ms(1000);
    blink_gpio_off(&g_blink_cfg);

    /** Init global imu struct g_imu **/
    imu_init(g_imu);
    sys_delay_ms(500);
    if (imu_self_test(&g_imu) != ESP_OK) {
        sys_enter_deep_sleep();
    }

    /** Init Wi-Fi **/
    ret = sys_wifi_init_sta();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Cannot connect to AP, entering deep sleep");
        sys_enter_deep_sleep();
    }
    /** Configure wifi tx power **/
    ESP_LOGI(TAG, "Set wifi tx power level: %d", CONFIG_MAX_TX_POWER);
    esp_wifi_set_max_tx_power(CONFIG_MAX_TX_POWER);

    /** Create system event group **/
    g_mcu.sys_event_group = xEventGroupCreate();

    clear_sys_event(EV_TCP_CONNECTED);
    clear_sys_event(EV_NTP_SYNCED);
    clear_sys_event(EV_IMU_ENABLED);
    set_sys_event(EV_UART_MANUAL_BLOCK);
    blink_stop();
}

void app_main(void) {

    esp_ota_mark_app_valid_cancel_rollback();
    init();

    /** Init queues **/
    QueueHandle_t serial_queue = xQueueCreate(CONFIG_SERIAL_QUEUE_LEN, sizeof(imu_dgram_t));
    ESP_LOGI(TAG, "\nSerial Queue Addr %p\n", serial_queue);
    configASSERT(serial_queue);

    /** Launch tasks **/
#if CONFIG_MULTI_CORE
    launch_task_multicore(app_time_sync, "app_time_sync", 2560, NULL, 2, time_sync_task, 0x1);
    launch_task_multicore(app_data_client, "app_data_client", 4096, serial_queue, 2, tcp_task, 0x0);
    launch_task_multicore(app_uart_monitor, "app_uart_monitor", 4096, serial_queue, 1, uart_task, 0x0);
    launch_task_multicore(app_controller, "app_controller", 4096, NULL, 1, controller_task, 0x1);
    launch_task_multicore(blink_init_task, "blink_init", 2048, NULL, 1, blink_task, 0x1);
#else
    launch_task(app_time_sync, "app_time_sync", 2560, NULL, 2, time_sync_task);
    launch_task(app_data_client, "app_data_client", 4096, serial_queue, 2, tcp_task);
    launch_task(app_uart_monitor, "app_uart_monitor", 4096, serial_queue, 1, uart_task);
    // launch_task(app_playground, "app_playground", 2048, NULL, 1, playground_task);
    launch_task(app_controller, "app_controller", 4096, NULL, 1, controller_task);
    launch_task(blink_init_task, "blink_init", 2048, NULL, 1, blink_task);
    // TODO: Add Websocket server
#endif

    sys_log_heap_size();
    device_reset_sleep_countup();

    EventBits_t bits;
    while (1) {
        ESP_LOGI(TAG, "Main loop, g_sleep_countup: %d", g_mcu.sleep_countup);
        os_delay_ms(CONFIG_MAIN_LOOP_DUTY_PERIOD_MS);

        /** If WIFI_FAIL event occurs after init, we have a wifi interrupt. Going to deep sleep (shutdown)**/
        bits = xEventGroupGetBits(g_mcu.wifi_event_group);
        if (bits & WIFI_FAIL_BIT) {
            ESP_LOGI(TAG, "Wi-Fi interrupt, going to deep sleep");
            sys_enter_deep_sleep();
        }

        /** Check other events **/
        bits = xEventGroupGetBits(g_mcu.sys_event_group);
        if ((bits & EV_UART_MANUAL_BLOCK_BIT)) {
            device_incr_sleep_countup(1);
        }

        if (g_mcu.sleep_countup > CONFIG_MAIN_LOOP_MAX_LOOP_NUM) {
            ESP_LOGI(TAG, "Operation Timeout");
            sys_enter_deep_sleep();
            // TODO: Replace this timeout function with WDT
        }
    }
}
