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
#include "nvs_flash.h"
#include "soc/soc_caps.h"
#include "soc/rtc.h"
#include "driver/adc.h"
#include "driver/rtc_io.h"


#include "lwip/sockets.h"
#include "coap/coap.h"

#include "apps.h"
#include "types.h"
#include "settings.h"
#include "funcs.h"
#include "gy95.h"
#include "events.h"

static const char* TAG = "app_main";
static TaskHandle_t tcp_task = NULL;
static TaskHandle_t uart_task = NULL;
static TaskHandle_t time_sync_task = NULL;


RTC_DATA_ATTR static int boot_count = 0;

void init() {
    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "This is %s chip with %d CPU core(s), WiFi%s%s, ",
             CONFIG_IDF_TARGET,
             chip_info.cores,
             (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    ESP_LOGI(TAG, "silicon revision %d, ", chip_info.revision);

    ESP_LOGI(TAG, "%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
             (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    ESP_LOGI(TAG, "Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
    ++boot_count;
    ESP_LOGI(TAG, "Boot count %d", boot_count);
    esp_get_device_id();
    ESP_LOGI(TAG, "Device ID: %s", g_device_id);

    /** Initialize NVS **/
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    wifi_init_sta();
    /** Configure wifi tx power **/
    ESP_LOGI(TAG, "set wifi tx power level: %d", CONFIG_MAX_TX_POWER);
    esp_wifi_set_max_tx_power(CONFIG_MAX_TX_POWER);

    /** Configure Timing **/

    /** Create TCP event group **/
    net_event_group = xEventGroupCreate();

    /** Enable gpio hold in deep sleep **/
    gpio_deep_sleep_hold_en();

    ESP_LOGI(TAG, "setting up gy95");
    uart_service_init(GY95_PORT, GY95_RX, GY95_TX, GY95_RTS, GY95_CTS);
    gy95_t gy;
    gy95_init(&gy, GY95_PORT, GY95_CTRL_PIN, GY95_ADDR);
    gy95_msp_init(&gy);
    
    gy95_enable();
}

void app_main(void) {

    init();

    QueueHandle_t serial_queue = xQueueCreate(128, sizeof(imu_msg_raw_t));
    // ESP_LOGI(TAG, "\n[ Serial Queue Addr] %p\n", serial_queue);
    ESP_LOGI(TAG, "launching time sync task");
    xEventGroupClearBits(net_event_group, TCP_CONNECTED_BIT);
    xEventGroupClearBits(net_event_group, NTP_SYNCED_BIT);
    xTaskCreate(app_time_sync,
                "app_time_sync",
                2560,
                NULL,
                2,
                &time_sync_task);
    if (tcp_task == NULL) {
        ESP_LOGI(TAG, "launching tcp client task");
#if CONFIG_MULTI_CORE
        xTaskCreatePinnedToCore(app_tcp_client,
                                "app_tcp_client",
                                4096,
                                (void*)serial_queue,
                                1,
                                NULL,
                                0x0);
#else
        xTaskCreate(app_tcp_client,
                    "app_tcp_client",
                    4096,
                    (void*)serial_queue,
                    1,
                    &tcp_task);

        // xTaskCreate(app_uart_monitor,
        //             "app_uart_monitor",
        //             2048,
        //             (void*)serial_queue,
        //             2,
        //             &tcp_task);
#endif
    }
    if (uart_task == NULL) {
        ESP_LOGI(TAG, "launching uart monitor task");
#if CONFIG_MULTI_CORE
        xTaskCreatePinnedToCore(app_uart_monitor,
                                "app_uart_monitor",
                                2560,
                                (void*)serial_queue,
                                1,
                                &uart_task,
                                0x0);
#else
        xTaskCreate(app_uart_monitor,
                    "app_uart_monitor",
                    2560,
                    (void*)serial_queue,
                    1,
                    &uart_task);
#endif
    }

    // app_blink("\xF1\x01\xFF\x0F", 3);
    // while (1) {esp_task_wdt_feed();}
    while (1) { vTaskDelay(1000); }
}
