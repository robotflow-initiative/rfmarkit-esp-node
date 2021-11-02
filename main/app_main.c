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
#include "types.h"
#include "settings.h"
#include "functions.h"
#include "gy95.h"
#include "events.h"
#include "main.h"

static const char* TAG = "app_main";
static TaskHandle_t tcp_task = NULL;
static TaskHandle_t uart_task = NULL;
static TaskHandle_t time_sync_task = NULL;
static TaskHandle_t controller_task = NULL;


gy95_t g_gy95_imu = { 0 };
int g_debug_sock = -1;
char debug_buffer[128] = {0};

RTC_DATA_ATTR static int boot_count = 0;

static void init() {
    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI("","\n\n\n\n\n\n# -------- Begin of app log -------- #");
    ESP_LOGI(TAG, "This is %s chip with %d CPU core(s), WiFi%s%s, ",
             CONFIG_IDF_TARGET,
             chip_info.cores,
             (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    ESP_LOGI(TAG, "silicon revision %d, ", chip_info.revision);

    ESP_LOGI(TAG, "%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
             (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    ESP_LOGW(TAG, "Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
    ++boot_count;
    ESP_LOGI(TAG, "Boot count %d", boot_count);
    esp_get_device_id();
    ESP_LOGI(TAG, "Device ID: %s", g_device_id);

    /** Setup GY95 **/
    // TODO: Decide if we should join uart_service_init to gy95_msp_init
    ESP_LOGI(TAG, "setting up gy95");
    uart_service_init(GY95_PORT, GY95_RX, GY95_TX, GY95_RTS, GY95_CTS);

    /** Cancel GPIO hold **/ 
    //TODO: Verify this will not cause trouble
    gpio_hold_dis(GY95_CTRL_PIN);
    gpio_deep_sleep_hold_dis();

    /** Init global imu struct g_gy95_imu **/
    /** @remark g_gy95_imu will not be used by app_uart_monitor (currently). It serves for cross function enable/disable and msp_init **/
    gy95_init(&g_gy95_imu, GY95_PORT, GY95_CTRL_PIN, GY95_ADDR);
    gy95_msp_init(&g_gy95_imu);
    gy95_enable(&g_gy95_imu);

    /** Initialize NVS **/
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    
    ESP_ERROR_CHECK(ret);
    ret = wifi_init_sta();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Cannot connect to AP, entering deep sleep");
        esp_enter_deep_sleep();
    }

    /** Configure wifi tx power **/
    ESP_LOGI(TAG, "set wifi tx power level: %d", CONFIG_MAX_TX_POWER);
    esp_wifi_set_max_tx_power(CONFIG_MAX_TX_POWER);

    // /** Check Update **/
    ESP_LOGW(TAG, "\n-------VERSION-------\nv%s\n---------END---------", __FIRMWARE_VERSION__);
    // ESP_LOGW(TAG, "Will try to update in 3 seconds");
    // vTaskDelay(3000 / portTICK_PERIOD_MS);
    // esp_do_ota();

    /** Create TCP event group **/
    g_sys_event_group = xEventGroupCreate();

    /** Enable gpio hold in deep sleep **/
    gpio_deep_sleep_hold_en();


    /** Configure Events **/
    xEventGroupClearBits(g_sys_event_group, TCP_CONNECTED_BIT);
    xEventGroupClearBits(g_sys_event_group, NTP_SYNCED_BIT);
    xEventGroupClearBits(g_sys_event_group, GY95_CALIBRATED_BIT);
    xEventGroupSetBits(g_sys_event_group, UART_BLOCK_BIT);

}

void app_main(void) {

    esp_ota_mark_app_valid_cancel_rollback();
    init();

    QueueHandle_t serial_queue = xQueueCreate(CONFIG_MSG_QUEUE_LEN, sizeof(imu_msg_raw_t));
    ESP_LOGD(TAG, "\nSerial Queue Addr %p\n", serial_queue);
    
    /** Launch time sync task **/
    ESP_LOGI(TAG, "launching time sync task");
    xTaskCreate(app_time_sync,
                "app_time_sync",
                2560,
                NULL,
                2,
                &time_sync_task);
    if (tcp_task == NULL) {
        ESP_LOGI(TAG, "Launching tcp client task");
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

#endif
    }
    if (uart_task == NULL) {
        ESP_LOGI(TAG, "Launching uart monitor task");
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
    
    if (controller_task == NULL) {
        ESP_LOGI(TAG, "Launching controller task");
#if CONFIG_MULTI_CORE
        xTaskCreatePinnedToCore(app_controller,
                                "app_controller",
                                4096,
                                NULL,
                                2,
                                &controller_task,
                                0x0);
#else
        xTaskCreate(app_controller,
                    "app_controller",
                    4096,
                    NULL,
                    2,
                    &controller_task);
#endif
    }

    // app_blink("\xF1\x01\xFF\x0F", 3);
    // while (1) {esp_task_wdt_feed();}
    ESP_LOGW(TAG, "Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
    while (1) { 
        vTaskDelay(10000 / portTICK_PERIOD_MS); 
        /** If WIFI_FAIL event occurs after init, we have a wifi interrupt. Going to deep sleep (shutdown)**/
        EventBits_t bits = xEventGroupWaitBits(g_wifi_event_group, WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
        if (bits & WIFI_FAIL_BIT) {
            ESP_LOGI(TAG, "Wi-Fi interrupt, going to deep sleep");
            esp_enter_deep_sleep();
        }
    }
}
