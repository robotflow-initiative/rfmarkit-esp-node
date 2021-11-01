#include "apps.h"

#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "driver/uart.h"
#include "freertos/queue.h"

#include "settings.h"
#include "types.h"
#include "gy95.h"
#include "events.h"
#include "main.h"


/** pseudo data **/
char pseudo_data1[40] = { 0xA4, 0x03, 0x08, 0x12, 0x00, 0x07,
                          0xFF, 0xFE, 0x08, 0x0A, 0xFF, 0xFE,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x24, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                          0x00, 0x04 };

char pseudo_data2[40] = { 0xA4, 0x03, 0x08, 0x27, 0xFC, 0xFF,
                          0x02, 0x00, 0x03, 0xF0, 0x01, 0x00,
                          0xFF, 0xFF, 0x00, 0x00, 0x04, 0x00,
                          0xFB, 0xFF, 0x18, 0x17, 0xED, 0xEC,
                          0x0C, 0xe0, 0x04 ,0x03, 0x02, 0x0F,
                          0xFB, 0xBE };
 
static const char* TAG =  "app_uart_monitor";

void app_uart_monitor(void* pvParameters) {
    ESP_LOGI(TAG, "app_uart_monitor started");

    QueueHandle_t serial_queue = (QueueHandle_t)pvParameters;
    ESP_LOGD(TAG, "\n[ Address ] %p\n", serial_queue);

    gy95_t gy;
    gy95_init(&gy, GY95_PORT, GY95_CTRL_PIN,  GY95_ADDR);
    struct timeval tv_now = { 0 };

    imu_msg_raw_t imu_data = { 0 };
    imu_msg_raw_t imu_data_trash = { 0 };
    int ret = 0;
    // int cnt = 0;
    // TickType_t last_update = 0;

    while (1) {
        EventBits_t bits;
        /** Wait until tcp connection is established **/

        // TODO: figure out why xEventGroupWaitBits won't work
        // last_update = xTaskGetTickCount();
        // xEventGroupWaitBits(net_event_group,
        //                     TCP_CONNECTED_BIT | NTP_SYNCED_BIT,
        //                     pdFALSE,
        //                     pdTRUE,
        //                     portMAX_DELAY);
        while (1) {
            bits = xEventGroupGetBits(g_sys_event_group);
            ESP_LOGD(TAG, "Bits: %x", bits);
            if (!(bits & TCP_CONNECTED_BIT) || !(bits & NTP_SYNCED_BIT) || (bits & UART_BLOCK_BIT)) {
                vTaskDelay(1000 / portTICK_PERIOD_MS); // TODO: Magic Delay
                continue;
            } else {
                if (bits & GY95_CALIBRATED_BIT) {
                    break;
                } else {
                    ESP_LOGI(TAG, "Enabling gy95");
                    gy95_enable(&g_gy95_imu);
                    vTaskDelay(1);
                    ESP_LOGI(TAG, "Setting up gy95");
                    gy95_setup(&gy);
                    xEventGroupSetBits(g_sys_event_group, GY95_CALIBRATED_BIT);
                    ESP_LOGI(TAG, "Setup finished");
                    break;
                }   
            }
        }

        /** Get data **/
#if USE_PSEUDO_VALUE
        bzero(imu_data.data, GY95_MSG_LEN);
        memcpy(imu_data.data, pseudo_data1, GY95_MSG_LEN);
        gettimeofday(&tv_now, NULL);
        imu_data.time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
        vTaskDelay(10 / portTICK_PERIOD_MS);
#else
        // ESP_LOGI(TAG, "Try to read gy");
        gy95_read(&gy);
        memcpy(imu_data.data, gy.buf, GY95_MSG_LEN);
        gettimeofday(&tv_now, NULL);
        imu_data.time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
#endif

        /** If queue is full, clear queue **/
        while (uxQueueSpacesAvailable(serial_queue) <= 0) {
            xQueueReceive(serial_queue, (void*)&imu_data_trash, (TickType_t)0xF);
            ESP_LOGE(TAG, "Buffer full\n");
        }

        ret = xQueueSend(serial_queue, (void*)&imu_data, (TickType_t)0xF);

        if (ret != pdPASS) {
            ESP_LOGE(TAG, "Cannot enqueue message\n");
        } else {
            ESP_LOGD(TAG, "Enqueue message\n");
        }
        // esp_task_wdt_reset();
        /** Somehow vTaskDelayUntil will cause hard fault **/
        // vTaskDelayUntil(last_update, GY95_N_TICK);
        // vTaskDelay(100);
    }
    vTaskDelete(NULL);
}