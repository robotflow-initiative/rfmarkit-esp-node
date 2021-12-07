#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_task_wdt.h"

#include "driver/uart.h"

#include "apps.h"
#include "settings.h"
#include "imu.h"
#include "device.h"

/** pseudo data **/
#if CONFIG_USE_PSEUDO_VALUE
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
#endif

static const char* TAG = "app_uart_monitor";

void app_uart_monitor(void* pvParameters) {
    ESP_LOGI(TAG, "app_uart_monitor started");

    QueueHandle_t serial_queue = (QueueHandle_t)pvParameters;
    ESP_LOGD(TAG, "\n[ Address ] %p\n", serial_queue);

    struct timeval tv_now = { 0 };

    imu_dgram_t imu_data = { 0 };
    int ret = 0;

    /** Wait until tcp connection is established, time synced and uart not blocked**/
    while (1) {
        EventBits_t bits;

        /** @warning xEventGroupWaitBits won't work because:
         * xEventGroupWaitBits will wait for all marked bits to be 1, but will ignore the status of unmarked bits 
        **/
        
        while (1) {
            bits = xEventGroupGetBits(g_mcu.sys_event_group);
            ESP_LOGD(TAG, "Bits: %x", bits);

            if (!(bits & TCP_CONNECTED_BIT) || !(bits & NTP_SYNCED_BIT) || (bits & UART_BLOCK_BIT)) {
                device_delay_ms(1000);
                xEventGroupClearBits(g_mcu.sys_event_group, UART_ACTIVE_BIT); // Mark uart as inactive
                continue;
            } else {
                if (bits & IMU_ENABLED_BIT) {
                    break;
                } else {
                    ESP_LOGI(TAG, "Enabling IMU");
                    imu_enable(&g_imu);
                    xEventGroupSetBits(g_mcu.sys_event_group, IMU_ENABLED_BIT);
                    break;
                }
            }
        }
        if (!(bits & UART_ACTIVE_BIT)) {
            xEventGroupSetBits(g_mcu.sys_event_group, UART_ACTIVE_BIT);
        }
        /** Get data **/
#if CONFIG_USE_PSEUDO_VALUE
        bzero(imu_data.data, sizeof(imu_data.data));
        memcpy(imu_data.data, pseudo_data1, sizeof(imu_data.data));
        gettimeofday(&tv_now, NULL);
        imu_data.timew_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
        vTaskDelay(10 / portTICK_PERIOD_MS);
#else
        ESP_LOGD(TAG, "Try to read gy");

        /** Tag starting point */
        gettimeofday(&tv_now, NULL);
        imu_data.start_time_us =  (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;

        imu_read(&g_imu);
        memcpy(imu_data.data, g_imu.buf, sizeof(g_imu.buf));

        /** Tag stop point **/
        gettimeofday(&tv_now, NULL);
        /** Count how many bits are left in the buffer **/
        size_t uart_buffer_len = 0;
        uart_get_buffered_data_len(g_imu.port, &uart_buffer_len);
        imu_data.time_us = ((int64_t)tv_now.tv_sec - ((int64_t)uart_buffer_len / (CONFIG_IMU_DEFAULT_FREQ * CONFIG_IMU_PAYLOAD_LEN))) * 1000000L + (int64_t)tv_now.tv_usec;
        
        // imu_data.time_us = ((int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec);

        imu_data.uart_buffer_len = uart_buffer_len;

#endif
        /** If queue is full, clear queue **/
        while (uxQueueSpacesAvailable(serial_queue) <= 0) {
            xQueueReceive(serial_queue, (void*)&imu_data, (TickType_t)0xF);
            ESP_LOGE(TAG, "Buffer full\n");
        }

        ret = xQueueSend(serial_queue, (void*)&imu_data, (TickType_t)0xF);

        if (ret != pdPASS) {
            ESP_LOGE(TAG, "Cannot enqueue message\n");
        } else {
            ESP_LOGD(TAG, "Enqueue message\n");
        }
    }
    vTaskDelete(NULL);
}