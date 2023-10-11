#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_wifi.h"

#include "driver/uart.h"

#include "apps.h"
#include "imu.h"
#include "sys.h"

static const char *TAG = "app_uart_monitor";

/**
 * @brief Compute and set timestamp with us resolution
 *
 */
#define tag_time_us(imu_data) \
        { \
                struct timeval tv_now = { 0 }; \
                gettimeofday(&tv_now, NULL); \
                (imu_data).time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec; \
                (imu_data).tsf_time_us = esp_wifi_get_tsf_time(WIFI_IF_STA); \
        }
#define tag_buffer_len(imu_data) \
        { \
            size_t uart_buffer_len = 0; \
            uart_get_buffered_data_len(g_imu.port, &uart_buffer_len); \
            (imu_data).uart_buffer_len = uart_buffer_len; \
        }


void app_uart_monitor(void *pvParameters) {
    ESP_LOGI(TAG, "app_uart_monitor started");

    QueueHandle_t serial_queue = (QueueHandle_t) pvParameters;
    ESP_LOGD(TAG, "\n[ Address ] %p\n", serial_queue);

    imu_dgram_t imu_data = {0};
    imu_dgram_t imu_data_trash = {0};
    int ret;
    EventBits_t bits;
    int32_t seq = 0;

    /** Wait until tcp connection is established, time synced and uart not blocked**/
    while (1) {
        {
            /** @warning xEventGroupWaitBits won't work because:
             * xEventGroupWaitBits will wait for all marked bits to be 1, but will ignore the status of unmarked bits
            **/
            bits = xEventGroupGetBits(g_mcu.sys_event_group);
            ESP_LOGD(TAG, "Bits: %x", bits);

            if (!(bits & EV_TCP_CONNECTED_BIT) || !(bits & EV_NTP_SYNCED_BIT) || (bits & EV_UART_MANUAL_BLOCK_BIT)) {
                os_delay_ms(200);
                clear_sys_event(EV_UART_ACTIVATED); // Mark uart as inactive
                xQueueReset(serial_queue);
                seq = 0;
                continue;
            }

            if (!(bits & EV_IMU_ENABLED_BIT)) { // TODO: remove this judgement by always enable imu
                ESP_LOGI(TAG, "Enabling IMU");
                imu_enable(&g_imu);
                set_sys_event(EV_IMU_ENABLED);
            }

            if (!(bits & EV_UART_ACTIVATED_BIT)) {
                set_sys_event(EV_UART_ACTIVATED);
//                uart_flush(g_imu.port);
            }
        }


        imu_read(&g_imu);
        memcpy(imu_data.imu, g_imu.raw.imu, sizeof(ch_imu_data_t));

        /** Tag seq number **/
        imu_data.seq = seq++;

        /** Tag stop point **/
        tag_time_us(imu_data);

        /** Count how many bits are left in the buffer **/
        tag_buffer_len(imu_data);

        /** If queue is full, wait **/
        if (uxQueueSpacesAvailable(serial_queue) <= 0) {
            ESP_LOGE(TAG, "Buffer full\n");
            taskYIELD();
            continue;
        }

        ret = xQueueSend(serial_queue, (void *) &imu_data, (TickType_t) 0xF);

        if (ret != pdPASS) {
            ESP_LOGE(TAG, "Cannot enqueue message\n");
        } else {
            ESP_LOGD(TAG, "Enqueue message\n");
        }
    }
//    vTaskDelete(NULL);
}
