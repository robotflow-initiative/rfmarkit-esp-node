#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_log.h"

#include "driver/uart.h"

#include "apps.h"
#include "imu.h"
#include "sys.h"

static const char* TAG = "app_uart_monitor";

/**
 * @brief Compute and set timestamp with us resolution
 *
 */
#define tag_time_us(time) \
        { \
                struct timeval tv_now = { 0 }; \
                gettimeofday(&tv_now, NULL); \
                (time) = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec; \
        }
#define tag_buffer_len(len) \
        { \
            size_t uart_buffer_len = 0; \
            uart_get_buffered_data_len(g_imu.port, &uart_buffer_len); \
            (len) = uart_buffer_len; \
        }


void app_uart_monitor(void* pvParameters) {
    ESP_LOGI(TAG, "app_uart_monitor started");

    QueueHandle_t serial_queue = (QueueHandle_t)pvParameters;
    ESP_LOGD(TAG, "\n[ Address ] %p\n", serial_queue);

    imu_dgram_t imu_data = { 0 };
    imu_dgram_t imu_data_trash = { 0 };
    int ret;

    /** Wait until tcp connection is established, time synced and uart not blocked**/
    while (1) {
        {
            EventBits_t bits;
            /** @warning xEventGroupWaitBits won't work because:
             * xEventGroupWaitBits will wait for all marked bits to be 1, but will ignore the status of unmarked bits
            **/
            while (1) {
                bits = xEventGroupGetBits(g_mcu.sys_event_group);
                ESP_LOGD(TAG, "Bits: %x", bits);

                if (!(bits & EV_TCP_CONNECTED_BIT) || !(bits & EV_NTP_SYNCED_BIT) || (bits & EV_UART_MANUAL_BLOCK_BIT)) {
                    os_delay_ms(200);
                    clear_sys_event(EV_UART_ACTIVATED); // Mark uart as inactive
                    continue;
                } else {
                    if (bits & EV_IMU_ENABLED_BIT) { // TODO: remove this judgement by always enable imu
                        break;
                    } else {
                        ESP_LOGI(TAG, "Enabling IMU");
                        imu_enable(&g_imu);
                        set_sys_event(EV_IMU_ENABLED);
                        break;
                    }
                }
            }
            if (!(bits & EV_UART_ACTIVATED_BIT)) {
                set_sys_event(EV_UART_ACTIVATED);
//                uart_flush(g_imu.port);
            }
        }

        /** Tag starting point */
        tag_time_us(imu_data.start_time_us);

        imu_read(&g_imu);
        memcpy(imu_data.data, g_imu.buf, g_imu.n_bytes);
        imu_data.n_bytes = g_imu.n_bytes;

        /** Tag stop point **/
        tag_time_us(imu_data.time_us);

        /** Count how many bits are left in the buffer **/
        tag_buffer_len(imu_data.uart_buffer_len);

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
    }
//    vTaskDelete(NULL);
}
