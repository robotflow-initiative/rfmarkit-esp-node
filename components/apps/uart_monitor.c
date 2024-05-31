#include <sys/cdefs.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <esp_mesh.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_wifi.h"

#include "driver/uart.h"

#include "apps.h"
#include "imu.h"
#include "sys.h"

static const char *TAG = "app.uart_monitor";

/**
 * @brief Compute and set timestamp with us resolution
 *
**/
static void IRAM_ATTR tag_time_us(imu_dgram_t *imu_data) {
    get_time_usec(imu_data->time_us);
    imu_data->tsf_time_us = esp_mesh_get_tsf_time();
}

/**
 * @brief Tag the buffer length
 * @param imu_data
**/
static void IRAM_ATTR tag_buffer_len(imu_dgram_t *imu_data) {
    size_t uart_buffer_len = 0;
    uart_get_buffered_data_len(g_imu.port, &uart_buffer_len);
    imu_data->uart_buffer_len = (int32_t) uart_buffer_len;
}

_Noreturn void app_uart_monitor(void *pvParameters) {
    ESP_LOGI(TAG, "app_uart_monitor started");

    /** Get a ring buffer pointer **/
    ring_buf_t *serial_buf = &g_mcu.imu_ring_buf;

    /** Create a imu data structure **/
    imu_dgram_t imu_data = {0};

    while (1) {
        /** Wait for the system to be active **/
        if (!g_imu.enabled || g_imu.mux != IMU_MUX_STREAM) {
            os_delay_ms(500);
            continue;
        }

        /** Flush the uart buffer and prepare ring_buffer **/
        uart_flush(g_imu.port);
        uint32_t seq = 0;
        ring_buf_reset(serial_buf);

#if CONFIG_FPS_ENABLED
        int64_t start_time;
        get_time_usec(start_time);
        uint32_t old_seq = 0;
#endif
        while (g_imu.enabled && g_imu.mux == IMU_MUX_STREAM) {
            /** Read IMU data **/
            esp_err_t err = imu_read(&g_imu, &imu_data, true);

            /** If the err occurs(most likely due to the empty uart buffer), wait **/
            if (err != ESP_OK) {
                ESP_LOGD(TAG, "IMU read error: %d", err);
                taskYIELD();
                os_delay_ms(5);
                continue;
            }

            /** Tag seq number, timestamp, buffer_len(how many bits are left in the buffer) **/
            imu_data.seq = seq++;
            tag_time_us(&imu_data);
            tag_buffer_len(&imu_data);

            /** Add the imu data to the ring buffer **/
            ring_buf_push(serial_buf, (uint8_t *) &imu_data);
#if CONFIG_FPS_ENABLED
            int64_t now;
            get_time_usec(now);
            if ( now - start_time > 1000000UL) {
                size_t buf_len;
                uart_get_buffered_data_len(g_imu.port, &buf_len);
                ESP_LOGI(TAG, "fps=%d, buf_len=%d", seq - old_seq, buf_len);
                start_time = now;
                old_seq = seq;
            }
#endif
        }
    }
}
