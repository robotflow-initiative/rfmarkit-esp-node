#include <sys/cdefs.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <esp_mesh.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_log.h"

#include "apps.h"
#include "imu.h"
#include "sys.h"

static const char *TAG = "app.monitor     ";

#if CONFIG_EN_READ_FPS_LIM

/**
 * @brief Timer callback handler， send a signal to the queue so that the data client read latest reading
 * @param arg
 */
static void IRAM_ATTR read_timer_cb_handler(void *arg) {
    ESP_LOGD(TAG, "timer_cb_handler called");
    const char signal = 0;
    xQueueSendFromISR((QueueHandle_t) arg, &signal, NULL);
}

static QueueHandle_t read_signal_queue = NULL;
#endif

_Noreturn void app_monitor(void *pvParameters) {
    ESP_LOGI(TAG, "app_monitor started");

    /** Get a ring buffer pointer **/
    ring_buf_t *serial_buf = &g_mcu.imu_ring_buf;

    /** Create a imu data structure **/
    imu_dgram_t imu_data = {0};

#if CONFIG_EN_READ_FPS_LIM
    /** Limit the read FPS **/
    if (read_signal_queue == NULL) read_signal_queue = xQueueCreate(128, sizeof(char));
    esp_timer_handle_t read_timer;
    const esp_timer_create_args_t blink_timer_args = {
        .callback = &read_timer_cb_handler,
        /** name is optional, but may help identify the timer when debugging */
        .name = "timeout",
        .skip_unhandled_events = true,
        .arg = read_signal_queue
    };
    ESP_ERROR_CHECK(esp_timer_create(&blink_timer_args, &read_timer));
#endif

    while (1) {
        /** Wait for the system to be active **/
        if (!g_imu.p_imu->enabled || g_imu.p_imu->mux != IMU_MUX_STREAM) {
            os_delay_ms(500);
            continue;
        }

        /** Flush the uart buffer and prepare ring_buffer **/
        g_imu.buffer_reset(g_imu.p_imu);
        uint32_t seq = 0;
        ring_buf_reset(serial_buf);

#if CONFIG_EN_FPS_PROFILING
        int64_t start_time;
        get_time_usec(start_time);
        uint32_t old_seq = 0;
#endif
#if CONFIG_EN_READ_FPS_LIM
        char signal = 0;
        xQueueReset(read_signal_queue);
        esp_timer_start_periodic(read_timer, 1000000 / g_mcu.target_fps);
#endif
        while (g_imu.p_imu->enabled && g_imu.p_imu->mux == IMU_MUX_STREAM) {
#if CONFIG_EN_READ_FPS_LIM
            /** get the signal from the queue **/
            xQueueReceive(read_signal_queue, &signal, portMAX_DELAY);
#endif
            /** Read IMU data **/
            esp_err_t err = g_imu.read(g_imu.p_imu, &imu_data, true);
            /** If the err occurs(most likely due to the empty uart buffer), wait **/
            if (err != ESP_OK) {
                ESP_LOGD(TAG, "IMU read error: %d", err);
                taskYIELD();
                continue;
            }
            /** Tag seq number, timestamp, buffer_len(how many bits are left in the buffer) **/
            imu_data.seq = seq++;

            /** Add the imu data to the ring buffer **/
            ring_buf_push(serial_buf, (uint8_t *) &imu_data);

#if CONFIG_EN_FPS_PROFILING
            int64_t now;
            get_time_usec(now);
            if (now - start_time > 1000000UL) {
                int64_t delay = g_imu.get_delay_us(g_imu.p_imu);
                ESP_LOGI(TAG, "fps=%d, delay=%lld", seq - old_seq, delay);
                start_time = now;
                old_seq = seq;
            }
#endif
            taskYIELD();
        }
    }
}
