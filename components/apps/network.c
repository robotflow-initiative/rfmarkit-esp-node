
#include <sys/cdefs.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"

#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_task_wdt.h"

#include "lwip/err.h"

#include "apps.h"
#include "settings.h"
#include "sys.h"
#include "imu.h"
#include "udp.h"

#define CONFIG_UDP_RETRY 3

typedef struct {
    float acc[3];           /* acceleration          */
    float gyr[3];           /* angular velocity      */
    float mag[3];           /* magnetic field        */
    float quat[4];          /* attitude: quaternion  */
    uint32_t imu_ts_ms;
    int64_t dev_ts_us;
    int64_t tsf_ts_us;
    uint32_t seq;
    int32_t dev_delay_us;
} marker_packet_t;


static const char *TAG = "app.data_client ";

/**
 * @brief Compute checksum of the data
 * @param data
 * @param len
 * @return
 */
__attribute__((unused)) static uint8_t compute_checksum(const uint8_t *data, size_t len) {
    uint8_t sum = 0;
    for (int idx = 0; idx < len; ++idx) sum ^= data[idx];
    return sum;
}

/**
 * @brief Tag the packet with the imu data, to avoid memory alignment issue
 * @param[out] pkt
 * @param imu_data
**/
static void tag_packet(marker_packet_t *pkt, imu_dgram_t *imu_data) {
    memcpy(pkt->acc, imu_data->imu.acc, sizeof(pkt->acc));
    memcpy(pkt->gyr, imu_data->imu.gyr, sizeof(pkt->gyr));
    memcpy(pkt->mag, imu_data->imu.mag, sizeof(pkt->mag));
    memcpy(pkt->quat, imu_data->imu.quat, sizeof(pkt->quat));
    pkt->imu_ts_ms = imu_data->imu.imu_ts_ms;
    pkt->dev_ts_us = imu_data->dev_ts_us;
    pkt->tsf_ts_us = imu_data->tsf_ts_us;
    pkt->seq = imu_data->seq;
    struct timeval tv_now = { 0, 0 };
    gettimeofday(&tv_now, NULL);
    int64_t now_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    pkt->dev_delay_us =(int32_t)(now_us - imu_data->dev_ts_us);
}

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

_Noreturn void app_data_client(void *pvParameters) {
    ESP_LOGI(TAG, "app_data_client started");

    /** Get a ring buffer pointer **/
    ring_buf_t *serial_buf = &g_mcu.imu_ring_buf;

    /** Initialize a packet **/
    marker_packet_t pkt = {0};
    udp_socket_t client = {0};
    esp_err_t err;

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

    while (1) {
        /** Wait for the system to be active **/
        if (!g_mcu.state.active || !g_mcu.state.ntp_synced || !g_mcu.state.discovery_completed) {
            os_delay_ms(500);
            continue;
        }

        /** Initialize the client, create udp socket**/
        if (!client.initialized) {
            err = udp_socket_init(&client, 0, g_mcu.data_host_ip_addr, CONFIG_DATA_HOST_PORT);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "socket unable to create: errno %d", errno);
                goto handle_error;
            }
        }
        os_delay_ms(100); // wait for monitor to start

        int64_t curr_index = 1;
        int64_t confirm_index = -1;

        xQueueReset(read_signal_queue);
        esp_timer_start_periodic(read_timer, 1000000 / g_mcu.target_fps); // 2x the target fps

        imu_dgram_t imu_reading = {0};
        char signal = 0;
        while (g_mcu.state.active) {
            /** get the signal from the queue **/
            if (confirm_index >= serial_buf->head) {
                xQueueReceive(read_signal_queue, &signal, portMAX_DELAY);
            }

            /** If the queue is empty, sleep for a while **/
            err = ring_buf_peek(serial_buf, &imu_reading, curr_index, &confirm_index);

            /** The ring buffer is empty or not ready **/
            if (err != ESP_OK) {
                taskYIELD();
                continue;
            }
            curr_index++; // increment the index

            /** imu_reading is available **/
            tag_packet(&pkt, &imu_reading);

            /** According to the document, udp send may fail because of memory issue, this is normal **/
            for (int i = 0; i < CONFIG_UDP_RETRY; ++i) {
                err = udp_socket_send(&client, (uint8_t *) &pkt, sizeof(pkt));
                if (err == ESP_OK) {
                    break;
                } else if (err == ESP_ERR_NO_MEM) {
                    ESP_LOGW(TAG, "failed transmitting packets to %s:%d, retry", g_mcu.data_host_ip_addr, CONFIG_DATA_HOST_PORT);
                    os_delay_ms(100);
                } else {
                    ESP_LOGE(TAG, "failed transmitting packets to %s:%d", g_mcu.data_host_ip_addr, CONFIG_DATA_HOST_PORT);
                    break;
                }
                taskYIELD();
            }

            /** If the packet is not sent, abort **/
            if (err != ESP_OK) goto handle_error;
        }

handle_error:
        esp_timer_stop(read_timer);
        ESP_LOGE(TAG, "data_client aborted, setting operation_mode=inactive");
        if (g_mcu.state.active) sys_set_operation_mode(false);
        os_delay_ms(5000);
    }

    vTaskDelete(NULL);
}