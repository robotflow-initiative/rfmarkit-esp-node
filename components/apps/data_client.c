
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
    uint8_t addr;
    uint32_t id;            /* user defined ID       */
    float acc[3];           /* acceleration          */
    float gyr[3];           /* angular velocity      */
    float mag[3];           /* magnetic field        */
    float eul[3];           /* attitude: eular angle */
    float quat[4];          /* attitude: quaternion  */
    float pressure;         /* air pressure          */
    uint32_t timestamp;
    int64_t time_us;
    int64_t tsf_time_us;
    uint32_t seq;
    int32_t buffer_delay_us;
    char device_id[12];
    uint8_t checksum;
} marker_packet_t;

//static marker_packet_t dummy_packet = {
//    .addr = 100,
//    .id = 101,
//    .acc = {1, 1, 1},
//    .gyr = {2, 2, 2},
//    .mag = {3, 3, 3},
//    .eul = {4, 4, 4},
//    .quat = {5, 5, 5, 5},
//    .pressure = 6,
//    .timestamp = 7,
//    .time_us = 8,
//    .tsf_time_us = 9,
//    .seq = 10,
//    .uart_buffer_len = 11,
//    .device_id = "abcdabcdabcd",
//    .checksum = 12
//};

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
    pkt->id = imu_data->imu.id;
    memcpy(pkt->acc, imu_data->imu.acc, sizeof(pkt->acc));
    memcpy(pkt->gyr, imu_data->imu.gyr, sizeof(pkt->gyr));
    memcpy(pkt->mag, imu_data->imu.mag, sizeof(pkt->mag));
    memcpy(pkt->eul, imu_data->imu.eul, sizeof(pkt->eul));
    memcpy(pkt->quat, imu_data->imu.quat, sizeof(pkt->quat));
    pkt->pressure = imu_data->imu.pressure;
    pkt->timestamp = imu_data->imu.timestamp;
    pkt->time_us = imu_data->time_us;
    pkt->tsf_time_us = imu_data->tsf_time_us;
    pkt->seq = imu_data->seq;
    pkt->buffer_delay_us = imu_data->buffer_delay_us;
    pkt->checksum = 0;

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
    pkt.addr = g_imu.p_imu->addr; // this part is fixed
    memcpy(pkt.device_id, g_mcu.device_id, sizeof(g_mcu.device_id));  // this part is fixed

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
        esp_timer_start_periodic(read_timer, 1000000 / g_mcu.target_fps);

        imu_dgram_t imu_reading = {0};
        char signal = 0;
        while (g_mcu.state.active) {
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
            pkt.checksum = 0;

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

            /** get the signal from the queue **/
            xQueueReceive(read_signal_queue, &signal, portMAX_DELAY);
        }

handle_error:
        esp_timer_stop(read_timer);
        ESP_LOGE(TAG, "data_client aborted, setting operation_mode=inactive");
        if (g_mcu.state.active) sys_set_operation_mode(false);
        os_delay_ms(5000);
    }

    vTaskDelete(NULL);
}