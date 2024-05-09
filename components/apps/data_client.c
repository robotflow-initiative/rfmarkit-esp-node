
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
    hi229_dgram_t dgram;
    char device_id[12];
    uint8_t checksum;
} marker_packet_t;

static const char *TAG = "app_data_client";

/**
 * @brief Compute checksum of the data
 * @param data
 * @param len
 * @return
 */
static uint8_t compute_checksum(const uint8_t *data, size_t len) {
    uint8_t sum = 0;
    for (int idx = 0; idx < len; ++idx) sum ^= data[idx];
    return sum;
}

_Noreturn void app_data_client(void *pvParameters) {
    ESP_LOGI(TAG, "app_data_client started");

    /** Get a ring buffer pointer **/
    ring_buf_t *serial_buf = &g_mcu.imu_ring_buf;

    /** Initialize a packet **/
    marker_packet_t pkt = {0};
    pkt.addr = g_imu.addr; // this part is fixed
    memcpy(pkt.device_id, g_mcu.device_id, sizeof(g_mcu.device_id));  // this part is fixed

    udp_socket_t client = {0};
    esp_err_t err;

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
        os_delay_ms(500); // wait for uart_monitor to start

        int64_t curr_index = 1;
        int64_t confirm_index = -1;
        while (g_mcu.state.active) {
            /** If the queue is empty, sleep for a while **/
            err = ring_buf_peek(serial_buf, &pkt.dgram, curr_index, &confirm_index);

            /** The ring buffer is empty or not ready **/
            if (err != ESP_OK) {
                taskYIELD();
                continue;
            }
            curr_index++; // increment the index

            /** imu_reading is available **/
            pkt.checksum = compute_checksum((uint8_t *) &pkt, sizeof(pkt) - sizeof(pkt.checksum));

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
        ESP_LOGE(TAG, "data_client aborted, setting operation_mode=inactive");
        if (g_mcu.state.active) sys_set_operation_mode(false);
        os_delay_ms(5000);
    }

    vTaskDelete(NULL);
}