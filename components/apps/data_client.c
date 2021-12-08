#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_task_wdt.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"


#include "apps.h"
#include "settings.h"
#include "device.h"
#include "imu.h"
#include "tcp.h"

static const char* TAG = "app_data_client";

static uint8_t s_send_buffer[CONFIG_PAYLOAD_BUFFER_LEN * 0xF];
static int s_send_buffer_tail = 0;
#define SEND_BUFFER_FULL() (s_send_buffer_tail > CONFIG_PAYLOAD_BUFFER_LEN * (10 - 2))
#define RESET_SEND_BUFFER() \
    s_send_buffer_tail = 0;

void app_data_client(void* pvParameters) {
    ESP_LOGI(TAG, "app_data_client started");

    QueueHandle_t serial_queue = (QueueHandle_t)pvParameters; 

    imu_dgram_t imu_reading = { 0 };
    tcp_client_t client = {
        .port = CONFIG_HOST_PORT,
        .address = CONFIG_HOST_IP_ADDR
    };

    while (1) {
        /** Wait time sync **/
        ESP_LOGI(TAG, "Waiting for time sync");
        xEventGroupWaitBits(g_mcu.sys_event_group,
                            NTP_SYNCED_BIT,
                            pdFALSE,
                            pdFALSE,
                            portMAX_DELAY);

        /** TCP connection is not established **/
        clear_sys_event(TCP_CONNECTED);

        client_init(&client);

        /** Connect socket so the IP address of node is reported **/
        int err = client_connect(&client);
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            device_delay_ms(1000);
            goto socket_error;
        }
        ESP_LOGI(TAG, "Successfully connected, setting TCP_CONNECTED_BIT");

        set_sys_event(TCP_CONNECTED);
        /** TCP connection is established **/

        while (1) {
            ESP_LOGD(TAG, "tcp_client loop");
            /** If the QueueIsEmpy, sleep for a while **/
            if (xQueueReceive(serial_queue, &imu_reading, (TickType_t)0x4) != pdPASS) {
                taskYIELD();
                ESP_LOGD(TAG, "No item in queue");
                continue;
            }
            /** imu_reading is available **/
            s_send_buffer_tail += imu_tag(&imu_reading,
                                          s_send_buffer + s_send_buffer_tail,
                                          sizeof(s_send_buffer) - s_send_buffer_tail);
            if (SEND_BUFFER_FULL()) {
                err =  send(client.client_sock, s_send_buffer, s_send_buffer_tail, 0);
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    clear_sys_event(TCP_CONNECTED);
                    break;
                } else {
                    device_reset_sleep_countup();
                }
                ESP_LOGD(TAG, "Reading sent to %s:%s", CONFIG_HOST_IP_ADDR, CONFIG_HOST_PORT);
                RESET_SEND_BUFFER();
            } else {
                taskYIELD();
            }
        }

socket_error:
        clear_sys_event(TCP_CONNECTED);
        ESP_LOGE(TAG, " Shutting down socket... for %d", errno);
        switch (errno) {
        case ECONNRESET:
            device_delay_ms(5000);
            break;
        default:
            device_delay_ms(100);
            break;
        };
        handle_socket_err(client.client_sock)
    }

    vTaskDelete(NULL);
}