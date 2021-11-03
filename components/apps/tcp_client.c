#include "apps.h"
#include <string.h>
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_task_wdt.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "settings.h"
#include "types.h"
#include "globals.h"
#include "functions.h"

static const char* TAG = "app_tcp_client";
static uint8_t payload_buffer[CONFIG_PAYLOAD_BUFFER_LEN];

void app_tcp_client(void* pvParameters) {
    ESP_LOGI(TAG, "app_tcp_client started");
    int addr_family = 0;
    int ip_protocol = 0;
    int n_tcp_retry = CONFIG_ESP_TCP_MAXIMUM_RETRY;
    int n_light_sleep_retry = CONFIG_ESP_LIGHT_SLEEP_MAXIMUM_RETRY;

    QueueHandle_t serial_queue = (QueueHandle_t)pvParameters;
    struct timeval timeout = { 3,0 }; // TCP timeout 

    imu_msg_raw_t imu_reading = { 0 };
    int payload_len;


    while (1) {
        /** TCP connection is not established **/
        xEventGroupClearBits(g_sys_event_group, TCP_CONNECTED_BIT);

        /** Create address struct **/
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(CONFIG_HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(CONFIG_HOST_PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        /** Create socket **/
        int sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            goto socket_error;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", CONFIG_HOST_IP_ADDR, CONFIG_HOST_PORT);

        /** Set socket options **/
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));

        /** Wait time sync **/ // TODO: Verify this part
        ESP_LOGI(TAG, "Waiting for time sync");
        xEventGroupWaitBits(g_sys_event_group,
                            NTP_SYNCED_BIT,
                            pdFALSE,
                            pdFALSE,
                            portMAX_DELAY);

        /** Connect socket so the IP address of node is reported **/
        int err = connect(sock, (struct sockaddr*)&dest_addr, sizeof(struct sockaddr_in6));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            goto socket_error;
        }
        ESP_LOGSOCKET(TAG, "Successfully connected, setting TCP_CONNECTED_BIT");
        ESP_LOGI(TAG, "Successfully connected, setting TCP_CONNECTED_BIT");

        SET_DEBUG_SOCK(sock);

        xEventGroupSetBits(g_sys_event_group, TCP_CONNECTED_BIT);
        n_tcp_retry = CONFIG_ESP_TCP_MAXIMUM_RETRY;

        while (1) {
            ESP_LOGD(TAG, "tcp_client loop");
            ESP_LOGSOCKET(TAG, "tcp_client loop");
            /** If the QueueIsEmpy, sleep for a while **/
            if (xQueueReceive(serial_queue, &imu_reading, (TickType_t)0x4) != pdPASS) {
                taskYIELD();
                ESP_LOGD(TAG, "No item in queue");
                continue;
            }
            /** imu_reading is available **/
#if CONFIG_SEND_PARSED
            err = parse_imu_reading(&imu_reading, payload_buffer, CONFIG_PAYLOAD_BUFFER_LEN);
            payload_len = strlen(payload_buffer);

            if (payload_len == 0 || !err) {
                esp_task_wdt_reset();
                ESP_LOGW(TAG, "Failed to parse IMU reading");
                continue;
            } else {
                ESP_LOGD(TAG, "Got parsed imu reading: \n%s\n", payload_buffer);
                /** Append '\n' at the end of payload **/
                payload_buffer[payload_len++] = '\n';
            }
#else
            bzero(payload_buffer, sizeof(payload_buffer));
            payload_len = tag_imu_reading(&imu_reading, payload_buffer, sizeof(payload_buffer));
#endif

            err = send(sock, payload_buffer, payload_len, 0);
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                xEventGroupClearBits(g_sys_event_group, TCP_CONNECTED_BIT);
                break;
            }
            taskYIELD();

            ESP_LOGD(TAG, "Message sent to %s:%s", CONFIG_HOST_IP_ADDR, CONFIG_HOST_PORT);
        }

socket_error:

        xEventGroupClearBits(g_sys_event_group, TCP_CONNECTED_BIT);
        vTaskDelay(10); // TODO: Magic Delay
        if (sock != -1) {
            ESP_LOGE(TAG, "n_tcp_retry = %d, Shutting down socket...", n_tcp_retry);
            shutdown(sock, 0);
            vTaskDelay(10 / portTICK_PERIOD_MS);
            close(sock);
        }

        /** Sleep related **/
        ESP_LOGW(TAG, "0x%x", xEventGroupGetBits(g_sys_event_group));
        if ((xEventGroupGetBits(g_sys_event_group) & UART_BLOCK_BIT)){
            --n_tcp_retry;
            if (n_light_sleep_retry <= 0) {
                ESP_LOGI(TAG, "Entering deep sleep");
                esp_enter_deep_sleep();
            }
            if (n_tcp_retry <= 0) {
                esp_enter_light_sleep();
                ESP_LOGI(TAG, "Light sleep retry: %d", n_light_sleep_retry);
                --n_light_sleep_retry;
                n_tcp_retry += 2;
            }
        }

    }

    vTaskDelete(NULL);
}