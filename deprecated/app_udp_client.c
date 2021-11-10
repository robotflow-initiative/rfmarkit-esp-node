#include "apps.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "settings.h"
#include "esp_log.h"
#include "types.h"

static const char* TAG = "app_udp_client";
static const char* PING_MSG = "I'm a teaport";

bool udp_ping_server(int sock, struct sockaddr_in* p_dest, int n_max_retry) {
    char rx_buffer[128];
    int err = sendto(sock, PING_MSG, strlen(PING_MSG), 0, (struct sockaddr*)p_dest, sizeof(*p_dest));
    if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        return false;
    }
    ESP_LOGI(TAG, "Message sent");

    struct sockaddr_storage source_addr;
    socklen_t socklen = sizeof(source_addr);

    for (int idx = 0; idx < n_max_retry; ++idx) {
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr*)&source_addr, &socklen);

        // Error occurred during receiving
        if (len < 0) {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
        }
        // Data received
        else {
            rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
            ESP_LOGI(TAG, "Received %d bytes from %s:", len, HOST_IP_ADDR);
            ESP_LOGI(TAG, "%s", rx_buffer);
            if (strncmp(rx_buffer, PING_MSG, strlen(PING_MSG)) == 0) {
                ESP_LOGI(TAG, "Received expected message");
                return true;
            } else {
                ESP_LOGI(TAG, "Received message: %s", rx_buffer);
                return false;
            }
        }
    }
    return false;
}

void app_udp_client(void* pvParameters) {
    int addr_family = 0;
    int ip_protocol = 0;

    QueueHandle_t serial_queue = (QueueHandle_t)pvParameters;
    struct timeval timeout = { 3,0 }; /** UDP timeout **/

    imu_msg_raw_t payload = { 0 };

    while (1) {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(HOST_PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, HOST_PORT);
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));


        while (1) {
            if (xQueueReceive(serial_queue, &payload, (TickType_t)0xF) != pdPASS) {
                continue;
            }
            int err = sendto(sock, payload.data, GY95_PAYLOAD_LEN, 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                // break;
            }
            // ESP_LOGI(TAG, "Message sent to %s:%s", HOST_IP_ADDR, HOST_PORT);

            // vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}