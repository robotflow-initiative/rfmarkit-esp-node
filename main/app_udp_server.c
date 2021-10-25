#include "apps.h"
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
#include "events.h"
#include "funcs.h"

static const char* TAG = "app_udp_server";

typedef enum server_command_t {
    RESTART,
    SLEEP,
    RESET,
    CALIBRATE_ACCGYRO,
    CALIBRATE_MAG,

} server_command_t;

server_command_t parse_command(char * command, int len) {
    ESP_LOGI(TAG, "Got command %s from controller", command);
    return RESTART;
}

bool execute_command(server_command_t cmd, char * res, int len) {
    return true;
};

void app_udp_server(void* pvParameters) {
    ESP_LOGI(TAG, "app_udp_server started");

    char rx_buffer[128];
    char tx_buffer[128] = "Hello, world!";

    char addr_str[128];
    int addr_family = 0;
    int ip_protocol = 0;

    struct timeval timeout = { 3,0 }; // UDP timeout

    while (1) {

        /** Create address struct **/
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(CONFIG_LOCAL_PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        /** Create DGRAM socket **/
        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            goto socket_error;
        }
        int err = bind(sock, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
            goto socket_error;
        }

        ESP_LOGI(TAG, "Socket bound, port %d", CONFIG_LOCAL_PORT);

        /** Set socket options **/
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));

        while (1) {

            ESP_LOGI(TAG, "Waiting for data");
            struct sockaddr_storage source_addr;
            socklen_t socklen = sizeof(source_addr);

            /** Receiving **/
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr*)&source_addr, &socklen);
            if (len < 0) {
                ESP_LOGE(TAG, "Recvfrom failed: errno %d", errno);
                break;
            }

            /** Data received **/
            else {
                /** Get the sender's ip address as string **/
                inet_ntoa_r(((struct sockaddr_in*)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        
                /** Process with command from controller **/
                rx_buffer[len] = 0; // Null-terminate
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);

                /** Reply the controller **/
                int err = sendto(sock, tx_buffer, strlen(tx_buffer), 0, (struct sockaddr*)&source_addr, sizeof(source_addr));
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    goto socket_error;
                }
            }
        }

socket_error:
        vTaskDelay(10); // TODO: Magic Delay
        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            vTaskDelay(10 / portTICK_PERIOD_MS); // TODO: Magic Delay
            close(sock);
        }
    }

    vTaskDelete(NULL);
}