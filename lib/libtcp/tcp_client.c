#include "freertos/FreeRTOS.h"

#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_task_wdt.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"

#include "tcp.h"

const char* TAG = "tcp_client";

void client_set_destination(tcp_client_t* client, const char* ip_addr, uint16_t port) {
    client->dest_addr.sin_addr.s_addr = inet_addr(ip_addr);
    client->dest_addr.sin_family = AF_INET;
    client->dest_addr.sin_port = htons(port);
}

esp_err_t client_create_socket(tcp_client_t* client) {
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;

    client->client_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (client->client_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        delay_ms(1000);
        handle_socket_err(client->client_sock);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Socket created, sending to %s:%d", client->address, client->port);
    return ESP_OK;
}

esp_err_t client_set_timeout(tcp_client_t* client, int timeout_s) {
    struct timeval timeout = { timeout_s, 0 };
    setsockopt(client->client_sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
    return ESP_OK;
}

void client_init(tcp_client_t* client, int timeout) {
    client_set_destination(client, client->address, client->port);
    client_create_socket(client);
    if (timeout > 0) {
        client_set_timeout(client, timeout);
    }
}

int client_connect(tcp_client_t* client) {
    int err = connect(client->client_sock, (struct sockaddr*)&client->dest_addr, sizeof(struct sockaddr_in6));
    return err;
}
