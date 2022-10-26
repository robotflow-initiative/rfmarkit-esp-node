#include "freertos/FreeRTOS.h"

#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_task_wdt.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"

#include "tcp.h"

static const char* TAG = "tcp_server";

void server_set_port(tcp_server_t* server, int port) {
    server->dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server->dest_addr.sin_family = AF_INET;
    server->dest_addr.sin_port = htons(port);
}

esp_err_t server_create_socket(tcp_server_t* server) {
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    server->server_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (server->server_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        delay_ms(1000);
        handle_socket_err(server->server_sock);
    }
    return (server->server_sock >= 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t server_bind(tcp_server_t* server) {
    server_set_port(server, server->port);

    int err = bind(server->server_sock, (struct sockaddr*)&server->dest_addr, sizeof(server->dest_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        handle_socket_err(server->server_sock);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Socket bound, port %d", server->port);

    return ESP_OK;
}

esp_err_t server_set_timeout(tcp_server_t* server, int timeout_s) {
    int opt = 1;
    struct timeval timeout = { timeout_s, 0 };
    setsockopt(server->server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    setsockopt(server->server_sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
    return ESP_OK;
}

esp_err_t server_listen(tcp_server_t* server) {
    int err = listen(server->server_sock, server->max_listen);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        handle_socket_err(server->server_sock);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t client_sock_set_keepalive(int sock) {
    int KEEP_ALIVE = CONFIG_KEEP_ALIVE;
    int KEEP_IDLE = CONFIG_KEEP_IDLE;
    int KEEP_COUNT = CONFIG_KEEP_COUNT;
    int KEEP_INTERVAL = CONFIG_KEEP_INTERVAL;
    /** Set tcp keepalive option **/
    setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &KEEP_ALIVE, sizeof(int));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &KEEP_IDLE, sizeof(int));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &KEEP_INTERVAL, sizeof(int));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &KEEP_COUNT, sizeof(int));
    return ESP_OK;
}

int server_accept(tcp_server_t* server) {
    struct sockaddr_storage source_addr;
    socklen_t socklen = sizeof(source_addr);

    ESP_LOGD(TAG, "Socket listening");
    int client_sock = accept(server->server_sock, (struct sockaddr*)&source_addr, &socklen);
    if (client_sock < 0) {
        return client_sock;
    }
    log_source_addr(source_addr);
    /** Set tcp keepalive option **/
    client_sock_set_keepalive(client_sock);
    return client_sock;
}

#define CONFIG_PORT 2333
#define CONFIG_LISTEN_NUM 1

_Noreturn void server_loop(tcp_server_t* server, void (*interact)(int)) {

    while (1) {
        /** Create address struct **/

        /** Create STREAM socket **/
        server_create_socket(server);

        /** Bind socket **/
        server_bind(server);

        /** Start listenning **/
        server_listen(server);
        /** Set listen socket options **/

        server_set_timeout(server, 0);

        while (1) {
            int client_sock = server_accept(server);
            if (client_sock < 0) {
                ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
                break;
            }

            interact(client_sock);

            shutdown(client_sock, 0);
            close(client_sock);
        }
        handle_socket_err(server->server_sock);
    }
}