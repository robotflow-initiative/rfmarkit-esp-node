#include "apps.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_task_wdt.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "settings.h"
#include "types.h"
#include "functions.h"

#define RX_BUFFER_LEN 64 
#define TX_BUFFER_LEN 512

static const char* TAG = "app_controller";

static char s_addr_str[128];
static char s_rx_buffer[RX_BUFFER_LEN];
static char s_tx_buffer[TX_BUFFER_LEN];

typedef enum server_command_t {
    IMU_RESTART = 0,
    IMU_PING = 1,
    IMU_SLEEP = 2,
    IMU_SHUTDOWN = 3,
    IMU_UPDATE = 4,
    IMU_CALI_RESET = 5,
    IMU_CALI_ACC = 6,
    IMU_CALI_MAG = 7,
    IMU_START = 8,
    IMU_STOP = 9,
    IMU_GY_ENABLE = 10,
    IMU_GY_DISABLE = 11,
    IMU_GY_STATUS = 12,
    IMU_GY_IMM = 13,
    IMU_ID = 14,
    IMU_VER = 15,
    IMU_ERROR,
} server_command_t;


esp_err_t(*command_funcs[])(char*, int) = {
    command_func_restart,
    command_func_ping,
    command_func_sleep,
    command_func_shutdown,
    command_func_update,
    command_func_cali_reset,
    command_func_cali_acc,
    command_func_cali_mag,
    command_func_start,
    command_func_stop,
    command_func_gy_enable,
    command_func_gy_disable,
    command_func_gy_status,
    command_func_gy_imm,
    command_func_id,
    command_func_ver
};

#define MATCH_CMD(x, cmd) (strncasecmp(x, cmd, strlen(cmd)) == 0)

server_command_t parse_command(char* command, int len) {
    ESP_LOGI(TAG, "Got command %s from controller", command);
    if (MATCH_CMD(command, "restart")) {
        return IMU_RESTART;
    } else if (MATCH_CMD(command, "ping")) {
        return IMU_PING;
    } else if (MATCH_CMD(command, "sleep")) {
        return IMU_SLEEP;
    } else if (MATCH_CMD(command, "shutdown")) {
        return IMU_SHUTDOWN;
    } else if (MATCH_CMD(command, "update")) {
        return IMU_UPDATE;
    } else if (MATCH_CMD(command, "cali_reset")) {
        return IMU_CALI_RESET;
    } else if (MATCH_CMD(command, "cali_acc")) {
        return IMU_CALI_ACC;
    } else if (MATCH_CMD(command, "cali_mag")) {
        return IMU_CALI_MAG;
    } else if (MATCH_CMD(command, "start")) {
        return IMU_START;
    } else if (MATCH_CMD(command, "stop") | MATCH_CMD(command, "pause")) {
        return IMU_STOP;
    } else if (MATCH_CMD(command, "gy_enable")) {
        return IMU_GY_ENABLE;
    } else if (MATCH_CMD(command, "gy_disable")) {
        return IMU_GY_DISABLE;
    } else if (MATCH_CMD(command, "gy_status")) {
        return IMU_GY_STATUS;
    } else if (MATCH_CMD(command, "gy_imm")) {
        return IMU_GY_IMM;
    } else if (MATCH_CMD(command, "id")) {
        return IMU_ID;
    } else if (MATCH_CMD(command, "ver")) {
        return IMU_VER;
    } else {
        return IMU_ERROR;
    }
}

esp_err_t execute_command(char* rx_buffer, char* tx_buffer, size_t rx_len, size_t tx_len) {
    server_command_t cmd = parse_command(rx_buffer, rx_len);
    esp_err_t ret = ESP_FAIL;
    if (cmd == IMU_ERROR) {
        /** Output error **/
        ESP_LOGE(TAG, "Got invalid command : %s", rx_buffer);

        /** Fill tx_buffer with 'ERROR\n' **/
        bzero(tx_buffer, sizeof(char) * tx_len);
        strcpy(tx_buffer, "ERROR\n\n");

        /** Return False **/
        return ESP_FAIL;
    } else {
        /** Output info **/

        /** Fill tx_buffer with '\0' **/
        bzero(tx_buffer, sizeof(char) * tx_len);
        /** Fill tx buffer with command related context **/
        ret = command_funcs[cmd](tx_buffer, tx_len);

        /** Command did not modify the buffer **/
        if (strlen(tx_buffer) == 0) {
            snprintf(tx_buffer, tx_len, "%s", (ret == ESP_OK) ? "OK\n\n" : "ERROR\n\n");
        }

        /** Return False **/
        return ret;
    }
    return true;
};

static void interact(const int sock)
{
    int len;
    ESP_LOGI(TAG, "Interacting");
    do {
        len = recv(sock, s_rx_buffer, sizeof(s_rx_buffer) - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        } else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed");
        } else {
            s_rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
            ESP_LOGI(TAG, "Received %d bytes: %s", len, s_rx_buffer);

            execute_command(s_rx_buffer, s_tx_buffer, RX_BUFFER_LEN, TX_BUFFER_LEN);
            int to_write = strlen(s_tx_buffer);
            while (to_write > 0) {
                int written = send(sock, s_tx_buffer, to_write, 0);
                if (written < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
                to_write -= written;
            }
        }
    } while (len > 0);
}

void app_controller(void* pvParameters) {
    ESP_LOGI(TAG, "app_controller started");

    // char s_addr_str[128];
    int addr_family = 0;
    int ip_protocol = 0;

    int KEEP_ALIVE = 1;
    int KEEP_IDLE = 5;
    int KEEP_COUNT = 3;
    int KEEP_INTERVAL = 5;

    struct timeval timeout = { 0, 0 }; // UDP timeout

    while (1) {

        /** Create address struct **/
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(CONFIG_LOCAL_PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        /** Create STREAM socket **/
        int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (listen_sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            goto socket_error;
        }

        /** Bind socket **/
        int err = bind(listen_sock, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
            goto socket_error;
        }
        ESP_LOGI(TAG, "Socket bound, port %d", CONFIG_LOCAL_PORT);

        /** Start listenning **/
        err = listen(listen_sock, 1);
        if (err != 0) {
            ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
            goto socket_error;
        }

        /** Set socket options **/
        int opt = 1;
        setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        setsockopt(listen_sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));

        struct sockaddr_storage source_addr;
        socklen_t socklen = sizeof(source_addr);

        while (1) {
            ESP_LOGI(TAG, "Socket listening");

            int sock = accept(listen_sock, (struct sockaddr*)&source_addr, &socklen);
            if (sock < 0) {
                ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
                break;
            }

            /** Set tcp keepalive option **/
            setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &KEEP_ALIVE, sizeof(int));
            setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &KEEP_IDLE, sizeof(int));
            setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &KEEP_INTERVAL, sizeof(int));
            setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &KEEP_COUNT, sizeof(int));

            /** Convert ip address to string **/
            if (source_addr.ss_family == PF_INET) {
                inet_ntoa_r(((struct sockaddr_in*)&source_addr)->sin_addr, s_addr_str, sizeof(s_addr_str) - 1);
            }
            ESP_LOGI(TAG, "Socket accepted ip address: %s", s_addr_str);

            interact(sock);

            shutdown(sock, 0);
            close(sock);
        }

socket_error:
        vTaskDelay(10); // TODO: Magic Delay
        if (listen_sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(listen_sock, 0);
            vTaskDelay(10 / portTICK_PERIOD_MS); // TODO: Magic Delay
            close(listen_sock);
        }
    }

    vTaskDelete(NULL);
}