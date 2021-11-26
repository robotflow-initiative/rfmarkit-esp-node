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
#include "lwip/inet.h"
#include "lwip/netdb.h"

#include "apps.h"
#include "blink.h"
#include "device.h"
#include "gy95.h"
#include "settings.h"

#define RX_BUFFER_LEN 64 
#define TX_BUFFER_LEN 512

static const char* TAG = "app_controller";

static char s_addr_str[128];
static char s_rx_buffer[RX_BUFFER_LEN];
static char s_tx_buffer[TX_BUFFER_LEN];

typedef struct command_reg_t {
    char name[16];
    esp_err_t(*func)(char*, int, char*, int);
} command_reg_t;

static command_reg_t s_registration[] = {// TODO: Use linked list to store function
    {.name = "restart", .func=command_func_restart},
    {.name = "ping", .func=command_func_ping},
    {.name = "shutdown", .func=command_func_shutdown},
    {.name = "update", .func=command_func_update},
    {.name = "imu_cali_reset", .func=command_func_imu_cali_reset},
    {.name = "imu_cali_acc", .func=command_func_imu_cali_acc},
    {.name = "imu_cali_mag", .func=command_func_imu_cali_mag},
    {.name = "start", .func=command_func_start},
    {.name = "stop", .func=command_func_stop},
    {.name = "imu_enable", .func=command_func_imu_enable},
    {.name = "imu_disable", .func=command_func_imu_disable},
    {.name = "imu_status", .func=command_func_imu_status},
    {.name = "imu_imm", .func=command_func_imu_imm},
    {.name = "imu_setup", .func=command_func_imu_setup},
    {.name = "imu_scale",.func=command_func_imu_scale},
    {.name = "id", .func=command_func_id},
    {.name = "ver", .func=command_func_ver},
    {.name = "blink_set", .func=command_func_blink_set},
    {.name = "blink_get", .func=command_func_blink_get},
    {.name = "blink_start", .func=command_func_blink_start},
    {.name = "blink_stop", .func=command_func_blink_stop},
    {.name = "blink_off",.func=command_func_blink_off},
    {.name = "self_test", .func=command_func_self_test},
    {.name = "always_on", .func=command_func_always_on},
    {.name = "v"CONFIG_FIRMWARE_VERSION"_shutdown", .func=command_func_shutdown}
};

#define MATCH_CMD(x, cmd) (strncasecmp(x, cmd, strlen(cmd)) == 0)

command_reg_t * parse_command(char* command, int len) {
    ESP_LOGI(TAG, "Got command %s from controller", command);
   for (int idx = 0; idx < sizeof(s_registration) / sizeof(command_reg_t); ++idx) {
       if (MATCH_CMD(command, s_registration[idx].name)) {
           return &s_registration[idx];
       }
   }
   return NULL;
}

esp_err_t execute_command(char* rx_buffer, char* tx_buffer, size_t rx_len, size_t tx_len) {
    command_reg_t * cmd = parse_command(rx_buffer, rx_len);
    esp_err_t ret = ESP_FAIL;
    if (cmd == NULL) {
        /** Output error **/
        ESP_LOGE(TAG, "Got invalid command : %s", rx_buffer);

        /** Fill tx_buffer with 'ERROR\n' **/
        bzero(tx_buffer, sizeof(char) * tx_len);
        strcpy(tx_buffer, "COMMAND NOT FOUND\n\n");

        /** Return False **/
        return ESP_FAIL;
    } else {
        /** Output info **/

        /** Fill tx_buffer with '\0' **/
        bzero(tx_buffer, sizeof(char) * tx_len);
        /** Fill tx buffer with command related context **/
        ret = cmd->func(rx_buffer, rx_len, tx_buffer, tx_len);

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

            RESET_SLEEP_COUNTUP();
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
        esp_delay_ms(100);
        if (listen_sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(listen_sock, 0);
            esp_delay_ms(100);
            close(listen_sock);
        }
    }

    vTaskDelete(NULL);
}