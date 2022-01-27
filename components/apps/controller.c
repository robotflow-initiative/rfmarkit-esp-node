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

#include "tcp.h"
#include "apps.h"
#include "blink.h"
#include "sys.h"
#include "imu.h"
#include "settings.h"

#define RX_BUFFER_LEN CONFIG_CTRL_RX_LEN
#define TX_BUFFER_LEN CONFIG_CTRL_TX_LEN

static const char* TAG = "app_controller";

static char s_rx_buffer[RX_BUFFER_LEN];
static char s_tx_buffer[TX_BUFFER_LEN];

typedef struct command_reg_t {
    char name[16];

    esp_err_t(*func)(char*, size_t, char*, size_t);
} command_reg_t;

static command_reg_t s_registration[] = {
        {.name = "restart", .func = command_func_restart},
        {.name = "ping", .func = command_func_ping},
        {.name = "shutdown", .func = command_func_shutdown},
        {.name = "update", .func = command_func_update},
        {.name = "imu_cali_reset", .func = command_func_imu_cali_reset},
        {.name = "imu_cali_acc", .func = command_func_imu_cali_acc},
        {.name = "imu_cali_mag", .func = command_func_imu_cali_mag},
        {.name = "start", .func = command_func_start},
        {.name = "stop", .func = command_func_stop},
        {.name = "imu_enable", .func = command_func_imu_enable},
        {.name = "imu_disable", .func = command_func_imu_disable},
        {.name = "imu_status", .func = command_func_imu_status},
        {.name = "imu_imm", .func = command_func_imu_imm},
        {.name = "imu_setup", .func = command_func_imu_setup},
        {.name = "imu_scale",.func = command_func_imu_scale},
        {.name = "imu_debug", .func = command_func_imu_debug},
        {.name = "id", .func = command_func_id},
        {.name = "ver", .func = command_func_ver},
        {.name = "blink_set", .func = command_func_blink_set},
        {.name = "blink_get", .func = command_func_blink_get},
        {.name = "blink_start", .func = command_func_blink_start},
        {.name = "blink_stop", .func = command_func_blink_stop},
        {.name = "blink_off", .func = command_func_blink_off},
        {.name = "self_test", .func = command_func_imu_self_test},
        {.name = "always_on", .func = command_func_always_on},
        {.name = "varset", .func = command_func_varset},
        {.name = "varget", .func = command_func_varget},
        {.name = "v"CONFIG_FIRMWARE_VERSION"_shutdown", .func = command_func_shutdown}
};

#define MATCH_CMD(x, cmd) (strncasecmp(x, cmd, strlen(cmd)) == 0)

static command_reg_t* parse_command(char* command, size_t len) {
    ESP_LOGI(TAG, "Got command %s from controller", command);
    for (int idx = 0; idx < sizeof(s_registration) / sizeof(command_reg_t); ++idx) {
        if (MATCH_CMD(command, s_registration[idx].name)) {
            return &s_registration[idx];
        }
    }
    return NULL;
}

esp_err_t execute_command(char* rx_buffer, char* tx_buffer, size_t rx_len, size_t tx_len) {
    command_reg_t* cmd = parse_command(rx_buffer, rx_len);
    esp_err_t ret = ESP_FAIL;
    bzero(tx_buffer, sizeof(char) * tx_len);

    if (cmd == NULL) {
        /** Output error **/
        ESP_LOGE(TAG, "Got invalid command : %s", rx_buffer);

        /** Fill tx_buffer with 'ERROR\n' **/
        if (rx_buffer[0] == '\n') {
            strcpy(tx_buffer, "\n");
        } else {
            strcpy(tx_buffer, "COMMAND NOT FOUND\n\n");
        }
        /** Return False **/
        return ESP_FAIL;
    } else {
        /** Output info **/

        /** Fill tx_buffer with '\0' **/
        /** Fill tx buffer with command related context **/
        ret = cmd->func(rx_buffer, rx_len, tx_buffer, tx_len);

        /** Command did not modify the buffer **/
        if (strlen(tx_buffer) == 0) {
            snprintf(tx_buffer, tx_len, "%s", (ret == ESP_OK) ? "OK\n\n" : "ERROR\n\n");
        }

        /** Return False **/
        return ret;
    }
};

static void interact(const int sock) {
    int recv_len;
    ESP_LOGI(TAG, "Interacting");
    do {
        recv_len = recv(sock, s_rx_buffer, sizeof(s_rx_buffer) - 1, 0);
        if (recv_len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        } else if (recv_len == 0) {
            ESP_LOGW(TAG, "Connection closed");
        } else {
            s_rx_buffer[recv_len] = 0; // Null-terminate whatever is received and treat it like a string
            ESP_LOGI(TAG, "Received %d bytes: %s", recv_len, s_rx_buffer);

            device_reset_sleep_countup();
            execute_command(s_rx_buffer, s_tx_buffer, RX_BUFFER_LEN, TX_BUFFER_LEN);

            send_all(sock, s_tx_buffer);
        }
        bzero(s_rx_buffer, sizeof(s_rx_buffer));
    } while (recv_len > 0);
}

void app_controller(void* pvParameters) {
    ESP_LOGI(TAG, "app_controller started");

    tcp_server_t server = { .port = CONFIG_LOCAL_PORT, .max_listen = 1 };

    server_loop(&server, interact);

    vTaskDelete(NULL);
}
