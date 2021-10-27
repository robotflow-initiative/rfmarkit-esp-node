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
#include "main.h"

#define RX_BUFFER_LEN 64 
#define TX_BUFFER_LEN 64
static const char* TAG = "app_controller";

typedef enum server_command_t {
    RESTART = 0,
    PING = 1,
    SLEEP = 2,
    STOP = 3,
    UPDATE = 4,
    CALI_RESET = 5,
    CALI_ACC = 6,
    CALI_MAG = 7,
    ERROR,
} server_command_t;

esp_err_t command_func_restart(void) {
    esp_restart();
    return ESP_OK;
}

esp_err_t command_func_ping(void) {
    return ESP_OK;
}

esp_err_t command_func_sleep(void) {
    esp_enter_light_sleep();
    return ESP_OK;
}

esp_err_t command_func_stop(void) {
    esp_deep_sleep_start();
    return ESP_OK;
}

esp_err_t command_func_update(void) {
    esp_do_ota();
    return ESP_OK;
}

esp_err_t command_func_cali_reset(void) {
    gy95_cali_reset(&g_gy95_imu);
    return ESP_OK;
}

esp_err_t command_func_cali_acc(void) {
    gy95_cali_acc(&g_gy95_imu);
    return ESP_OK;
}

esp_err_t command_func_cali_mag(void) {
    gy95_cali_mag(&g_gy95_imu);
    return ESP_OK;
}

esp_err_t(*command_funcs[])(void) = {
    command_func_restart,
    command_func_ping,
    command_func_sleep,
    command_func_stop,
    command_func_update,
    command_func_cali_reset,
    command_func_cali_acc,
    command_func_cali_mag
};


char* command_func_descripts[] = {
    "RESTART",
    "PING",
    "SLEEP",
    "STOP",
    "UPDATE",
    "CALI_RESET",
    "CALI_ACC",
    "CALI_MAG"
};

#define MATCH_CMD(x, cmd) (strncmp(x, cmd, strlen(cmd)) == 0)

server_command_t parse_command(char* command, int len) {
    ESP_LOGI(TAG, "Got command %s from controller", command);
    if (MATCH_CMD(command, "restart")) {
        return RESTART;
    } else if (MATCH_CMD(command, "ping")) {
        return PING;
    } else if (MATCH_CMD(command, "sleep")) {
        return SLEEP;
    } else if (MATCH_CMD(command, "stop")) {
        return STOP;
    } else if (MATCH_CMD(command, "update")) {
        return UPDATE;
    } else if (MATCH_CMD(command, "cali_reset")) {
        return CALI_RESET;
    } else if (MATCH_CMD(command, "cali_acc")) {
        return CALI_ACC;
    } else if (MATCH_CMD(command, "cali_mag")) {
        return CALI_MAG;
    } else {
        return ERROR;
    }
}

esp_err_t execute_command(char* rx_buffer, char* tx_buffer, size_t rx_len, size_t tx_len) {
    server_command_t cmd = parse_command(rx_buffer, rx_len);
    esp_err_t ret;
    if (cmd == ERROR) {
        /** Output error **/
        ESP_LOGE(TAG, "Got invalid command : %s", rx_buffer);

        /** Fill tx_buffer with 'ERROR\n' **/
        memset(tx_buffer, 0, sizeof(char) * tx_len);
        strcpy(tx_buffer, "ERROR\n");

        /** Return False **/
        return ESP_FAIL;
    } else {
        /** Output info **/
        ESP_LOGI(TAG, "Executing command : %s", command_func_descripts[cmd]);

        ret = command_funcs[cmd]();

        /** Fill tx_buffer with 'OK\n' or 'ERROR\n' **/
        memset(tx_buffer, 0, sizeof(char) * tx_len);
        snprintf(tx_buffer, tx_len, "%s", (ret==ESP_OK)? "OK\n" : "ERROR\n");

        /** Return False **/
        return ret;
    }
    return true;
};

void app_controller(void* pvParameters) {
    ESP_LOGI(TAG, "app_controller started");

    char rx_buffer[RX_BUFFER_LEN];
    char tx_buffer[TX_BUFFER_LEN];

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

        /** Bind socket **/
        int err = bind(sock, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
            goto socket_error;
        }

        ESP_LOGI(TAG, "Socket bound, port %d", CONFIG_LOCAL_PORT);

        /** Set socket options **/
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));

        struct sockaddr_storage source_addr;
        socklen_t socklen = sizeof(source_addr);

        while (1) {

            ESP_LOGD(TAG, "Waiting for data");

            /** Receiving **/
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr*)&source_addr, &socklen);


            if (len < 0) {
                /** Receive nothing **/
                ESP_LOGD(TAG, "Recvfrom failed: errno %d", errno);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            /** Data received **/
            else {
                /** Get the sender's ip address as string **/
                inet_ntoa_r(((struct sockaddr_in*)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);

                /** Process with command from controller **/
                rx_buffer[len] = 0; // Null-terminate
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);
                execute_command(rx_buffer, tx_buffer, RX_BUFFER_LEN, TX_BUFFER_LEN);


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