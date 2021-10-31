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
#define TX_BUFFER_LEN 256
static const char* TAG = "app_controller";

esp_err_t command_func_restart(char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_RESTART");
    esp_restart();
    return ESP_OK;
}

esp_err_t command_func_ping(char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_PING");
    return ESP_OK;
}

esp_err_t command_func_sleep(char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_SLEEP");
    esp_enter_light_sleep();
    return ESP_OK;
}

esp_err_t command_func_shutdown(char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_SHUTDOWN");
    esp_enter_deep_sleep();
    return ESP_OK;
}

esp_err_t command_func_update(char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_UPDATE");
    xEventGroupSetBits(g_sys_event_group, UART_BLOCK_BIT);
    esp_err_t ret = esp_do_ota();
    return ret;
}

esp_err_t command_func_cali_reset(char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_RESET");
    gy95_cali_reset(&g_gy95_imu);
    return ESP_OK;
}

esp_err_t command_func_cali_acc(char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_CALI_ACC");
    gy95_cali_acc(&g_gy95_imu);
    return ESP_OK;
}

esp_err_t command_func_cali_mag(char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_CALI_MAG");
    gy95_cali_mag(&g_gy95_imu);
    return ESP_OK;
}

esp_err_t command_func_start(char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_START");
    xEventGroupClearBits(g_sys_event_group, UART_BLOCK_BIT);
    return ESP_OK;
}

esp_err_t command_func_stop(char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_STOP");
    xEventGroupSetBits(g_sys_event_group, UART_BLOCK_BIT);
    return ESP_OK;
}

esp_err_t command_func_gy_enable(char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_ENABLE");
    gy95_enable(&g_gy95_imu);
    return ESP_OK;
}

esp_err_t command_func_gy_disable(char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_DISABLE");
    gy95_disable(&g_gy95_imu);
    return ESP_OK;
}

/** FIXME: The status of  gy is output via serial debug port **/
esp_err_t command_func_gy_status(char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_STATUS");

    int ret = gpio_get_level(g_gy95_imu.ctrl_pin);
    ESP_LOGI(TAG, "GY95 control pin %s\n\n", ret ? "HIGH" : "LOW");
    snprintf(tx_buffer, tx_len, "GY95 control pin %s", ret ? "HIGH" : "LOW");
    return ESP_OK;
}

esp_err_t command_func_gy_imm(char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_IMM");
    imu_msg_raw_t imu_data;
    gy95_read(&g_gy95_imu);
    memcpy(imu_data.data, g_gy95_imu.buf, GY95_MSG_LEN);
    int offset = 0;
    for (int idx = 0; idx < GY95_MSG_LEN; ++idx) {
        snprintf(tx_buffer + offset, tx_len - offset, "%02x, ", imu_data.data[idx]);
        offset = strlen(tx_buffer);
    };
    snprintf(tx_buffer + offset, tx_len - offset, "\n\n");

    return ESP_OK;
}

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
};

#define MATCH_CMD(x, cmd) (strncmp(x, cmd, strlen(cmd)) == 0)

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
        memset(tx_buffer, 0, sizeof(char) * tx_len);
        strcpy(tx_buffer, "ERROR\n\n");

        /** Return False **/
        return ESP_FAIL;
    } else {
        /** Output info **/

        /** Fill tx_buffer with '\0' **/
        memset(tx_buffer, 0, sizeof(char) * tx_len);
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

void app_controller(void* pvParameters) {
    ESP_LOGI(TAG, "app_controller started");

    char rx_buffer[RX_BUFFER_LEN];
    char tx_buffer[TX_BUFFER_LEN];

    // char addr_str[128];
    int addr_family = 0;
    int ip_protocol = 0;

    struct timeval timeout = { 0, 0 }; // UDP timeout

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
                vTaskDelay(5000 / portTICK_PERIOD_MS);
            }
            /** Data received **/
            else {
                // /** Get the sender's ip address as string **/
                // inet_ntoa_r(((struct sockaddr_in*)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);

                /** Process with command from controller **/
                rx_buffer[len] = 0; // Null-terminate
                ESP_LOGI(TAG, "Received %d bytes", len);
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