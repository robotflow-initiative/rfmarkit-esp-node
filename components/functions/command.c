#include "functions.h"

#include "esp_err.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "globals.h"
#include "apps.h"

static const char* TAG = "func_command";

esp_err_t command_func_restart(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_RESTART");
    esp_restart();
    return ESP_OK;
}

esp_err_t command_func_ping(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_PING");
    return ESP_OK;
}

esp_err_t command_func_sleep(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_SLEEP");
    esp_enter_light_sleep();
    return ESP_OK;
}

esp_err_t command_func_shutdown(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_SHUTDOWN");
    esp_enter_deep_sleep();
    return ESP_OK;
}

esp_err_t command_func_update(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_UPDATE");
    xEventGroupSetBits(g_sys_event_group, UART_BLOCK_BIT);
    esp_err_t ret = esp_do_ota();
    return ret;
}

esp_err_t command_func_cali_reset(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_RESET");
    esp_err_t err = gy95_cali_reset(&g_imu);

    if (err == ESP_OK) {
        snprintf(tx_buffer, tx_len, "CALI_RESET_OK\n\n");

    } else {
        snprintf(tx_buffer, tx_len, "CALI_RESET_FAIL\n\n");

    }
    return ESP_OK;
}

esp_err_t command_func_cali_acc(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_CALI_ACC");
    esp_err_t err = gy95_cali_acc(&g_imu);
    if (err == ESP_OK) {
        snprintf(tx_buffer, tx_len, "CALI_ACC_OK\n\n");

    } else {
        snprintf(tx_buffer, tx_len, "CALI_ACC_FAIL\n\n");

    }
    return ESP_OK;
}

esp_err_t command_func_cali_mag(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_CALI_MAG");
    gy95_cali_mag(&g_imu);
    return ESP_OK;
}

esp_err_t command_func_start(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_START");
    xEventGroupClearBits(g_sys_event_group, UART_BLOCK_BIT);
    return ESP_OK;
}

esp_err_t command_func_stop(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_STOP");
    xEventGroupSetBits(g_sys_event_group, UART_BLOCK_BIT);
    return ESP_OK;
}

esp_err_t command_func_gy_enable(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_ENABLE");
    gy95_enable(&g_imu);
    return ESP_OK;
}

esp_err_t command_func_gy_disable(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_DISABLE");
    gy95_disable(&g_imu);
    return ESP_OK;
}

/** FIXME: The status of  gy is output via serial debug port **/
esp_err_t command_func_gy_status(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_STATUS");

    int ret = gpio_get_level(g_imu.ctrl_pin);
    ESP_LOGI(TAG, "GY95 control pin %s\n\n", ret ? "HIGH" : "LOW");
    snprintf(tx_buffer, tx_len, "GY95 control pin %s\n\n", ret ? "HIGH" : "LOW");
    return ESP_OK;
}

esp_err_t command_func_gy_imm(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_IMM");
    imu_msg_raw_t imu_data;
    // uart_flush_input(g_gy95_imu.port);
    uint8_t trash[1];
    for (int idx = 0; idx < 512; ++idx) {
        uart_read_bytes(g_imu.port, trash, 1, 0);
    }
    gy95_read(&g_imu);
    memcpy(imu_data.data, g_imu.buf, GY95_MSG_LEN);
    int offset = 0;

    parse_imu_reading(&imu_data, tx_buffer, tx_len);
    offset = strlen(tx_buffer);
    snprintf(tx_buffer + offset, tx_len - offset, "\n\n");
    // for (int idx = 0; idx < GY95_MSG_LEN; ++idx) {
    //     snprintf(tx_buffer + offset, tx_len - offset, "0x%02x, ", imu_data.data[idx]);
    //     offset = strlen(tx_buffer);
    // };
    // snprintf(tx_buffer + offset, tx_len - offset, "\n\n");

    return ESP_OK;
}

esp_err_t command_func_gy_setup(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_SETUP");
    esp_err_t err = gy95_setup(&g_imu);
    if (err == ESP_OK) {
        snprintf(tx_buffer, tx_len, "SETUP_OK\n\n");

    } else {
        snprintf(tx_buffer, tx_len, "SETUP_FAIL\n\n");

    }
    return ESP_OK;
}

esp_err_t command_func_id(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_ID");
    snprintf(tx_buffer, tx_len, "%s\n\n", g_device_id);
    return ESP_OK;
}

esp_err_t command_func_ver(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_ver");
    snprintf(tx_buffer, tx_len, "%s\n\n", CONFIG_FIRMWARE_VERSION);
    return ESP_OK;
}

esp_err_t command_func_blink_set(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_BLINK_SET");
    uint8_t seq = 0;
    uint8_t pin = CONFIG_BLINK_DEFAULT_PIN;
    int offset = 0;

    /** Open nvs table **/
    nvs_handle_t blink_handle;
    nvs_open("blink", NVS_READWRITE, &blink_handle);

    /** usage: blink_set R [0-255] **/
    switch (rx_buffer[10]) // TODO: Magic Number
    {
    case 'R':
    case 'r':
        pin = CONFIG_BLINK_RED_PIN;
        snprintf(tx_buffer + offset, tx_len - offset, "Blink pin set to RED_PIN; ");
        offset += strlen(tx_buffer);
        break;
    case 'G':
    case 'g':
        pin = CONFIG_BLINK_GREEN_PIN;
        snprintf(tx_buffer + offset, tx_len - offset, "Blink pin set to GREEN_PIN; ");
        offset += strlen(tx_buffer);
        break;
    case 'B':
    case 'b':
        pin = CONFIG_BLINK_BLUE_PIN;
        snprintf(tx_buffer + offset, tx_len - offset, "Blink pin set to BLUE_PIN; ");
        offset += strlen(tx_buffer);
        break;
    default:
        snprintf(tx_buffer + offset, tx_len - offset, "Blink set failed\n\n");
        offset += strlen(tx_buffer);
        return ESP_FAIL;
    }

    nvs_set_u8(blink_handle, "pin", pin);
    nvs_commit(blink_handle);

    seq = strtol(rx_buffer + 12, NULL, 10) % 0x100; // TODO: Magic Number 
    nvs_set_u8(blink_handle, "seq", seq);
    nvs_commit(blink_handle);

    snprintf(tx_buffer + offset, tx_len - offset, "Blink seq set to %d, reboot to make effective\n\n", seq);

    nvs_close(blink_handle);
    return ESP_OK;
}

esp_err_t command_func_blink_start(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_BLINK_START");

    app_blink_start();

    snprintf(tx_buffer, tx_len, "Blink started\n\n");
    return ESP_OK;
}

esp_err_t command_func_blink_stop(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_BLINK_STOP");

    app_blink_stop();

    snprintf(tx_buffer, tx_len, "Blink stopped\n\n");
    return ESP_OK;
}

esp_err_t command_func_blink_get(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_BLINK_GET");
    uint8_t seq = 0;
    uint8_t pin = 0;

    /** Open nvs table **/
    nvs_handle_t blink_handle;
    nvs_open("blink", NVS_READWRITE, &blink_handle);

    nvs_get_u8(blink_handle, "pin", &pin);
    nvs_get_u8(blink_handle, "seq", &seq);

    snprintf(tx_buffer, tx_len, "Blink pin is %d, seq is %d, \n\n", pin, seq);

    nvs_close(blink_handle);
    return ESP_OK;
}

esp_err_t command_func_blink_off(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_BLINK_OFF");

    app_blink_stop();
    
    gpio_set_level(g_blink_pin, !CONFIG_BLINK_LED_ENABLE_VALUE);

    snprintf(tx_buffer, tx_len, "Blink pin %d set to OFF\n\n", g_blink_pin);

    return ESP_OK;
}