#include "functions.h"

#include "esp_err.h"
#include "esp_log.h"
#include "globals.h"

static const char * TAG = "func_command";

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
}·

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
        snprintf(tx_buffer + offset, tx_len - offset, "0x%02x, ", imu_data.data[idx]);
        offset = strlen(tx_buffer);
    };
    snprintf(tx_buffer + offset, tx_len - offset, "\n\n");

    return ESP_OK;
}

esp_err_t command_func_gy_id(char* tx_buffer, int tx_len){
    ESP_LOGI(TAG, "Executing command : IMU_GY_ID");
    snprintf(tx_buffer, tx_len, "%s\n\n", g_device_id);
    return ESP_OK;
}