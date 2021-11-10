#ifndef _FUNCS_H
#define _FUNCS_H

#include "types.h"

/** @brief func_dev **/
esp_err_t wifi_init_sta(void);
void esp_enter_light_sleep(void);
void esp_enter_deep_sleep(void);
void esp_get_device_id(void);
void esp_button_init(void);

/** @brief func_parse **/
esp_err_t parse_imu_reading(imu_msg_raw_t* p_reading, char* buffer, int len);
esp_err_t tag_imu_reading(imu_msg_raw_t* p_reading, uint8_t* payload, int len);

/** @brief func_ota **/
esp_err_t esp_do_ota();

/** @brief func_command **/
esp_err_t command_func_restart(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len);
esp_err_t command_func_ping(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len);
esp_err_t command_func_sleep(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len);
esp_err_t command_func_shutdown(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len);
esp_err_t command_func_update(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len);
esp_err_t command_func_cali_reset(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len);
esp_err_t command_func_cali_acc(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len);
esp_err_t command_func_cali_mag(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len);
esp_err_t command_func_start(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len);
esp_err_t command_func_stop(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len);
esp_err_t command_func_gy_enable(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len);
esp_err_t command_func_gy_disable(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len);
esp_err_t command_func_gy_status(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len);
esp_err_t command_func_gy_imm(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len);
esp_err_t command_func_gy_setup(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len);
esp_err_t command_func_id(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len);
esp_err_t command_func_ver(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len);
esp_err_t command_func_blink_set(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len);
esp_err_t command_func_blink_start(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len);
esp_err_t command_func_blink_stop(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len);
esp_err_t command_func_blink_get(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len);
esp_err_t command_func_blink_off(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len);
#endif