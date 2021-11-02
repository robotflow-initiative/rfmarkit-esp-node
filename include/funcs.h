#ifndef _FUNCS_H
#define _FUNCS_H

#include "types.h"

/** @brief func_dev **/
esp_err_t wifi_init_sta(void);
void esp_enter_light_sleep(void);
void esp_enter_deep_sleep(void);
void esp_get_device_id(void);
void uart_service_init(int port, int rx, int tx, int rts, int cts);

/** @brief func_parse **/
esp_err_t parse_imu_reading(imu_msg_raw_t* p_reading, char * buffer, int len);
esp_err_t tag_imu_reading(imu_msg_raw_t* p_reading, uint8_t* payload, int len);

/** @brief func_ota **/
esp_err_t esp_do_ota();

/** @brief func_command **/
esp_err_t command_func_restart(char* tx_buffer, int tx_len);
esp_err_t command_func_ping(char* tx_buffer, int tx_len);
esp_err_t command_func_sleep(char* tx_buffer, int tx_len);
esp_err_t command_func_shutdown(char* tx_buffer, int tx_len);
esp_err_t command_func_update(char* tx_buffer, int tx_len);
esp_err_t command_func_cali_reset(char* tx_buffer, int tx_len);
esp_err_t command_func_cali_acc(char* tx_buffer, int tx_len);
esp_err_t command_func_cali_mag(char* tx_buffer, int tx_len);
esp_err_t command_func_start(char* tx_buffer, int tx_len);
esp_err_t command_func_stop(char* tx_buffer, int tx_len);
esp_err_t command_func_gy_enable(char* tx_buffer, int tx_len);
esp_err_t command_func_gy_disable(char* tx_buffer, int tx_len);
/** FIXME: The status of  gy is output via serial debug port **/
esp_err_t command_func_gy_status(char* tx_buffer, int tx_len);
esp_err_t command_func_gy_imm(char* tx_buffer, int tx_len);
esp_err_t command_func_gy_id(char* tx_buffer, int tx_len);
#endif