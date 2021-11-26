#ifndef _FUNCS_H
#define _FUNCS_H

#include "types.h"
#include "freertos/FreeRTOS.h"
#include "gy95.h"

/** @brief func_dev **/
esp_err_t esp_wifi_init_sta(void);
void esp_enter_deep_sleep(void);
void esp_get_device_id(void);
void esp_button_init(void);
esp_err_t esp_self_test(void);
#define esp_delay_ms(x) vTaskDelay(x / portTICK_PERIOD_MS)

/** @brief func_parse **/
esp_err_t parse_imu_reading(gy95_t* p_gy,
                            imu_dgram_t* p_reading,
                            imu_res_t* p_parsed,
                            char* buffer, int len);
int tag_imu_reading(imu_dgram_t* p_reading, uint8_t* payload, int len);

/** @brief func_ota **/
esp_err_t esp_do_ota();

/** @brief func_command **/
#define COMMAND_FUNCTION(name) \
        esp_err_t command_func_##name(char* rx_buffer, \
                       int rx_len, \
                       char* tx_buffer, \
                       int tx_len)

COMMAND_FUNCTION(restart);
COMMAND_FUNCTION(ping);
COMMAND_FUNCTION(sleep);
COMMAND_FUNCTION(shutdown);
COMMAND_FUNCTION(update);
COMMAND_FUNCTION(cali_reset);
COMMAND_FUNCTION(cali_acc);
COMMAND_FUNCTION(cali_mag);
COMMAND_FUNCTION(start);
COMMAND_FUNCTION(stop);
COMMAND_FUNCTION(gy_enable);
COMMAND_FUNCTION(gy_disable);
COMMAND_FUNCTION(gy_status);
COMMAND_FUNCTION(gy_imm);
COMMAND_FUNCTION(gy_setup);
COMMAND_FUNCTION(gy_scale);
COMMAND_FUNCTION(id);
COMMAND_FUNCTION(ver);
COMMAND_FUNCTION(blink_set);
COMMAND_FUNCTION(blink_start);
COMMAND_FUNCTION(blink_stop);
COMMAND_FUNCTION(blink_get);
COMMAND_FUNCTION(blink_off);
COMMAND_FUNCTION(self_test);
COMMAND_FUNCTION(always_on);
COMMAND_FUNCTION(wifi_set);
COMMAND_FUNCTION(host_set);
#endif