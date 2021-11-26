#ifndef _FUNCS_H
#define _FUNCS_H

#include "freertos/FreeRTOS.h"
#include "settings.h"

/** @brief func_dev **/
esp_err_t esp_wifi_init_sta(void);
void esp_enter_deep_sleep(void);
void esp_get_device_id(void);
void esp_button_init(void);
esp_err_t esp_self_test(void);
#define esp_delay_ms(x) vTaskDelay(x / portTICK_PERIOD_MS)
esp_err_t esp_do_ota();

COMMAND_FUNCTION(restart);
COMMAND_FUNCTION(ping);
COMMAND_FUNCTION(sleep);
COMMAND_FUNCTION(shutdown);
COMMAND_FUNCTION(update);
COMMAND_FUNCTION(start);
COMMAND_FUNCTION(stop);
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