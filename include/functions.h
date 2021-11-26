#ifndef _FUNCS_H
#define _FUNCS_H

#include "freertos/FreeRTOS.h"

#include "settings.h"
#include "sys.h"

/** @brief func_dev **/
esp_err_t esp_wifi_init_sta(void);
void esp_enter_deep_sleep(void);
void esp_get_device_id(void);
void esp_button_init(void);
#define esp_delay_ms(x) vTaskDelay(x / portTICK_PERIOD_MS)
esp_err_t esp_do_ota();


#endif