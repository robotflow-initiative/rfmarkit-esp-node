#ifndef _FUNCS_H
#define _FUNCS_H

#include "types.h"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

/** @brief Global device ID **/
extern char g_device_id[14];

void wifi_init_sta(void);
void esp_enter_light_sleep(void);
void esp_get_device_id(void);
esp_err_t parse_imu_reading(imu_msg_raw_t* p_reading, char * buffer, int len);
void uart_service_init(int port, int rx, int tx, int rts, int cts);
esp_err_t esp_do_ota();
#endif