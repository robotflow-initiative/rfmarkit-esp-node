#include <sys/cdefs.h>

#ifndef _APPS_H
#define _APPS_H

_Noreturn
void app_data_client(void* pvParameters);

void app_uart_monitor(void* pvParameters);

void app_time_sync(void* pvParameters);

void app_controller(void* pvParameters);

void app_playground(void* pvParameters);

#endif