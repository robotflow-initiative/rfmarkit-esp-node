#include <sys/cdefs.h>

#ifndef _APPS_H
#define _APPS_H

_Noreturn void app_data_client(void* pvParameters);

_Noreturn void app_uart_monitor(void* pvParameters);

_Noreturn void app_system_loop(void* pvParameters);

#endif