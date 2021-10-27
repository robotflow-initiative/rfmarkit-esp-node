#ifndef _APPS_H
#define _APPS_H
void app_blink(char * seq, int length);
void app_udp_client(void *pvParameters);
void app_tcp_client(void *pvParameters);
void app_uart_monitor(void * pvParameters);
void app_time_sync(void * pvParameters);
void app_controller(void *pvParameters);

#endif