#ifndef _APPS_H
#define _APPS_H

void app_blink_init();
void app_blink_start();
void app_blink_stop();
void app_udp_client(void *pvParameters);
void app_tcp_client(void *pvParameters);
void app_uart_monitor(void * pvParameters);
void app_time_sync(void * pvParameters);
void app_controller(void *pvParameters);

#endif