#ifndef _EVENTS_H
#define _EVENTS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#define TCP_CONNECTED_BIT BIT0
#define NTP_SYNCED_BIT BIT1
#define GY95_CALIBRATED_BIT BIT2
#define UART_BLOCK_BIT BIT3
#define TCP_READY_BIT BIT4

extern EventGroupHandle_t g_wifi_event_group;
extern EventGroupHandle_t sys_event_group;

#endif