#ifndef _GLOBALS_H
#define _GLOBALS_H

#include "settings.h"

/** @brief Device properties **/
extern char g_device_id[14];
extern RTC_DATA_ATTR int boot_count;
extern int g_sleep_countup;
#define RESET_SLEEP_COUNTUP() g_sleep_countup = (g_sleep_countup>0)?0:g_sleep_countup

/** @brief TCP Debug related **/
#include "lwip/sockets.h"
#if CONFIG_EN_DEBUG_OVER_TCP
#define TCP_DEBUG_BUFFER_LEN 128
extern int g_debug_sock;
extern char g_debug_buffer[TCP_DEBUG_BUFFER_LEN];

#define ESP_LOGSOCKET(TAG, ...) snprintf((g_debug_buffer), (g_debug_buffer), __VA_ARGS__); if (g_debug_sock >0) send((g_debug_sock), (g_debug_buffer), (strlen(g_debug_buffer)), (0))
#define SET_DEBUG_SOCK(x) g_debug_sock = x
#else
#define ESP_LOGSOCKET(sock, TAG, ...)
#define SET_DEBUG_SOCK(...)
#endif


/** @brief GY95 related **/
#include "gy95.h"
extern gy95_t g_imu;

/** @brief Global Events **/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

/** System **/
#define TCP_CONNECTED_BIT BIT0
#define NTP_SYNCED_BIT BIT1
#define GY95_ENABLED_BIT BIT2
#define UART_BLOCK_BIT BIT3
#define UART_ACTIVE_BIT BIT4
extern EventGroupHandle_t g_sys_event_group;

/** Wi-Fi **/
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
extern EventGroupHandle_t g_wifi_event_group;

/** Blink **/
extern uint8_t g_blink_pin;
#endif

