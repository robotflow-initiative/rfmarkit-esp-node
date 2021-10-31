#ifndef _MAIN_H
#define _MAIN_H

#include "settings.h"

#include "gy95.h"
#define __FIRMWARE_VERSION__ "1.1.7"

extern gy95_t g_gy95_imu;

/** TCP debug **/
#include "lwip/sockets.h"

#if CONFIG_EN_DEBUG_OVER_TCP
#define TCP_DEBUG_BUFFER_LEN 128
extern int g_debug_sock;
extern char debug_buffer[TCP_DEBUG_BUFFER_LEN];

#define ESP_LOGSOCKET(TAG, ...) snprintf((debug_buffer), (TCP_DEBUG_BUFFER_LEN), __VA_ARGS__); if (g_debug_sock >0) send((g_debug_sock), (debug_buffer), (strlen(debug_buffer)), (0))
#define SET_DEBUG_SOCK(x) g_debug_sock = x

#else

#define ESP_LOGSOCKET(sock, TAG, ...)
#define SET_DEBUG_SOCK(...)
#endif
#endif