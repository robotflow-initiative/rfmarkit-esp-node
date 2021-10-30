#ifndef _MAIN_H
#define _MAIN_H

#include "gy95.h"
#define __FIRMWARE_VERSION__ "1.1.7"

extern gy95_t g_gy95_imu;

/** TCP debug **/
#include "lwip/sockets.h"

#define TCP_DEBUG_BUFFER_LEN 128
extern int debug_sock;
extern char debug_buffer[TCP_DEBUG_BUFFER_LEN];

#define ESP_LOGSOCKET(sock, TAG, ...) snprintf((debug_buffer), (TCP_DEBUG_BUFFER_LEN), __VA_ARGS__); if (sock >0) send((sock), (debug_buffer), (strlen(debug_buffer)), (0))

#endif