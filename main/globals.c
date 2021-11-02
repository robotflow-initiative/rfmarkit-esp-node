#include "globals.h"
#include "settings.h"
#include "types.h"
gy95_t g_gy95_imu = { 0 };

#if CONFIG_EN_DEBUG_OVER_TCP
int g_debug_sock = -1;
char g_debug_buffer[TCP_DEBUG_BUFFER_LEN] = {0};
#endif

RTC_DATA_ATTR int boot_count = 0;

EventGroupHandle_t g_sys_event_group = NULL;
EventGroupHandle_t g_wifi_event_group;