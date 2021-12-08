#ifndef _FUNCS_H
#define _FUNCS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "settings.h"

typedef struct {
    char device_id[CONFIG_DEVICE_ID_LEN];
    int boot_count;
    int sleep_countup;
    EventGroupHandle_t sys_event_group;
    EventGroupHandle_t wifi_event_group;
    uint8_t blink_pin;
} mcu_t;

extern RTC_DATA_ATTR mcu_t g_mcu;

/** Low power related **/
#define device_init_sleep_countup() g_mcu.sleep_countup = 0
#define device_reset_sleep_countup() g_mcu.sleep_countup = (g_mcu.sleep_countup>0)?0:g_mcu.sleep_countup
#define device_incr_sleep_countup(n) g_mcu.sleep_countup += n

/** @brief TCP Debug related **/ // TODO: Complete TCP debug function
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

/** FreeRTOS related **/
#define launch_task(target, name, size, arg, priority, handle) \
    TaskHandle_t handle = NULL; \
    if (handle == NULL) { \
        ESP_LOGI(TAG, "Launching "name" tcp client task"); \
        xTaskCreate(target, \
                    name, \
                    size, \
                    (void*)(arg), \
                    priority, \
                    &handle); \
    }

#define launch_task_multicore(target, name, size, arg, priority, handle, core) \
    TaskHandle_t handle = NULL; \
    if (handle == NULL) { \
        ESP_LOGI(TAG, "Launching "name" tcp client task"); \
        xTaskCreatePinnedToCore(target, \
                                name, \
                                size, \
                                (void*)arg, \
                                priority, \
                                &handle, \
                                core); \
    }

/** System **/
#define TCP_CONNECTED_BIT BIT0
#define NTP_SYNCED_BIT BIT1
#define IMU_ENABLED_BIT BIT2
#define UART_BLOCK_BIT BIT3
#define UART_ACTIVE_BIT BIT4

/** Wi-Fi **/
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

/** @brief func_command related **/
#define COMMAND_FUNCTION(name) \
        esp_err_t command_func_##name(char* rx_buffer, \
                       int rx_len, \
                       char* tx_buffer, \
                       int tx_len)

/** Device control functions **/
#define device_delay_ms(x) vTaskDelay(x / portTICK_PERIOD_MS)

#define device_log_heap_size() \
    ESP_LOGW(TAG, "Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size())

esp_err_t device_wifi_init_sta(void);
void device_get_device_id(void);
void device_button_init(int);
esp_err_t device_do_ota(void);
void device_log_chip_info(void);
void device_reset_gpio(int);

/** Register command functions **/
COMMAND_FUNCTION(restart); 
COMMAND_FUNCTION(ping);
COMMAND_FUNCTION(sleep);
COMMAND_FUNCTION(shutdown);
COMMAND_FUNCTION(update);
COMMAND_FUNCTION(start);
COMMAND_FUNCTION(stop);
COMMAND_FUNCTION(id);
COMMAND_FUNCTION(ver);
COMMAND_FUNCTION(self_test);
COMMAND_FUNCTION(always_on);
COMMAND_FUNCTION(wifi_set);
COMMAND_FUNCTION(host_set);

#endif