#ifndef SYS_H_
#define SYS_H_


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_err.h"

#include "settings.h"

#define MIN(a, b) (a)<(b)?(a):(b)
#define MAX(a, b) (a)<(b)?(b):(a)

/** Variable manager **/
#define CONFIG_MAX_VAR_NAME_LEN 16
#define CONFIG_VAR_NVS_TABLE_NAME "var"
#define CONFIG_VAR_STR_MAX_LEN 32
typedef enum {
    VAR_INT32,
    VAR_FLOAT32,
    VAR_STR,
    VAR_UINT8,
} mcu_var_type_t;

typedef union {
    u_int8_t uint8;
    int32_t int32;
    float float32;
    char *str;
} mcu_var_data_t;

typedef struct {
    char name[CONFIG_MAX_VAR_NAME_LEN];
    mcu_var_type_t type;
} mcu_var_t;
/** **/

typedef struct {
    char device_id[CONFIG_DEVICE_ID_LEN];
    int boot_count;
    int sleep_countup;
    EventGroupHandle_t sys_event_group;
    EventGroupHandle_t wifi_event_group;
    char wifi_ssid[CONFIG_MAX_VAR_NAME_LEN];
    char wifi_psk[CONFIG_MAX_VAR_NAME_LEN];
    char data_host_ip_addr[CONFIG_MAX_VAR_NAME_LEN];
    char ota_host_ip_addr[CONFIG_MAX_VAR_NAME_LEN];
    char ntp_host_ip_addr[CONFIG_MAX_VAR_NAME_LEN];
    int32_t imu_baud;
} mcu_t;

extern mcu_var_t g_mcu_vars[];
extern RTC_DATA_ATTR mcu_t g_mcu;

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
    if ((handle) == NULL) { \
        ESP_LOGI(TAG, "Launching "name" tcp client task"); \
        xTaskCreate(target, \
                    name, \
                    size, \
                    (void*)(arg), \
                    priority, \
                    &(handle)); \
    }

#define launch_task_multicore(target, name, size, arg, priority, handle, core) \
    TaskHandle_t handle = NULL; \
    if ((handle) == NULL) { \
        ESP_LOGI(TAG, "Launching "name" tcp client task"); \
        xTaskCreatePinnedToCore(target, \
                                name, \
                                size, \
                                (void*)(arg), \
                                priority, \
                                &(handle), \
                                core); \
    }

/** System Events**/
#define EV_TCP_CONNECTED_BIT BIT0
#define EV_TCP_CONNECTED // Pseudo define
#define EV_NTP_SYNCED_BIT BIT1
#define EV_NTP_SYNCED
#define EV_IMU_ENABLED_BIT BIT2
#define EV_IMU_ENABLED
#define EV_UART_MANUAL_BLOCK_BIT BIT3
#define EV_UART_MANUAL_BLOCK
#define EV_UART_ACTIVATED_BIT BIT4
#define EV_UART_ACTIVATED

#define set_sys_event(ev) \
        xEventGroupSetBits(g_mcu.sys_event_group, ev##_BIT);
#define clear_sys_event(ev) \
        xEventGroupClearBits(g_mcu.sys_event_group, ev##_BIT);

/** Wi-Fi **/
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1


/** Low power related **/
#define device_init_sleep_countup() g_mcu.sleep_countup = 0
#define device_reset_sleep_countup() g_mcu.sleep_countup = (g_mcu.sleep_countup>0)?0:g_mcu.sleep_countup
#define device_incr_sleep_countup(n) g_mcu.sleep_countup += n
#define deice_always_on() g_mcu.sleep_countup = INT32_MIN

/** @brief func_command related **/
#define COMMAND_FUNCTION(name) \
        esp_err_t command_func_##name(char* rx_buffer, \
                       size_t rx_len, \
                       char* tx_buffer, \
                       size_t tx_len)

/** Device control functions **/
#define os_delay_ms(x) vTaskDelay((x) / portTICK_PERIOD_MS)

#define sys_log_heap_size() \
    ESP_LOGW(TAG, "Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size())


esp_err_t sys_wifi_init_sta(void);

void sys_get_device_id(void);

void sys_button_init(int);

esp_err_t sys_do_ota(void);

void sys_log_chip_info(void);

void sys_reset_gpio(int);

void sys_enter_deep_sleep(void);

esp_err_t device_set_nvs_var(mcu_var_t *p_var, char *value);

esp_err_t device_get_nvs_var(mcu_var_t *p_var, mcu_var_data_t *out, char *value_buffer);

void sys_load_configuration(void);

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

COMMAND_FUNCTION(time);

COMMAND_FUNCTION(always_on);

COMMAND_FUNCTION(varset);

COMMAND_FUNCTION(varget);

COMMAND_FUNCTION(shutdown);

#endif