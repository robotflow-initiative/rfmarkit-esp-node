#ifndef SYS_H_
#define SYS_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_wifi_types.h"
#include "esp_err.h"

#include "settings.h"
#include "ring_buf.h"

/** Core Macros **/
#define MIN(a, b) (a)<(b)?(a):(b)
#define MAX(a, b) (a)<(b)?(b):(a)

#define for_count_up(it, min, max) \
        for(size_t (it) = min; (it) < (max); ++(it))

#define for_count_down(it, min, max) \
        for(size_t (it) = max; (it) > min; --(it))

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte) \
((byte) & 0x80 ? '1' : '0'), \
((byte) & 0x40 ? '1' : '0'), \
((byte) & 0x20 ? '1' : '0'), \
((byte) & 0x10 ? '1' : '0'), \
((byte) & 0x08 ? '1' : '0'), \
((byte) & 0x04 ? '1' : '0'), \
((byte) & 0x02 ? '1' : '0'), \
((byte) & 0x01 ? '1' : '0')

/** FreeRTOS related **/
#define launch_task(target, name, size, arg, priority, handle) \
    TaskHandle_t handle = NULL; \
    if ((handle) == NULL) { \
        ESP_LOGI(TAG, "launching "name" task"); \
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
        ESP_LOGI(TAG, "launching "name" task"); \
        xTaskCreatePinnedToCore(target, \
                                name, \
                                size, \
                                (void*)(arg), \
                                priority, \
                                &(handle), \
                                core); \
    }
#define os_delay_ms(x) vTaskDelay((x) / portTICK_PERIOD_MS)

/** Time related **/
#define get_time_sec(time) \
    {     struct timeval tv_now = { 0 }; \
          gettimeofday(&tv_now, NULL); \
          time = tv_now.tv_sec; \
    }NULL
#define get_time_usec(time) \
    {     struct timeval tv_now = { 0, 0 }; \
          gettimeofday(&tv_now, NULL); \
          time = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec; \
    }NULL

/** Global System Events**/
#define EV_SYS_ALL_BIT 0x00FFFFFF
#define EV_SYS_ALL

#define EV_SYS_WIFI_CONFIG_UPDATED_BIT  BIT0
#define EV_SYS_WIFI_CONFIG_UPDATED
#define EV_SYS_WIFI_CONNECTED_BIT       BIT1
#define EV_SYS_WIFI_CONNECTED
#define EV_SYS_WIFI_DISCONNECTED_BIT    BIT2
#define EV_SYS_WIFI_DISCONNECTED
#define EV_SYS_WIFI_FAIL_BIT            BIT3
#define EV_SYS_WIFI_FAIL
#define EV_SYS_NTP_SYNCED_BIT           BIT4
#define EV_SYS_NTP_SYNCED
#define EV_SYS_DISCOVERY_COMPLETED_BIT  BIT5
#define EV_SYS_DISCOVERY_COMPLETED
#define EV_SYS_LED_STATUS_CHANGED_BIT   BIT6
#define EV_SYS_LED_STATUS_CHANGED
#define EV_SYS_OTA_TRIGGERED_BIT        BIT7
#define EV_SYS_OTA_TRIGGERED
#define EV_SYS_MODE_CHANGE_BIT          BIT8
#define EV_SYS_MODE_CHANGE
#define EV_SYS_POWER_MGMT_BIT           BIT9
#define EV_SYS_POWER_MGMT

#define set_sys_event(ev) \
        xEventGroupSetBits(g_mcu.sys_event_group, ev##_BIT)
#define clear_sys_event(ev) \
        xEventGroupClearBits(g_mcu.sys_event_group, ev##_BIT)
#define get_sys_event(ev) \
        (xEventGroupClearBits(g_mcu.sys_event_group, 0) & ev##_BIT)

/** Task System Events **/
#define EV_TASK_TIME_SYNC_BIT       BIT0
#define EV_TASK_TIME_SYNC
#define EV_TASK_DISCOVERY_BIT       BIT1
#define EV_TASK_DISCOVERY
#define EV_TASK_LED_STATUS_BIT      BIT2
#define EV_TASK_LED_STATUS
#define EV_TASK_WIFI_RECONNECT_BIT  BIT3
#define EV_TASK_WIFI_RECONNECT

#define set_task_event(ev) \
        xEventGroupSetBits(g_mcu.task_event_group, ev##_BIT)
#define clear_task_event(ev) \
        xEventGroupClearBits(g_mcu.task_event_group, ev##_BIT)
#define get_task_event(ev) \
        (xEventGroupClearBits(g_mcu.task_event_group, 0) & ev##_BIT)


/** NVS Variable manager **/
#define CONFIG_MAX_VAR_NAME_LEN 16
#define CONFIG_VAR_NVS_TABLE_NAME "var"
#define CONFIG_VAR_STR_MAX_LEN 64

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

extern mcu_var_t g_mcu_vars[];

mcu_var_t *sys_find_var(char *name, size_t len);

esp_err_t sys_set_nvs_var(mcu_var_t *p_var, char *value);

esp_err_t sys_get_nvs_var(mcu_var_t *p_var, mcu_var_data_t *out, char *value_buffer);

void sys_init_nvs(void);

/** LED related **/
typedef enum {
    LED_UNKNOWN,
    LED_OFF,
    LED_ON,
    LED_DUTY,
    LED_FAST_FLASH,
    LED_FAST_BREATH,
    LED_SLOW_BREATH,
    LED_SEQ_ENC,
} led_status_enum;

esp_err_t sys_set_led_status(led_status_enum target);

void sys_led_status_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);

/** MCU System **/
typedef struct {
    bool wifi_connected;
    bool ntp_synced;
    int64_t ntp_sync_success_unix_usec;
    bool discovery_completed;
    int64_t last_discovery_unix_usec;
    led_status_enum led_status;
    bool led_manual;
    bool active;
    esp_err_t ota_err;
} mcu_state_t;

typedef struct {
    char device_id[CONFIG_DEVICE_ID_LEN];
    char _device_id_sep_; // separates device_id and ble_local_name
    char ble_local_name[CONFIG_BLE_LOCAL_NAME_LEN];

    /** Volatile Variables **/
    ring_buf_t imu_ring_buf;
    mcu_state_t state;

    EventGroupHandle_t sys_event_group;
    EventGroupHandle_t task_event_group;
    EventGroupHandle_t wifi_event_group;
    esp_event_loop_handle_t system_loop;

    /** Non Volatile Variables **/
    char wifi_ssid[CONFIG_VAR_STR_MAX_LEN];
    char wifi_psk[CONFIG_VAR_STR_MAX_LEN];
    char data_host_ip_addr[CONFIG_VAR_STR_MAX_LEN];
    char ota_host[CONFIG_VAR_STR_MAX_LEN];
    char ntp_host_ip_addr[CONFIG_VAR_STR_MAX_LEN];
    int32_t use_hamming;
    int32_t imu_baud;
    int32_t seq;
} mcu_t;

typedef enum {
    SYSTEM_TIME_SYNC_EVENT,
    SYSTEM_DISCOVERY_EVENT,
    SYSTEM_LED_STATUS_EVENT,
    SYSTEM_WIFI_RECONNECT_EVENT,
    SYSTEM_POWER_MGMT_EVENT,
    SYSTEM_OTA_EVENT,
    SYSTEM_MODE_CHANGE_EVENT,
} sys_event_type_t;

ESP_EVENT_DECLARE_BASE(SYSTEM_EVENTS);

extern mcu_t g_mcu;

esp_err_t sys_ota_perform();

void sys_ota_guard(void);

void sys_init_chip(void);

void sys_log_heap_size(void);

void sys_log_trace(void);

void sys_init_events(void);

esp_err_t sys_set_operation_mode(bool active);

void sys_mode_change_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);

/** Power Management related **/
#define CONFIG_WAKE_UP_TIME_SEC 20

typedef enum {
    POWER_UNKNOWN = -1,
    POWER_NORMAL_BOOT,
    POWER_STANDBY, // RF is ON, no TX
    POWER_ACTIVE,  // RF is ON, TX
    POWER_SAVE, // RF is OFF, no TX, system pause and check IMU periodically
    POWER_DEEP_SLEEP,
    POWER_WAKEN
} power_mgmt_state_t;

typedef enum {
    POWER_MODE_PERFORMANCE, // Deep-Sleep is disabled
    POWER_MODE_NORMAL, // Deep-Sleep is enabled, BLE is on
    POWER_MODE_LOW_ENERGY, // Deep-Sleep is enabled, BLE is off
} power_mode_t;

typedef struct {
    /** Non-Volatile Variables **/
    bool initialized;
    power_mgmt_state_t state;
    power_mode_t mode;
    int boot_count;

    /** Volatile Variables **/
    SemaphoreHandle_t mutex;
} power_mgmt_info_t;

extern RTC_FAST_ATTR power_mgmt_info_t g_power_mgmt_info;

esp_err_t power_mgmt_init();

power_mgmt_state_t power_mgmt_wake_up_handler();

esp_err_t power_mgmt_on_enter_standby();

esp_err_t power_mgmt_on_enter_active();

esp_err_t power_mgmt_on_enter_power_save();

esp_err_t power_mgmt_on_enter_deep_sleep(bool);

void sys_power_mgmt_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);

/** WiFi related **/

/** Wi-Fi Events **/
#define EV_WIFI_CONNECTED_BIT   BIT0
#define EV_WIFI_FAIL_BIT        BIT1

esp_err_t sys_wifi_msp_init(void);

esp_err_t sys_wifi_try_connect(wifi_config_t *wifi_config);

esp_err_t sys_wifi_prepare_config(wifi_config_t *wifi_config, const char *ssid, const char *psk);

esp_err_t sys_wifi_try_disconnect(void);

void sys_wifi_reconnect_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);

/** Button related **/
void sys_init_buttons();

/** Time Sync related **/
void sys_time_sync_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);

/** Discovery related **/
void sys_discovery_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);

#endif