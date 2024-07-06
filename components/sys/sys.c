#include <string.h>
#include <esp_event.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_https_ota.h"
#include "esp_ota_ops.h"
#include "nvs_flash.h"

#include "sys.h"
#include "imu.h"
#include "settings.h"
#include "apps.h"
#include "battery.h"

/** MCU structure **/
mcu_t g_mcu = {0};

static const char *TAG = "sys             ";

static esp_err_t esp_ota_event_handler(esp_http_client_event_t *evt) {
    switch (evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}

/**
 * Perform OTA operation
 * @return
**/
esp_err_t sys_ota_perform() {
    ESP_LOGI(TAG, "[ota] performing OTA, url=%s", g_mcu.ota_url);
    esp_http_client_config_t config = {
        .url = g_mcu.ota_url,
        .max_authorization_retries = 3,
        .auth_type = HTTP_AUTH_TYPE_NONE,
        .event_handler = esp_ota_event_handler,
        .keep_alive_enable = true,
        .skip_cert_common_name_check=true
    };
    esp_err_t ret = esp_https_ota(&config);
    if (ret == ESP_OK) {
        esp_restart();
    } else {
        ESP_LOGW(TAG, "[ota] OTA update error, return code: %d", (int) ret);
        g_mcu.state.ota_err = ret;
        g_power_mgmt_ctx.no_sleep = false; // change back to normal mode
        return ret;
    }
}

/**
 * Check if there is an uncommitted OTA operation
 * @return
**/
static bool sys_ota_uncommitted() {
    esp_ota_img_states_t ota_state;
    const esp_partition_t *boot_partition = esp_ota_get_boot_partition();
    esp_ota_get_state_partition(boot_partition, &ota_state);
    if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
        return true;
    } else {
        return false;
    }
}

/**
 * Check if the firmware is operational, by trying to perform OTA
**/
void sys_ota_guard() {
    bool ota_uncommitted = sys_ota_uncommitted();
#ifdef CONFIG_OTA_GUARD_ENABLED
    while (1) {
        if (ota_uncommitted) {
            if (g_mcu.state.wifi_connected) {
                ESP_LOGI(TAG, "[ota] checking operational by try ota again (will not succeed)");
                esp_err_t err = sys_ota_perform();
                if (err == ESP_ERR_OTA_ROLLBACK_INVALID_STATE) {
                    ESP_LOGI(TAG, "[ota] firmware is working fine, cancel rollback");
                    esp_ota_mark_app_valid_cancel_rollback(); // cancel rollback
                } else {
                    ESP_LOGI(TAG, "[ota] marker is not working, rolling back");
                    esp_restart(); // rollback to previous firmware by reboot
                }
                break;
            }
            os_delay_ms(2000);
        } else {
            ESP_LOGI(TAG, "[ota] no need to check operational");
            break;
        }
    }
#else
    os_delay_ms(3000); // wait for the system to be ready)
    if (ota_uncommitted) {
        esp_ota_mark_app_valid_cancel_rollback();
        ESP_LOGI(TAG, "[ota] firmware is working fine, cancel rollback");
        esp_ota_mark_app_valid_cancel_rollback(); // cancel rollback
    }
#endif
}

/**
 * Initialize the system, should be called at the beginning of the program
**/
void sys_init_chip() {
    /** Print chip information **/
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGD("", "\n\n\n\n\n\n# -------- Begin of app log -------- #");
    ESP_LOGD(TAG, "This is %s chip with %d CPU core(s), WiFi%s%s, ",
             CONFIG_IDF_TARGET,
             chip_info.cores,
             (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
    ESP_LOGD(TAG, "Silicon revision %d, ", chip_info.revision);
    ESP_LOGD(TAG, "%dMB %s flash", spi_flash_get_chip_size() / (1024 * 1024),
             (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    ESP_LOGD(TAG, "Minimum free heap size: %d bytes", esp_get_minimum_free_heap_size());

    /** Initialize default event loop required for Wi-Fi, etc. **/
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /** Set the device_id **/
    uint8_t base_mac_addr[6];
    esp_efuse_mac_get_default(base_mac_addr);
    char buf[13];
    snprintf(buf, sizeof(buf), "%02x%02x%02x%02x%02x%02x",
             base_mac_addr[0],
             base_mac_addr[1],
             base_mac_addr[2],
             base_mac_addr[3],
             base_mac_addr[4],
             base_mac_addr[5]);
    memcpy(g_mcu.device_id, buf, CONFIG_DEVICE_ID_LEN);

    /** Set the ble_local_name as "markit_<device_id>" **/
    bzero(g_mcu.ble_local_name, CONFIG_BLE_LOCAL_NAME_LEN);
    memcpy(g_mcu.ble_local_name, CONFIG_BLE_LOCAL_NAME_PREFIX, strlen(CONFIG_BLE_LOCAL_NAME_PREFIX));
    memcpy(g_mcu.ble_local_name + strlen(CONFIG_BLE_LOCAL_NAME_PREFIX), buf, CONFIG_DEVICE_ID_LEN);

    ESP_LOGI(TAG, "Device ID: %s", g_mcu.device_id);
    ESP_LOGI(TAG, "BLE LocalName: %s", g_mcu.ble_local_name);
    ESP_LOGW(TAG, "\n-------VERSION-------\n%s\n---------END---------", CONFIG_FIRMWARE_VERSION);

    /** Init the battery monitor **/
    battery_msp_init();

}

/**
 * Initialize the system events, should be called at the beginning of the program
**/
void sys_init_events() {
    g_mcu.sys_event_group = xEventGroupCreate();
    g_mcu.wifi_event_group = xEventGroupCreate();
    g_mcu.task_event_group = xEventGroupCreate();

    // Initially, the system is in idle mode, Wi-Fi is disconnected
    set_sys_event(EV_SYS_WIFI_DISCONNECTED);
}

/**
 * Log the heap size
 * @param event
**/
void sys_log_heap_size(void) {
    ESP_LOGW(TAG, "minimum free heap size: %d bytes", esp_get_minimum_free_heap_size());
}

/**
 * Log the system trace if the system is in debug mode
**/
void sys_log_trace() {
    char InfoBuffer[512] = {0};
    char CPU_RunInfo[1024] = {0};

    vTaskList((char *) &InfoBuffer);
    InfoBuffer[sizeof(InfoBuffer) - 1] = 0; // null-terminate
    printf("---------------------------------------------\r\n\n");
    printf("TaskName        Status  Prior   Free    TaskNo. Core\r\n");
    printf("\r\n%s\r\n", InfoBuffer);

    vTaskGetRunTimeStats((char *) &CPU_RunInfo);
    CPU_RunInfo[sizeof(CPU_RunInfo) - 1] = 0; // null-terminate
    printf("TaskName        CPUCounter      Percent\r\n");
    printf("%s", CPU_RunInfo);
    printf("---------------------------------------------\r\n\n");
    /** Print heap size **/
    printf("IDLE: ****free internal ram %d  all heap size: %d Bytes****\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           heap_caps_get_free_size(MALLOC_CAP_8BIT));
    printf("IDLE: ****free SPIRAM size: %d Bytes****\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

#if CONFIG_EN_PROFILING
static _Noreturn void app_log_trace(void * pvParameters) {
    while (1) {
        os_delay_ms(CONFIG_MAIN_LOOP_DUTY_PERIOD_S * 1000);
        sys_log_heap_size();
        sys_log_trace();
    }
}
#endif

/**
 * Start all required tasks in a normal boot scenario or resume from power save
**/
void sys_start_tasks(void) {
#if CONFIG_EN_MULTI_CORE
    launch_task_multicore(app_data_client, "app_data_client", 4096, NULL, 12, g_mcu.tasks.app_data_client_task, 0x1);
    launch_task_multicore(app_monitor, "app_monitor", 4096, NULL, 12, g_mcu.tasks.app_monitor_task, 0x1);
    launch_task_multicore(app_system_loop, "app_system_loop", 4096, NULL, 8, g_mcu.tasks.app_system_loop_task, 0x0);
#if CONFIG_EN_PROFILING
    launch_task_multicore(app_log_trace, "app_log_trace", 4096, NULL, 5, g_mcu.tasks.app_log_trace_task, 0x0);
#endif
#else
    launch_task(app_data_client, "app_data_client", 4096, NULL, 12, g_mcu.tasks.app_data_client_task);
    launch_task(app_uart_monitor, "app_monitor", 4096, NULL, 12, g_mcu.tasks.app_monitor_task);
    launch_task(app_system_loop, "app_system_loop", 4096, NULL, 8, g_mcu.tasks.app_system_loop_task);
#if CONFIG_PROFILING_ENABLED
    if (g_mcu.tasks.app_log_trace_task == NULL){
        launch_task(app_log_trace, "app_log_trace", 2048, NULL, 5, g_mcu.tasks.app_log_trace_task);
    }
#endif
#endif
}

/**
 * Stop all tasks in case enter power save mode
**/
void sys_stop_tasks(void) {
    vTaskDelete(g_mcu.tasks.app_system_loop_task);
    g_mcu.tasks.app_system_loop_task = NULL;
    vTaskDelete(g_mcu.tasks.app_monitor_task);
    g_mcu.tasks.app_monitor_task = NULL;
    vTaskDelete(g_mcu.tasks.app_data_client_task);
    g_mcu.tasks.app_data_client_task = NULL;
    // #if CONFIG_PROFILING_ENABLED
    //     vTaskDelete(g_mcu.tasks.app_log_trace_task);
    // #endif
}

/**
 * Stop all timers in case enter power save mode
**/
void sys_stop_timers(void) {
    xTimerStop(g_mcu.timers.discovery_timer, 0);
    xTimerStop(g_mcu.timers.time_sync_timer, 0);
    xTimerStop(g_mcu.timers.power_mgmt_timer, 0);
}

/**
 * Set system operation mode
 * @param active
 * @return
**/
esp_err_t sys_set_operation_mode(bool active) {
    g_mcu.state.active = active;
    set_sys_event(EV_SYS_MODE_CHANGE);
    return ESP_OK;

}

/**
 * Respond to the system mode change event
 * @param handler_args
 * @param base
 * @param id
 * @param event_data
**/
void sys_mode_change_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data) {
    if (g_mcu.state.active) {
        g_imu.p_imu->mux = IMU_MUX_STREAM;
    } else {
        g_imu.p_imu->mux = IMU_MUX_IDLE;
    }
}