//
// Created by liyutong on 2024/4/30.
//
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "sys.h"
#include "settings.h"

static const char *TAG = "sys.nvs_mgmt    ";

/** MCU vars **/
mcu_var_t g_mcu_vars[] = {
    {.name = CONFIG_NVS_WIFI_SSID_NAME, .type = VAR_STR},
    {.name = CONFIG_NVS_WIFI_PSK_NAME, .type = VAR_STR},
    {.name = CONFIG_NVS_DATA_HOST_NAME, .type=VAR_STR},
    {.name = CONFIG_NVS_OTA_HOST_NAME, .type=VAR_STR},
    {.name = CONFIG_NVS_NTP_HOST_NAME, .type=VAR_STR},
    {.name = CONFIG_NVS_TEST_NAME, .type =VAR_INT32},
    {.name = CONFIG_NVS_IMU_BAUD_NAME, .type=VAR_INT32},
    {.name = CONFIG_NVS_SEQ_NAME, .type=VAR_INT32},
    {.name = CONFIG_NVS_TARGET_FPS_NAME, .type=VAR_INT32},
};

static void sys_load_nvs_configuration(void);

/**
 * Initialize NVS function, run ONCE
 */
void sys_init_nvs() {
    /** Initialize NVS **/
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_flash_init();
    }
    sys_load_nvs_configuration();
}

/**
 * Find a variable by name
 * @param name
 * @param len
 * @return pointer to the variable
 */
mcu_var_t *sys_find_var(char *name, size_t len) {
    for (int idx = 0; idx < (sizeof(g_mcu_vars) / sizeof(mcu_var_t)); ++idx) {
        if (strncmp(name, g_mcu_vars[idx].name, MAX(strlen(g_mcu_vars[idx].name), len)) == 0) {
            return &g_mcu_vars[idx];
        }
    }
    return NULL;
}

/**
 * Save a variable in NVS
 * @param p_var
 * @param value
 * @return
**/
esp_err_t sys_set_nvs_var(mcu_var_t *p_var, char *value) {

    nvs_handle_t var_handle;
    ESP_ERROR_CHECK(nvs_open(CONFIG_VAR_NVS_TABLE_NAME, NVS_READWRITE, &var_handle));
    mcu_var_data_t data;

    switch (p_var->type) {
        case VAR_INT32:
            data.int32 = (int32_t) strtol(value, NULL, 10);
            nvs_set_i32(var_handle, p_var->name, data.int32);
            nvs_commit(var_handle);
            break;
        case VAR_FLOAT32:
            data.float32 = (float) strtold(value, NULL);
            nvs_set_blob(var_handle, p_var->name, &data.float32, sizeof(float));
            nvs_commit(var_handle);
            break;
        case VAR_STR:
            nvs_set_blob(var_handle, p_var->name, value, CONFIG_VAR_STR_MAX_LEN);
            nvs_commit(var_handle);
            break;
        case VAR_UINT8:
            data.uint8 = (uint8_t) strtol(value, NULL, 10);
            nvs_set_u8(var_handle, p_var->name, data.uint8);
            nvs_commit(var_handle);
            break;
    }
    nvs_close(var_handle);
    return ESP_OK;
}

/**
 *
 * @param p_var
 * @param out
 * @param value_buffer
 * @param len
 *
 * @warning value_buffer should be greater than CONFIG_VAR_STR_MAX_LEN
 * @return
 */
esp_err_t sys_get_nvs_var(mcu_var_t *p_var, mcu_var_data_t *out, char *value_buffer) {

    nvs_handle_t var_handle;
    ESP_ERROR_CHECK(nvs_open(CONFIG_VAR_NVS_TABLE_NAME, NVS_READWRITE, &var_handle));
    mcu_var_data_t data;
    size_t tmp;

    if (value_buffer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    switch (p_var->type) {
        case VAR_INT32:
            nvs_get_i32(var_handle, p_var->name, (void *) &data);
            snprintf(value_buffer, CONFIG_VAR_STR_MAX_LEN, "%d", (int32_t) data.int32);
            break;
        case VAR_FLOAT32:
            nvs_get_blob(var_handle, p_var->name, (void *) &data, NULL);
            snprintf(value_buffer, CONFIG_VAR_STR_MAX_LEN, "%f", (double) data.float32);
            break;
        case VAR_STR:
            nvs_get_blob(var_handle, p_var->name, value_buffer, &tmp);
            data.str = value_buffer;
            break;
        case VAR_UINT8:
            nvs_get_u8(var_handle, p_var->name, (void *) &data);
            snprintf(value_buffer, CONFIG_VAR_STR_MAX_LEN, "%d", (uint8_t) data.uint8);
            break;
    }

    if (out != NULL) {
        memcpy(out, &data, sizeof(data));
    }

    nvs_close(var_handle);
    return ESP_OK;
}


#define sys_load_str_conf(name, target, default_value) \
        p_var = sys_find_var((name), strlen(name)); \
        if (p_var != NULL) { \
            sys_get_nvs_var(p_var, &data, value_buffer); \
            if (strlen(value_buffer) > 0) { \
                memcpy((target), value_buffer,  MIN(sizeof((target)), CONFIG_VAR_STR_MAX_LEN)); \
            } else { \
                memcpy((target), (default_value), strlen((default_value))); \
            } \
            ESP_LOGW(TAG, "Variable "name"=%s; size(value_buffer)=%d", (target), strlen(target)); \
        } else { \
          ESP_LOGW(TAG, "Variable "name"=%s is not found", (target)); \
        } \
        p_var = NULL; \
        bzero(value_buffer, CONFIG_VAR_STR_MAX_LEN)

#define sys_load_int32_conf(name, target, default_value) \
        p_var = sys_find_var((name), strlen(name));\
        if (p_var != NULL) {\
            sys_get_nvs_var(p_var, &data, value_buffer);\
            if (data.int32 > 0) {\
                (target) = data.int32;\
            } else {\
                (target) = default_value;\
            }  \
            ESP_LOGW(TAG, "Variable "name"=%d; value_stored=%d", (target), data.int32); \
        } else { \
            ESP_LOGW(TAG, "Variable "name"=%d is not found", (target)); \
        } \
        p_var = NULL; \
        bzero(value_buffer, CONFIG_VAR_STR_MAX_LEN)

/**
 * Load NVS configuration to g_mcu variable, for only ONCE
**/
static void sys_load_nvs_configuration() {
    mcu_var_t *p_var = NULL;
    mcu_var_data_t data = {0};

    os_delay_ms(100);

    char value_buffer[CONFIG_VAR_STR_MAX_LEN];
    char _protect = 0;
    sys_load_int32_conf(CONFIG_NVS_TEST_NAME, _protect, 0); // NOTE: a workaround to protect the following variable from getting corrupted
    sys_load_str_conf(CONFIG_NVS_WIFI_SSID_NAME, g_mcu.wifi_ssid, CONFIG_WIFI_SSID);
    sys_load_str_conf(CONFIG_NVS_WIFI_PSK_NAME, g_mcu.wifi_psk, CONFIG_WIFI_PSK);
    sys_load_str_conf(CONFIG_NVS_DATA_HOST_NAME, g_mcu.data_host_ip_addr, CONFIG_DATA_HOST_IP_ADDR);
    sys_load_str_conf(CONFIG_NVS_OTA_HOST_NAME, g_mcu.ota_host, CONFIG_OTA_HOST);
    sys_load_str_conf(CONFIG_NVS_NTP_HOST_NAME, g_mcu.ntp_host_ip_addr, CONFIG_NTP_HOST_IP_ADDR);
    sys_load_int32_conf(CONFIG_NVS_IMU_BAUD_NAME, g_mcu.imu_baud, CONFIG_IMU_BAUD);
    sys_load_int32_conf(CONFIG_NVS_SEQ_NAME, g_mcu.seq, 0);
    sys_load_int32_conf(CONFIG_NVS_TARGET_FPS_NAME, g_mcu.target_fps, CONFIG_TARGET_FPS);


    /** Temporary Fix NVS invalid variable **/
    g_mcu.imu_baud = MIN(g_mcu.imu_baud, 921600);
    g_mcu.seq = MIN(g_mcu.seq, 255);
    g_mcu.seq = MAX(g_mcu.seq, 0);
    g_mcu.target_fps = MIN(g_mcu.target_fps, 200);

    /** Macro Expansion Reference, DO NOT REMOVE **/
    /**
    p_var = sys_find_var("IMU_BAUD", sizeof("IMU_BAUD"));
    if (p_var != NULL) {
        sys_get_nvs_var(p_var, &data, value_buffer);
        if (data.int32 > 0) {
            g_mcu.imu_baud = data.int32;
        } else {
            g_mcu.imu_baud = CONFIG_IMU_BAUD;
        }
    }
    ESP_LOGW(TAG, "Variable IMU_BAUD=%d;", g_mcu.imu_baud);
    p_var = NULL;
    bzero(value_buffer, CONFIG_VAR_STR_MAX_LEN);
    **/
}