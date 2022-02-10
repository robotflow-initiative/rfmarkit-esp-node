#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "esp_https_ota.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "sys.h"
#include "imu.h"
#include "sys.h"
#include "blink.h"
#include "settings.h"


static const char *TAG = "sys";

/** MCU vars **/
mcu_var_t g_mcu_vars[] = {
        {.name = "WIFI_SSID", .type = VAR_STR},
        {.name = "WIFI_PSK", .type = VAR_STR},
        {.name = "DATA_HOST", .type=VAR_STR},
        {.name = "OTA_HOST", .type=VAR_STR},
        {.name = "NTP_HOST", .type=VAR_STR},
        {.name ="TEST", .type =VAR_INT32},
        {.name = "IMU_BAUD", .type=VAR_INT32},
};

/** MCU structure **/
RTC_DATA_ATTR mcu_t
        g_mcu;



/** TCP Debug  related **/
#if CONFIG_EN_DEBUG_OVER_TCP
int g_debug_sock = -1;
char g_debug_buffer[TCP_DEBUG_BUFFER_LEN] = { 0 };
#endif

static int s_wifi_retry_num = CONFIG_ESP_MAXIMUM_RETRY;

/**
 * @brief Default Wi-Fi event handler from esp-idf example
 *
 * @param arg
 * @param event_base
 * @param event_id
 * @param event_data
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_wifi_retry_num > 0) {
            esp_wifi_connect();
            s_wifi_retry_num--;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(g_mcu.wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "got ip:"
                IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_retry_num = CONFIG_ESP_MAXIMUM_RETRY;
        xEventGroupSetBits(g_mcu.wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/**
 * @brief Default Wi-Fi STA connect function from esp-idf example
 *
 */
esp_err_t sys_wifi_init_sta(void) {
    g_mcu.wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
            .sta = {
                    .ssid = CONFIG_WIFI_SSID,
                    .password = CONFIG_WIFI_PSK,
                    /* Setting a password implies station will connect to all security modes including WEP/WPA.
                     * However these modes are deprecated and not advisable to be used. Incase your Access point
                     * doesn't support WPA2, these mode can be enabled by commenting below line */
                    .threshold.authmode = WIFI_AUTH_WPA2_PSK,

                    .pmf_cfg = {
                            .capable = true,
                            .required = false
                    },
            },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "sys_wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by wifi_event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(g_mcu.wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 CONFIG_WIFI_SSID, CONFIG_WIFI_PSK);
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 CONFIG_WIFI_SSID, CONFIG_WIFI_PSK);
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
        return ESP_FAIL;
    }

    /** @remark Original example Disables wifi event group, we dont since we need it to reconnect after resumming from light sleep**/
    /* The event will not be processed after unregister */
    // ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    // ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    // vEventGroupDelete(g_wifi_event_group);
}

static void sys_enter_deep_sleep_from_isr(void *params) {
    g_mcu.sleep_countup += CONFIG_MAIN_LOOP_MAX_LOOP_NUM;
}

/**
 * @brief Read esp mac address from chip to g_mcu.device_id
 *
**/
void sys_get_device_id() {
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
    memcpy(g_mcu.device_id, buf, 12);
}

void sys_button_init(int pin) {
    /** Init GPIO **/
    gpio_config_t io_config = {
            .pin_bit_mask = (1ull << pin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io_config);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(pin, sys_enter_deep_sleep_from_isr, NULL);
}


esp_err_t sys_do_ota() {
    char url[128] = {0};
    snprintf(url, sizeof(url), "http://%s:%d/firmware.bin", g_mcu.ota_host_ip_addr, CONFIG_OTA_PORT);
    esp_http_client_config_t config = {
            .url = url,
            .max_authorization_retries = 3,
            .auth_type = HTTP_AUTH_TYPE_NONE,
    };
    esp_err_t ret = esp_https_ota(&config);
    if (ret == ESP_OK) {
        esp_restart();
    } else {
        ESP_LOGI(TAG, "OTA update error, return code: %d", (int) ret);
        return ESP_FAIL;
    }
}

void sys_log_chip_info() {
    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI("", "\n\n\n\n\n\n# -------- Begin of app log -------- #");
    ESP_LOGI(TAG, "This is %s chip with %d CPU core(s), WiFi%s%s, ",
             CONFIG_IDF_TARGET,
             chip_info.cores,
             (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    ESP_LOGI(TAG, "Silicon revision %d, ", chip_info.revision);

    ESP_LOGI(TAG, "%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
             (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    ESP_LOGW(TAG, "Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
    ++g_mcu.boot_count;
    ESP_LOGI(TAG, "Boot count %d", g_mcu.boot_count);
    sys_get_device_id();
    ESP_LOGI(TAG, "Device ID: %s", g_mcu.device_id);

    ESP_LOGW(TAG, "\n-------VERSION-------\nv%s\n---------END---------", CONFIG_FIRMWARE_VERSION);
}

void sys_reset_gpio(int pin) {
    /** Cancel hold **/
    gpio_hold_dis(pin);
    gpio_deep_sleep_hold_dis();
    /** Enable gpio hold in deep sleep **/
    gpio_deep_sleep_hold_en();
}


static mcu_var_t *device_find_var(char *name, size_t len) {
    for (int idx = 0; idx < (sizeof(g_mcu_vars) / sizeof(mcu_var_t)); ++idx) {
        if (strncmp(name, g_mcu_vars[idx].name, len) == 0) {
            return &g_mcu_vars[idx];
        }
    }
    return NULL;
}


#define sys_load_str_conf(name, target, default_value) \
        p_var = device_find_var((name), strlen(name)); \
        if (p_var != NULL) { \
            device_get_nvs_var(p_var, &data, value_buffer); \
            if (strlen(value_buffer) > 0) { \
                memcpy((target), value_buffer,  MIN(sizeof((target)), CONFIG_VAR_STR_MAX_LEN)); \
            } else { \
                memcpy((target), (default_value), strlen((default_value))); \
            } \
            ESP_LOGW(TAG, "Variable "name"=%s; size(value_buffer)=%d", (target), strlen(value_buffer)); \
        } else { \
          ESP_LOGW(TAG, "Variable "name"=%s is not found", (target)); \
        } \
        p_var = NULL; \
        bzero(value_buffer, CONFIG_VAR_STR_MAX_LEN)

#define sys_load_int32_conf(name, target, default_value) \
        p_var = device_find_var((name), strlen(name));\
        if (p_var != NULL) {\
            device_get_nvs_var(p_var, &data, value_buffer);\
            if (data.int32 > 0) {\
                (target) = data.int32;\
            } else {\
                (target) = default_value;\
            }  \
            ESP_LOGW(TAG, "Variable"name"=%d; value_stored=%d", (target), data.int32); \
        } else { \
            ESP_LOGW(TAG, "Variable "name"=%s is not found", (target)); \
        } \
        p_var = NULL; \
        bzero(value_buffer, CONFIG_VAR_STR_MAX_LEN)

void sys_load_configuration() {
    mcu_var_t *p_var = NULL;
    mcu_var_data_t data = {0};
    char value_buffer[CONFIG_VAR_STR_MAX_LEN];

    sys_load_str_conf("WIFI_SSID", g_mcu.wifi_ssid, CONFIG_WIFI_SSID);
    sys_load_str_conf("WIFI_PSK", g_mcu.wifi_psk, CONFIG_WIFI_PSK);
    sys_load_str_conf("DATA_HOST", g_mcu.data_host_ip_addr, CONFIG_DATA_HOST_IP_ADDR);
    sys_load_str_conf("OTA_HOST", g_mcu.ota_host_ip_addr, CONFIG_OTA_HOST_IP_ADDR);
    sys_load_str_conf("NTP_HOST", g_mcu.ntp_host_ip_addr, CONFIG_NTP_HOST_IP_ADDR);

    p_var = device_find_var("IMU_BAUD", sizeof("IMU_BAUD"));
    if (p_var != NULL) {
        device_get_nvs_var(p_var, &data, value_buffer);
        if (data.int32 > 0) {
            g_mcu.imu_baud = data.int32;
        } else {
            g_mcu.imu_baud = CONFIG_IMU_BAUD;
        }
    }
    ESP_LOGW(TAG, "Variable IMU_BAUD=%d;", g_mcu.imu_baud);
    p_var = NULL;
    bzero(value_buffer, CONFIG_VAR_STR_MAX_LEN);

}

esp_err_t device_set_nvs_var(mcu_var_t *p_var, char *value) {

    nvs_handle_t var_handle;
    ESP_ERROR_CHECK(nvs_open(CONFIG_VAR_NVS_TABLE_NAME, NVS_READWRITE, &var_handle));
    mcu_var_data_t data = {0};

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
 * @warning value_buffer should be greateer than CONFIG_VAR_STR_MAX_LEN
 * @return 
 */
esp_err_t device_get_nvs_var(mcu_var_t *p_var, mcu_var_data_t *out, char *value_buffer) {

    nvs_handle_t var_handle;
    ESP_ERROR_CHECK(nvs_open(CONFIG_VAR_NVS_TABLE_NAME, NVS_READWRITE, &var_handle));
    mcu_var_data_t data = {0};
    size_t tmp;

    switch (p_var->type) {
        case VAR_INT32:
            nvs_get_i32(var_handle, p_var->name, (void *) &data);
            snprintf(value_buffer, CONFIG_VAR_STR_MAX_LEN, "%d\n\n", (int32_t) data.int32);
            break;
        case VAR_FLOAT32:
            nvs_get_blob(var_handle, p_var->name, (void *) &data, NULL);
            snprintf(value_buffer, CONFIG_VAR_STR_MAX_LEN, "%f\n\n", (double) data.float32);
            break;
        case VAR_STR:
            nvs_get_blob(var_handle, p_var->name, value_buffer, &tmp);
            ESP_LOGD(TAG, "%s:%p:%d", p_var->name, value_buffer, strlen(value_buffer));
            if (value_buffer != NULL) {
                data.str = value_buffer;
            }
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


void sys_enter_deep_sleep() {

    /* Enter sleep mode */
    ESP_LOGI(TAG, " Going to deep sleep (shutdown)");

    ESP_LOGI(TAG, "Disabling IMU");
    imu_disable(&g_imu);
    xEventGroupClearBits(g_mcu.sys_event_group, EV_IMU_ENABLED_BIT);
    clear_sys_event(EV_IMU_ENABLED);

    os_delay_ms(200);
    ESP_LOGI(TAG, "IMU ctrl_pin is set to %d", gpio_get_level(g_imu.ctrl_pin));

    CONFIG_LED_OFF();

    gpio_hold_en(g_imu.ctrl_pin);
    gpio_hold_en(CONFIG_BLINK_PIN);

    ESP_LOGI(TAG, "Entering deep sleep (holding pin %d)\n", g_imu.ctrl_pin);

    /** If we donote disable wakeup source, then deep sleep will be waken **/
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TOUCHPAD);


    /** Begin deep sleep **/
    esp_deep_sleep_start();
    /** ESP shutdown **/

}


COMMAND_FUNCTION(restart) {
    ESP_LOGI(TAG, "Executing command : IMU_RESTART");
    esp_restart();
    return ESP_OK;
}

COMMAND_FUNCTION(ping) {
    ESP_LOGI(TAG, "Executing command : IMU_PING");
    return ESP_OK;
}

COMMAND_FUNCTION(update) {
    ESP_LOGI(TAG, "Executing command : IMU_UPDATE");
    set_sys_event(EV_UART_MANUAL_BLOCK);
    esp_err_t ret = sys_do_ota();
    return ret;
}

COMMAND_FUNCTION(start) {
    ESP_LOGI(TAG, "Executing command : IMU_START");
    clear_sys_event(EV_UART_MANUAL_BLOCK);
    EventBits_t bits = xEventGroupWaitBits(g_mcu.sys_event_group, EV_UART_ACTIVATED_BIT, pdFALSE, pdFALSE, CONFIG_MAIN_LOOP_DUTY_PERIOD_MS / portTICK_PERIOD_MS);
    if (bits & EV_UART_ACTIVATED_BIT) {
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}

COMMAND_FUNCTION(stop) {
    ESP_LOGI(TAG, "Executing command : IMU_STOP");
    set_sys_event(EV_UART_MANUAL_BLOCK);
    return ESP_OK;
}


COMMAND_FUNCTION(id) {
    ESP_LOGI(TAG, "Executing command : IMU_ID");
    snprintf(tx_buffer, tx_len, "%s\n\n", g_mcu.device_id);
    return ESP_OK;
}

COMMAND_FUNCTION(ver) {
    ESP_LOGI(TAG, "Executing command : IMU_VER");
    snprintf(tx_buffer, tx_len, "%s\n\n", CONFIG_FIRMWARE_VERSION);
    return ESP_OK;
}

COMMAND_FUNCTION(time) {
    ESP_LOGI(TAG, "Executing command : IMU_TIME");
    struct timeval tv_now = {0};
    gettimeofday(&tv_now, NULL);
    int64_t time = (int64_t) tv_now.tv_sec * 1000000L + (int64_t) tv_now.tv_usec;
    snprintf(tx_buffer, tx_len, "{\"timestamp\": %f}\n\n", (((double) time) / 1e6));
    return ESP_OK;
}

COMMAND_FUNCTION(always_on) {
    ESP_LOGI(TAG, "Executing command : IMU_ALWAYS_ON");

    g_mcu.sleep_countup = INT32_MIN;

    return ESP_OK;
}

COMMAND_FUNCTION(varset) {
    ESP_LOGI(TAG, "Executing command : IMU_VAR_SET");

    mcu_var_t *p_var = NULL;
    char *var_name = NULL;
    size_t var_name_len = 0;
    char *var_value = NULL;
    size_t var_value_len = 0;


    for (int idx = 6; idx < rx_len; ++idx) {
        if (rx_buffer[idx] != ' ' && !var_name) {
            var_name = &rx_buffer[idx];
        }
        if (rx_buffer[idx] == '=' && !var_value) {
            var_name_len = &rx_buffer[idx] - var_name;
            var_value = &rx_buffer[idx + 1];
        }
        if ((rx_buffer[idx] == ' ' || rx_buffer[idx] == '\n' || rx_buffer[idx] == 0) && var_value) {
            var_value_len = (&rx_buffer[idx] - var_value);
            ESP_LOGD(TAG, "&rx_buffer[idx]: %p, var_value: %p", &rx_buffer[idx], var_value);
            rx_buffer[idx] = '\0';
            break;
        }
    }
    ESP_LOGD(TAG, "Set %s, len: %d", var_value, var_value_len);
    if (var_value && var_value_len <= 0) {
        var_value_len = rx_len - (var_value - rx_buffer);
    }
    if (var_name && var_name_len > 0 && var_value && var_value_len > 0) {
        p_var = device_find_var(var_name, var_name_len);
    } else {
        p_var = NULL;
    }

    if (p_var) {
        device_set_nvs_var(p_var, var_value);
        ESP_LOGD(TAG, "Set %s, len: %d", var_value, var_value_len);
        return ESP_OK;
    } else {
        snprintf(tx_buffer, tx_len, "Cannot set value for \"%s\", len: %d; value %s, len : %d\n\n", var_name, var_name_len, var_value, var_value_len);
        return ESP_FAIL;
    }
    
    // FIXME: Wrong format (e.g.: "varset DATA_HOST 10.53.25.21") cause reboot
}

COMMAND_FUNCTION(varget) {
    ESP_LOGI(TAG, "Executing command : IMU_VAR_GET");

    mcu_var_t *p_var = NULL;
    char *var_name = NULL;
    size_t var_name_len = 0;

    for (int idx = 6; idx < rx_len; ++idx) {
        if (rx_buffer[idx] != ' ' && !var_name) {
            var_name = &rx_buffer[idx];
        }
        if ((rx_buffer[idx] == '?' || rx_buffer[idx] == '\n') && var_name) {
            var_name_len = &rx_buffer[idx] - var_name;
            break;
        }
    }

    if (var_name && var_name_len > 0) {
        p_var = device_find_var(var_name, var_name_len);
    } else {
        p_var = NULL;
    }


    if (p_var) {
        mcu_var_data_t data;
        char value_buffer[CONFIG_VAR_STR_MAX_LEN] = {0};
        device_get_nvs_var(p_var, &data, value_buffer);
        memcpy(tx_buffer, value_buffer, MIN(tx_len, CONFIG_VAR_STR_MAX_LEN));
        return ESP_OK;
    } else {
        snprintf(tx_buffer, tx_len, "Cannot get value for \"%s\", len: %d\n\n", var_name, var_name_len);
        return ESP_FAIL;
    }
}

COMMAND_FUNCTION(shutdown) {
    ESP_LOGI(TAG, "Executing command : IMU_SHUTDOWN");
    sys_enter_deep_sleep();
    return ESP_OK;
}