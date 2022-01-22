#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "esp_https_ota.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "blink.h"
#include "device.h"
#include "settings.h"

/* FreeRTOS event group to signal when we are connected*/


/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */

static const char* TAG = "device";

static int s_retry_num = CONFIG_ESP_MAXIMUM_RETRY;

/** Socket-Debug related **/
#if CONFIG_EN_DEBUG_OVER_TCP
int g_debug_sock = -1;
char g_debug_buffer[TCP_DEBUG_BUFFER_LEN] = { 0 };
#endif

/** MCU structure **/
RTC_DATA_ATTR mcu_t
g_mcu;

/**
 * @brief Default Wi-Fi event handler from esp-idf example
 *
 * @param arg
 * @param event_base
 * @param event_id
 * @param event_data
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num > 0) {
            esp_wifi_connect();
            s_retry_num--;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(g_mcu.wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "got ip:"
                 IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = CONFIG_ESP_MAXIMUM_RETRY;
        xEventGroupSetBits(g_mcu.wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/**
 * @brief Default Wi-Fi STA connect function from esp-idf example
 *
 */
esp_err_t device_wifi_init_sta(void) {
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
                    .ssid = CONFIG_ESP_WIFI_SSID,
                    .password = CONFIG_ESP_WIFI_PASSWORD,
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

    ESP_LOGI(TAG, "device_wifi_init_sta finished.");

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
                 CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD);
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD);
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

static void sys_enter_deep_sleep_from_isr(void* params) {
    g_mcu.sleep_countup += CONFIG_MAIN_LOOP_MAX_LOOP_NUM;
}

/**
 * @brief Read esp mac address from chip to g_mcu.device_id
 *
**/
void device_get_device_id() {
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

void device_button_init(int pin) {
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


esp_err_t device_do_ota() {
    esp_http_client_config_t config = {
            .url = CONFIG_OTA_APIHOST,
            .max_authorization_retries = CONFIG_OTA_MAXIMUM_RETRY,
            .auth_type = HTTP_AUTH_TYPE_NONE,
    };
    esp_err_t ret = esp_https_ota(&config);
    if (ret == ESP_OK) {
        esp_restart();
    } else {
        ESP_LOGI(TAG, "OTA update error, return code: %d", (int)ret);
        return ESP_FAIL;
    }
    return ESP_OK;
}

void device_log_chip_info() {
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
    device_get_device_id();
    ESP_LOGI(TAG, "Device ID: %s", g_mcu.device_id);

    ESP_LOGW(TAG, "\n-------VERSION-------\nv%s\n---------END---------", CONFIG_FIRMWARE_VERSION);
}

void device_reset_gpio(int pin) {
    /** Cancel hold **/
    gpio_hold_dis(pin);
    gpio_deep_sleep_hold_dis();
    /** Enable gpio hold in deep sleep **/
    gpio_deep_sleep_hold_en();
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
    set_sys_event(UART_BLOCK);
    esp_err_t ret = device_do_ota();
    return ret;
}

COMMAND_FUNCTION(start) {
    ESP_LOGI(TAG, "Executing command : IMU_START");
    clear_sys_event(UART_BLOCK);
    EventBits_t bits = xEventGroupWaitBits(g_mcu.sys_event_group, UART_ACTIVE_BIT, pdFALSE, pdFALSE, CONFIG_MAIN_LOOP_DUTY_PERIOD_MS / portTICK_PERIOD_MS);
    if (bits & UART_ACTIVE_BIT) {
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}

COMMAND_FUNCTION(stop) {
    ESP_LOGI(TAG, "Executing command : IMU_STOP");
    set_sys_event(UART_BLOCK);
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


COMMAND_FUNCTION(always_on) {
    ESP_LOGI(TAG, "Executing command : IMU_ALWAYS_ON");

    g_mcu.sleep_countup = INT32_MIN;

    return ESP_OK;
}


static mcu_var_t s_mcu_vars[] = {
        {.name = "WIFI_SSID", .type = VAR_STR},
        {.name = "WIFI_PSK", .type = VAR_STR},
        {.name = "BLINK_SEQ", .type = VAR_STR},

};

static mcu_var_t* device_find_var(char* name, size_t len) {
    for (int idx = 0; idx < (sizeof(s_mcu_vars) / sizeof(mcu_var_t)); ++idx) {
        if (strncmp(name, s_mcu_vars[idx].name, len) == 0) {
            return &s_mcu_vars[idx];
        }
    }
    return NULL;
}
static esp_err_t device_set_nvs_var(mcu_var_t* p_var, char* value, size_t len) {

    if (p_var == NULL) {
        return ESP_FAIL;
    }
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
        nvs_set_str(var_handle, p_var->name, value);
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

static esp_err_t device_get_nvs_var(mcu_var_t* p_var, mcu_var_data_t * out, char* value_buffer, size_t len) {

    if (p_var == NULL) {
        return ESP_FAIL;
    }
    nvs_handle_t var_handle;
    ESP_ERROR_CHECK(nvs_open(CONFIG_VAR_NVS_TABLE_NAME, NVS_READWRITE, &var_handle));
    mcu_var_data_t data = {0};

    switch (p_var->type) {
    case VAR_INT32:
        nvs_get_i32(var_handle, p_var->name, (void *)&data);
        snprintf(value_buffer, len, "%d", (int32_t)data.int32);
        break;
    case VAR_FLOAT32:
        nvs_get_blob(var_handle, p_var->name, (void *)&data, NULL);
        snprintf(value_buffer, len, "%f", (double)data.float32);
        break;
    case VAR_STR:
        nvs_get_str(var_handle, p_var->name, value_buffer, NULL);
        break;
    case VAR_UINT8:
        nvs_get_u8(var_handle, p_var->name, (void *)&data);
        snprintf(value_buffer, len, "%d", (uint8_t)data.uint8);
        break;
    }

    if (out != NULL) {
        memcpy(out, &data, sizeof(data));
    }

    nvs_close(var_handle);
    return ESP_OK;
}


COMMAND_FUNCTION(var_set) {
    ESP_LOGI(TAG, "Executing command : IMU_VAR_SET");

    mcu_var_t* p_var = NULL;
    char* var_name = NULL;
    size_t var_name_len = 0;
    char* var_value = NULL;
    size_t var_value_len = 0;

    {
        for (int idx = 7; idx < rx_len; ++idx) {
            if (rx_buffer[idx] != ' ' && !var_name) {
                var_name = &rx_buffer[idx];
            }
            if (rx_buffer[idx] == '=' && !var_value) {
                var_name_len = &rx_buffer[idx] - var_name;
                var_value = &rx_buffer[idx + 1];
            }
            if ((rx_buffer[idx] == ' ' || rx_buffer[idx] == '\n' || rx_buffer[idx] == 0) && var_value) {
                var_value_len = &rx_buffer[idx] - var_value;
            }
        }
        if (var_value && var_value_len <= 0) {
            var_value_len = rx_len - (var_value - rx_buffer);
        }
        if (var_name && var_name_len > 0 && var_value && var_value_len > 0) {
            p_var = device_find_var(var_name, var_name_len);
        } else {
            p_var = NULL;
        }
    }

    if (p_var) {
        device_set_nvs_var(p_var, var_value, var_value_len);
        return ESP_OK;
    } else {
        snprintf(tx_buffer, tx_len, "Cannot set value for \"%s\"", rx_buffer);
        return ESP_FAIL;
    }
}

COMMAND_FUNCTION(var_get) { // TODO: Finish host set 
    ESP_LOGI(TAG, "Executing command : IMU_VAR_GET");

    ESP_LOGI(TAG, "Executing command : IMU_VAR_SET");
    
    mcu_var_t* p_var = NULL;
    char* var_name = NULL;
    size_t var_name_len = 0;

    {
        for (int idx = 7; idx < rx_len; ++idx) {
            if (rx_buffer[idx] != ' ' && !var_name) {
                var_name = &rx_buffer[idx];
            }
            if (rx_buffer[idx] == '?' && !var_name) {
                var_name_len = &rx_buffer[idx] - var_name;
            }
        }

        if (var_name && var_name_len > 0) {
            p_var = device_find_var(var_name, var_name_len);
        } else {
            p_var = NULL;
        }
    }

    if (p_var) {
        mcu_var_data_t data;
        device_get_nvs_var(p_var, &data, tx_buffer, tx_len);
        return ESP_OK;
    } else {
        snprintf(tx_buffer, tx_len, "Cannot getvalue for \"%s\"", rx_buffer);
        return ESP_FAIL;
    }


    return ESP_FAIL;
}