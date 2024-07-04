//
// Created by liyutong on 2024/4/28.
//
#include <string.h>

#include "esp_wifi_types.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "sys.h"

static const char *TAG_HANDLER = "wifi_event_handler";
static int s_wifi_retry_num = CONFIG_WIFI_MAX_RETRY;
static SemaphoreHandle_t sync_mutex = NULL;

static const char *TAG = "sys.wifi        ";

/**
 * @brief Default Wi-Fi event handler from esp-idf example
 *
 * @param arg
 * @param event_base
 * @param event_id
 * @param event_data
**/
static void wifi_event_handler(void *arg, const char *event_base,
                               __int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        s_wifi_retry_num = CONFIG_ESP_MAXIMUM_RETRY;
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_wifi_retry_num > 0) {
            esp_wifi_connect();
            s_wifi_retry_num--;
            ESP_LOGI(TAG_HANDLER, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(g_mcu.wifi_event_group, EV_WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG_HANDLER, "connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG_HANDLER, "got ip:"
            IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(g_mcu.wifi_event_group, EV_WIFI_CONNECTED_BIT);
        /** Configure the ip_info used for BLE service **/
        g_mcu.ip_info = event->ip_info;
    }
}

/**
 * @brief Default Wi-Fi netif init function from esp-idf example plus hostname setting
**/
void sys_wifi_netif_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_t *interface = esp_netif_create_default_wifi_sta();
    /** Set the hostname **/
    ESP_ERROR_CHECK(esp_netif_set_hostname(interface, g_mcu.ble_local_name));
}

/**
 * @brief Default Wi-Fi STA connect function from esp-idf example
 *
**/
esp_err_t sys_wifi_msp_init(void) {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
                                               ESP_EVENT_ANY_ID,
                                               &wifi_event_handler,
                                               NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,
                                               IP_EVENT_STA_GOT_IP,
                                               &wifi_event_handler,
                                               NULL));

    /** Configure Wi-Fi mode **/
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    ESP_LOGI(TAG, "sys_wifi_msp_init finished.");
    return ESP_OK;
}

esp_err_t sys_wifi_msp_deinit(void) {
    sys_wifi_try_disconnect();
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler));
    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_wifi_deinit());
    ESP_LOGI(TAG, "sys_wifi_msp_deinit finished.");
    return ESP_OK;
}

/**
 * @brief Default Wi-Fi STA connect function from esp-idf example
 *
 * @param wifi_config
**/
esp_err_t sys_wifi_try_connect(wifi_config_t *wifi_config) {
    xEventGroupClearBits(g_mcu.wifi_event_group, EV_WIFI_CONNECTED_BIT | EV_WIFI_FAIL_BIT);

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    /**
     * Waiting until either the connection is established (EV_WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (EV_WIFI_FAIL_BIT). The bits are set by wifi_event_handler() (see above)
    **/
    TickType_t bits = xEventGroupWaitBits(g_mcu.wifi_event_group,
                                          EV_WIFI_CONNECTED_BIT | EV_WIFI_FAIL_BIT,
                                          pdFALSE,
                                          pdFALSE,
                                          portMAX_DELAY);

    /**
     * xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened.
    **/
    if (bits & EV_WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 wifi_config->sta.ssid, wifi_config->sta.password);
        g_mcu.state.wifi_connected = true;
        set_sys_event(EV_SYS_WIFI_CONNECTED);
        return ESP_OK;
    } else if (bits & EV_WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "failed to connect to SSID:%s, password:%s",
                 wifi_config->sta.ssid, wifi_config->sta.password);
        g_mcu.state.wifi_connected = false;
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "unexpected event");
        g_mcu.state.wifi_connected = false;
        return ESP_FAIL;
    }
}

/**
 * @brief Default Wi-Fi STA disconnect function from esp-idf example
**/
esp_err_t sys_wifi_try_disconnect() {
    esp_wifi_disconnect();
    esp_wifi_stop();
    g_mcu.state.wifi_connected = false;
    set_sys_event(EV_SYS_WIFI_DISCONNECTED);
    ESP_LOGI(TAG, "disconnected");
    return ESP_OK;
}

/**
 * Prepare Wi-Fi configuration
 * @param wifi_config
 * @param ssid
 * @param psk
**/
esp_err_t sys_wifi_prepare_config(wifi_config_t *wifi_config, const char *ssid, const char *psk) {
    /** In case of multiple attempts, we need to clear the previous configuration **/
    memset(wifi_config, 0, sizeof(wifi_config_t));

    strcpy((char *) (wifi_config->sta.ssid), ssid);
    strcpy((char *) (wifi_config->sta.password), psk);
    /**
     * Setting a password implies station will connect to all security modes including WEP/WPA.
     * However these modes are deprecated and not advisable to be used. Incase your Access point
     * doesn't support WPA2, these mode can be enabled by commenting below line
    **/
    wifi_config->sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config->sta.pmf_cfg.capable = true;
    wifi_config->sta.pmf_cfg.required = false;
    return ESP_OK;
}

/**
 * @brief Wi-Fi reconnect event handler
 *
 * @param handler_args
 * @param base
 * @param id
 * @param event_data
**/
void sys_wifi_reconnect_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data) {
    if (!sync_mutex) {
        sync_mutex = xSemaphoreCreateMutex();
    }
    if (xSemaphoreTake(sync_mutex, portMAX_DELAY) == pdTRUE) {
        sys_wifi_try_disconnect();
        wifi_config_t wifi_config = {0};
        ESP_ERROR_CHECK(sys_wifi_prepare_config(&wifi_config, g_mcu.wifi_ssid, g_mcu.wifi_psk));
        ESP_LOGI(TAG, "try to connect to %s with %s", g_mcu.wifi_ssid, g_mcu.wifi_psk);

        int ret = sys_wifi_try_connect(&wifi_config);
        ESP_LOGI(TAG, "connected=%s", (ret == ESP_OK) ? "true" : "false");
        if (ret != ESP_OK) {
            set_sys_event(EV_SYS_WIFI_FAIL);
        } else {
            clear_sys_event(EV_SYS_WIFI_FAIL);
        }

        clear_task_event(EV_TASK_WIFI_RECONNECT);
        xSemaphoreGive(sync_mutex);
    }
}