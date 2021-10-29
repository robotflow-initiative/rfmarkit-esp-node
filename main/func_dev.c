#include "funcs.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "settings.h"
#include "events.h"
#include "gy95.h"
#include "main.h"

/* FreeRTOS event group to signal when we are connected*/
EventGroupHandle_t g_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */

static const char* TAG = "func_dev";

static int s_retry_num = CONFIG_ESP_MAXIMUM_RETRY;

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
            xEventGroupSetBits(g_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = CONFIG_ESP_MAXIMUM_RETRY;
        xEventGroupSetBits(g_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/**
 * @brief Default Wi-Fi STA connect function from esp-idf example
 * 
 */
esp_err_t wifi_init_sta(void) {
    g_wifi_event_group = xEventGroupCreate();

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

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by wifi_event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(g_wifi_event_group,
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

/**
 * @brief Enter sleep mode if tcp connection is not established
 *
 * @param nms
 */
static struct timeval now = { 0 };
static struct timeval sleep_enter_time = { 0 };
void esp_enter_light_sleep() {

    esp_sleep_enable_timer_wakeup(CONFIG_DISCONNECT_SLEEP_NUS);
    // esp_sleep_enable_gpio_wakeup();
    /* Enter sleep mode */
    ESP_LOGI(TAG, "Maximum retry. Going to sleep");
    gettimeofday(&sleep_enter_time, NULL);
    ESP_LOGI(TAG, "Disconnecting Wifi");
    ESP_ERROR_CHECK(esp_wifi_disconnect());
    ESP_LOGI(TAG, "Stopping Wifi");
    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_LOGI(TAG, "Disabling GY95");

    gy95_disable(&g_gy95_imu);
    xEventGroupClearBits(g_sys_event_group, GY95_CALIBRATED_BIT);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // TODO: Magic Delay

    /** Begin sleep **/
    esp_light_sleep_start();
    /** End sleep **/

    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

    /** Detect wake up cause **/
    switch (esp_sleep_get_wakeup_cause()) {
    case ESP_SLEEP_WAKEUP_EXT1: {
        ESP_LOGD(TAG, "Wake up from GPIO");
        break;
    }
    case ESP_SLEEP_WAKEUP_TIMER: { // Wake up by timer
        ESP_LOGD(TAG, "Wake up from timer. Time spent in deep sleep: %dms", sleep_time_ms);
        break;
    }
    case ESP_SLEEP_WAKEUP_UNDEFINED: // Normal boot
    default:
        ESP_LOGD(TAG, "Not a deep sleep reset");
    }

    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_connect();
    vTaskDelay(5000 / portTICK_PERIOD_MS); // TODO: Magic Delay
    EventBits_t bits = xEventGroupWaitBits(g_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGD(TAG, "Reconnected to ap ");
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGD(TAG, "Failed to reconnect");
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
    /* Execution continues here after wakeup */
}
void esp_enter_deep_sleep() {

    // esp_sleep_enable_gpio_wakeup();
    /* Enter sleep mode */
    ESP_LOGI(TAG, " Going to deep sleep (shutdown)");

    ESP_LOGI(TAG, "Disabling GY95");
    gy95_disable(&g_gy95_imu);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "GY95 ctrl_pin is set to %d", gpio_get_level(g_gy95_imu.ctrl_pin));

    gpio_hold_en(g_gy95_imu.ctrl_pin);
    ESP_LOGI(TAG, "Entering deep sleep (holding pin %d)\n", g_gy95_imu.ctrl_pin);

    vTaskDelay(1000 / portTICK_PERIOD_MS); // TODO: Magic Delay

    /** Begin deep sleep **/
    esp_deep_sleep_start();
    /** ESP shutdown **/

    
}

char g_device_id[14] = { 0 };
/**
 * @brief Read esp mac address from chip to g_device_id
 * 
**/
void esp_get_device_id() {
    uint8_t base_mac_addr[6];
    esp_efuse_mac_get_default(base_mac_addr);
    snprintf(g_device_id, 13, "%02x%02x%02x%02x%02x%02x",
             base_mac_addr[0],
             base_mac_addr[1],
             base_mac_addr[2],
             base_mac_addr[3],
             base_mac_addr[4],
             base_mac_addr[5]);
}

void uart_service_init(int port, int rx, int tx, int rts, int cts) {
    uart_config_t uart_config = {
        .baud_rate = 115200, // TODO: Magic baud_rate
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,

    };
    int intr_alloc_flags = 0;
    ESP_LOGI(TAG, "Initiate uart service at port %d, rx:%d, tx:%d", port, rx, tx);
    ESP_ERROR_CHECK(uart_driver_install(port, 512, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(port, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(port, tx, rx, rts, cts));
}
