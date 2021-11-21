#include "functions.h"
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
#include "esp_intr_alloc.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "settings.h"
#include "gy95.h"
#include "globals.h"

/* FreeRTOS event group to signal when we are connected*/


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

void esp_enter_deep_sleep() {

    // esp_sleep_enable_gpio_wakeup();
    /* Enter sleep mode */
    ESP_LOGI(TAG, " Going to deep sleep (shutdown)");

    ESP_LOGI(TAG, "Disabling GY95");
    gy95_disable(&g_imu);
    xEventGroupClearBits(g_sys_event_group, GY95_ENABLED_BIT);
    
    vTaskDelay(200 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "GY95 ctrl_pin is set to %d", gpio_get_level(g_imu.ctrl_pin));

    LED_ALLOFF();

    gpio_hold_en(g_imu.ctrl_pin);
    gpio_hold_en(CONFIG_BLINK_RED_PIN);
    gpio_hold_en(CONFIG_BLINK_GREEN_PIN);
    gpio_hold_en(CONFIG_BLINK_BLUE_PIN);

    ESP_LOGI(TAG, "Entering deep sleep (holding pin %d)\n", g_imu.ctrl_pin);

    /** If we donote disable wakeup source, then deep sleep will be waken **/
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TOUCHPAD);


    /** Begin deep sleep **/
    esp_deep_sleep_start();
    /** ESP shutdown **/

}

static void esp_enter_deep_sleep_from_isr(void* params) {
    g_sleep_countup += CONFIG_MAIN_LOOP_MAX_COUNT_NUM;
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

void esp_button_init() {
    /** Init GPIO **/
    gpio_config_t io_config = {
        .pin_bit_mask = (1ull << CONFIG_BUTTON_GPIO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io_config);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(CONFIG_BUTTON_GPIO_PIN, esp_enter_deep_sleep_from_isr, NULL);
}

void esp_self_test(char* tx_buffer, size_t tx_len) {

    ESP_LOGI(TAG, "Running self test");
    gy95_disable(&g_imu);
    esp_delay_ms(100);
    gy95_enable(&g_imu);
    esp_delay_ms(1000);
    while (1) {
        gy95_setup(&g_imu);
        esp_delay_ms(500);

    }
}