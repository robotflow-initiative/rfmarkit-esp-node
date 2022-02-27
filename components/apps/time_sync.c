#include <string.h>
#include <time.h>
#include <stdbool.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sntp.h"

#include "apps.h"
#include "sys.h"
#include "settings.h"

static const char *TAG = "app_time_sync";

void time_sync_notification_cb(struct timeval *tv) {
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

#define STRTIME_BUF_LEN 64
#define fill_strtime_buf(buf) \
    { \
        time_t now; \
        struct tm timeinfo; \
        time(&now); \
        localtime_r(&now, &timeinfo); \
        strftime(buf, sizeof(buf), "%c", &timeinfo); \
        ESP_LOGI(TAG, "The current date/time in "CONFIG_LOCAL_TZ" is: %s", buf); \
    }

static esp_err_t device_wait_sync_time(const char *const posix_tz) {
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    ESP_LOGD(TAG, "g_mcu.ntp_host_ip_addr=%s", g_mcu.ntp_host_ip_addr);
    sntp_setservername(0, g_mcu.ntp_host_ip_addr);
    sntp_setservername(1, CONFIG_NTP_HOST_IP_ADDR_BACKUP);


    sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_init();

    /** wait for time to be set **/
    int n_retry = 0;
    char strftime_buf[STRTIME_BUF_LEN];

    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++n_retry < CONFIG_NTP_TIMEOUT_S) {
        fill_strtime_buf(strftime_buf);
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", n_retry, CONFIG_NTP_TIMEOUT_S);
        os_delay_ms(1000);
    }

    setenv("TZ", posix_tz, 1);
    tzset();
    sntp_stop();

    return (n_retry < CONFIG_NTP_TIMEOUT_S) ? ESP_OK : ESP_FAIL;
}

void app_time_sync(void *pvParameters) {
    ESP_LOGI(TAG, "app_time_sync started");
    clear_sys_event(EV_NTP_SYNCED);

    char strftime_buf[STRTIME_BUF_LEN];
    esp_err_t ret;
    int n_retry = CONFIG_NTP_MAX_RETRY;

    while (1) {
        /** Set timezone to China Standard Time **/
        ret = device_wait_sync_time(CONFIG_LOCAL_TZ);
        if (ret == ESP_OK) {
            /** Update 'now' variable with current time **/
            fill_strtime_buf(strftime_buf);

            /** Set event **/
            ESP_LOGI(TAG, "Setting NTP_SYNCED_BIT");
            set_sys_event(EV_NTP_SYNCED);
            /** Sleep **/
            os_delay_ms(CONFIG_NTP_UPDATE_INTERVAL_MS);
            n_retry = CONFIG_NTP_MAX_RETRY;
        } else {
            ESP_LOGW(TAG, "Previous time sync failed");
            --n_retry;
        }
        if (n_retry <= 0) {
            esp_restart();
        }
    }

    vTaskDelete(NULL);
}
