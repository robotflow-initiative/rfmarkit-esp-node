#include "apps.h"

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

#include "events.h"
#include "settings.h"

static const char* TAG = "app_time_sync";

void time_sync_notification_cb(struct timeval* tv) {
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

bool esp_wait_sync_time(const char* const posix_tz) {
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, CONFIG_NTP_SERVER_ADDR);
    sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_init();

    /** wait for time to be set **/
    int n_retry = 0;
    const int max_retry = 20;
    time_t now;
    struct tm timeinfo;
    char strftime_buf[64];

    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++n_retry < max_retry) {
        time(&now);
        localtime_r(&now, &timeinfo);
        strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
        ESP_LOGI(TAG, "The current date/time in Shanghai is: %s", strftime_buf); // TODO: Magic City Shanghai
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", n_retry, max_retry);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }


    setenv("TZ", posix_tz, 1);
    tzset();
    sntp_stop();

    return (n_retry < max_retry) ? true : false;
}

void app_time_sync(void* pvParameters) {
    ESP_LOGI(TAG, "app_time_sync started");
    xEventGroupClearBits(g_sys_event_group, NTP_SYNCED_BIT);

    time_t now;
    struct tm timeinfo;
    char strftime_buf[64];
    bool ret;
    int n_retry = CONFIG_NTP_MAX_RETRY;

    while (1) {
        /** Set timezone to China Standard Time **/
        ret = esp_wait_sync_time("CTS-8"); // TODO: Magic Timezone
        if (ret) {
            /** Update 'now' variable with current time **/
            time(&now);
            localtime_r(&now, &timeinfo);
            strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
            ESP_LOGI(TAG, "The current date/time in Shanghai is: %s", strftime_buf); // TODO: Magic City Shanghai

            /** Set event **/
            ESP_LOGI(TAG, "Setting NTP_SYNCED_BIT");
            xEventGroupSetBits(g_sys_event_group, NTP_SYNCED_BIT);
            /** Sleep **/
            vTaskDelay(CONFIG_NTP_UPDATE_INTERVAL_MS / portTICK_PERIOD_MS);
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




