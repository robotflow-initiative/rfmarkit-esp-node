//
// Created by liyutong on 2024/4/30.
//
#include <stdlib.h>

#include <lwip/apps/sntp.h>

#include "esp_log.h"
#include "esp_sntp.h"

#include "apps.h"
#include "settings.h"
#include "sys.h"

static SemaphoreHandle_t sync_mutex = NULL;
static int s_sntp_retry_num = 0;

static const char *TAG = "sys.time        ";

/**
 * Time sync notification callback
 * @param tv
**/
void time_sync_notification_cb(struct timeval *tv) {
    ESP_LOGI(TAG, "notification of a time synchronization event, unix time %ld|%ld", tv->tv_sec, tv->tv_usec);
    g_mcu.state.ntp_synced = true;
    set_sys_event(EV_SYS_NTP_SYNCED); // notify the system that the time is synced
    get_time_usec(g_mcu.state.ntp_sync_success_unix_usec); // set the time of the last successful sync
}

/**
 * Time sync handler
 * @param handler_args
 * @param base
 * @param id
 * @param event_data
**/
void sys_time_sync_handler(
    __attribute__((unused)) void *handler_args,
    __attribute__((unused)) esp_event_base_t base,
    __attribute__((unused)) int32_t id,
    __attribute__((unused)) void *event_data
) {
    /** @note: This function is VERY CPU intensive, and should be called only when necessary **/
    if (!sync_mutex) {
        sync_mutex = xSemaphoreCreateMutex();
    }
    if (xSemaphoreTake(sync_mutex, portMAX_DELAY) == pdTRUE) {
        ESP_LOGI(TAG, "time_sync event handled");
        setenv("TZ", CONFIG_LOCAL_TZ, 1);
        tzset();

        /** Configure **/
        sntp_setoperatingmode(SNTP_OPMODE_POLL);
        if (s_sntp_retry_num <= CONFIG_NTP_MAX_RETRY) {
            sntp_setservername(0, g_mcu.ntp_host_ip_addr);
            ESP_LOGI(TAG, "primary sntp server is set to %s", g_mcu.ntp_host_ip_addr);
        } else {
            sntp_setservername(0, CONFIG_NTP_HOST_IP_ADDR_BACKUP);
            ESP_LOGI(TAG, "backup sntp server is set to %s", CONFIG_NTP_HOST_IP_ADDR_BACKUP);
        }

        sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
        sntp_set_time_sync_notification_cb(time_sync_notification_cb);

        /** Start and Wait **/
        sntp_init();
        int n_retry = 0;
        while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++n_retry < CONFIG_NTP_TIMEOUT_S) {
            os_delay_ms(1000);
        }
        sntp_stop();

        ESP_LOGI(TAG, "previous time sync ended with %d tries, sntp_get_sync_status()= %d", n_retry, sntp_get_sync_status());

        if (n_retry >= CONFIG_NTP_TIMEOUT_S) {
            ESP_LOGW(TAG, "time sync failed after %d tries", n_retry);
            s_sntp_retry_num++;
        } else {
            s_sntp_retry_num = 0;
        }

        xSemaphoreGive(sync_mutex);
        clear_task_event(EV_TASK_TIME_SYNC);
    }
}

/** @note:
 * @url https://github.com/espressif/esp-idf/issues/7244
 * [jack0c] commented on May 20, 2022
 * AP have a TBTT time to send beacon which is times beacon interval, for instance TBTTn=n*102400, n = 0,1,2,3,...
 * However, AP can't send the beacon in time, for instance sent at TSF, which is TSF.
 * Station receive the beacon at local time T0, which is record by the hardware.
 * If you can get TSF,T0, and your now the current time, then you can sync time among the stations connected to the same AP.
 *
 * - TBTT=TSF - TSF%Beaon_interval
 * - T1 - T0 + TSF%Beaon_interval means time since AP TBTT
 * - If you want to make TBTT corresponding to T2, then current time should be T2+T1-T0+TSF%Beaon_interval
 * - then the delta is T2-T0+TSF%Beaon_interval, you need increase your local time by T2-T0+TSF%Beaon_interval
 * - all devices connect to the same AP should have same time
**/