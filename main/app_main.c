#include <esp_ota_ops.h>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/FreeRTOSConfig.h"

#include "esp_system.h"
#include "esp_log.h"

#include "apps.h"
#include "settings.h"
#include "imu.h"
#include "sys.h"
#include "ring_buf.h"

static const char *TAG = "app_main";

static uint8_t s_serial_buf[CONFIG_SERIAL_QUEUE_LEN * sizeof(imu_dgram_t)];

static void init() {
    sys_init_chip();
    sys_init_nvs(); // Initialize NVS
    sys_init_events(); // Create system event group
    sys_wifi_netif_init(); // Initialize Wi-Fi netif

    /** If the device is waken up, decide whether to go to deep sleep again or not **/
    power_mgmt_init(); // Initialize power management
    if (g_power_mgmt_ctx.state == POWER_WAKEN) {
        power_mgmt_state_t new_state = power_mgmt_wake_up_handler();
        if (new_state == POWER_DEEP_SLEEP) {
            power_mgmt_on_enter_deep_sleep(true);
        }
    } else if (g_power_mgmt_ctx.state == POWER_UNKNOWN) {
        esp_restart(); // Restart the device if the state is unknown
    }
    power_mgmt_on_enter_standby(); // Enter standby mode

    /** Init ring buffer **/
    ring_buf_init(&g_mcu.imu_ring_buf, CONFIG_SERIAL_QUEUE_LEN, sizeof(imu_dgram_t), s_serial_buf, true);

    /** Test IMU availability, must run at the end of init() **/
    imu_self_test(&g_imu);
}


void app_main(void) {
    init();
    ESP_LOGI(TAG, "main application started");

    /** Launch tasks **/
    sys_start_tasks();

    sys_ota_guard();
    ESP_LOGI(TAG, "main application finished");
    vTaskDelete(NULL);
}
