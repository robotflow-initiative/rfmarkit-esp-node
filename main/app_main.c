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

#if CONFIG_EN_IMU_CB
#include "bno08x.h"
#include "bno08x_driver.h"
#endif

static const char *TAG = "app_main";

static uint8_t s_data_buf[CONFIG_DATA_BUF_LEN * sizeof(imu_dgram_t)];

#if CONFIG_EN_IMU_CB
typedef struct {
    imu_t base;
    BNO08x driver;
} bno08x_t;

/** Get a ring buffer pointer **/
ring_buf_t *serial_buf = &g_mcu.imu_ring_buf;

/** Create a imu data structure **/
imu_dgram_t imu_data = {0};
uint32_t seq = 0;

void imu_data_cb(void *arg)
{
    /** Read IMU data **/
    esp_err_t err = g_imu.read(g_imu.p_imu, &imu_data, true);

    /** If the err occurs(most likely due to the empty uart buffer), wait **/
    /** but it should be impossible */
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "IMU read error: %d", err);
    }

    /** Tag seq number, timestamp, buffer_len(how many bits are left in the buffer) **/
    imu_data.seq = seq++;

    /** Add the imu data to the ring buffer **/
    ring_buf_push(serial_buf, (uint8_t *) &imu_data);
}
#endif

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
    ring_buf_init(&g_mcu.imu_ring_buf, CONFIG_DATA_BUF_LEN, sizeof(imu_dgram_t), s_data_buf, true);

    /** Test IMU availability, must run at the end of init() **/
    g_imu.self_test(g_imu.p_imu);

#if CONFIG_EN_IMU_CB
    bno08x_t *p_bno08x = (bno08x_t *) g_imu.p_imu;
    BNO08x *p_driver = &p_bno08x->driver;
    /** register the IMU callback */
    BNO08x_register_cb(p_driver, imu_data_cb);
#endif
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
