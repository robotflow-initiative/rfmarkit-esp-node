#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/FreeRTOSConfig.h"

#include "esp_system.h"
#include "esp_log.h"

#include "apps.h"
#include "blink.h"
#include "settings.h"
#include "imu.h"
#include "sys.h"
#include "ble_srv.h"
#include "ring_buf.h"
#include "rest_controller.h"

static const char *TAG = "app_main";

static uint8_t s_serial_buf[CONFIG_SERIAL_QUEUE_LEN * sizeof(imu_dgram_t)];

static void init() {
    sys_init_chip();
    sys_init_nvs(); // Initialize NVS
    sys_init_events(); // Create system event group

    /** If the device is waken up, decide whether to go to deep sleep again or not **/
    power_mgmt_init(); // Initialize power management
    if (g_power_mgmt_info.state == POWER_WAKEN) {
        power_mgmt_state_t new_state = power_mgmt_wake_up_handler();
        if (new_state == POWER_DEEP_SLEEP) {
            power_mgmt_on_enter_deep_sleep(true);
        }
    } else if (g_power_mgmt_info.state == POWER_UNKNOWN) {
        esp_restart(); // Restart the device if the state is unknown
    }
    power_mgmt_on_enter_standby();

    sys_init_buttons();/** Setup Button **/
    blink_msp_init(); /** Init LED **/
    imu_init(g_imu); /** Init global imu struct g_imu **/

    sys_init_ble_srv(); /** Init nimble BLE **/
    sys_wifi_msp_init(); /** Initialize Wi-Fi **/
}

_Noreturn void app_main(void) {
    init();
    /** Test IMU availability **/
    imu_self_test(&g_imu);

    /** Init ring buffer **/
    ring_buf_init(&g_mcu.imu_ring_buf, CONFIG_SERIAL_QUEUE_LEN, sizeof(imu_dgram_t), s_serial_buf, true);

    /** Launch tasks **/
#if CONFIG_MULTI_CORE
    launch_task_multicore(app_data_client, "app_data_client", 4096, NULL, 12, tcp_task, 0x1);
    launch_task_multicore(app_uart_monitor, "app_uart_monitor", 4096, NULL, 12,app_uart_monitor_task, 0x1);
    launch_task_multicore(app_system_loop, "app_system_loop", 4096, NULL, 8, app_system_loop_task, 0x0);
#else
//    launch_task(app_data_client, "app_data_client", 4096, serial_queue, 5, tcp_task);
    launch_task(app_uart_monitor, "app_uart_monitor", 4096, serial_queue, 5, app_uart_monitor_task);
    launch_task(app_system_loop, "app_system_loop", 4096, NULL, 3, app_system_loop_task);
#endif

    /** Launch RESTful controller **/
    rest_controller_start(CONFIG_CONTROLLER_BASE_PATH);

    sys_log_heap_size();
    vTaskPrioritySet(NULL, 5);
    sys_ota_guard();

    while (1) {

        os_delay_ms(CONFIG_MAIN_LOOP_DUTY_PERIOD_S * 1000);
#if CONFIG_TOGGLE_PROFILING
        sys_log_trace();
#endif
    }
}
