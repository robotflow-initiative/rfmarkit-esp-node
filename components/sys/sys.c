#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_sleep.h"
#include "esp_log.h"

#include "sys.h"
#include "imu.h"
#include "device.h"
#include "blink.h"
#include "settings.h"

static const char* TAG = "sys";

void sys_enter_deep_sleep() {

    // esp_sleep_enable_gpio_wakeup();
    /* Enter sleep mode */
    ESP_LOGI(TAG, " Going to deep sleep (shutdown)");

    ESP_LOGI(TAG, "Disabling GY95");
    imu_disable(&g_imu);
    xEventGroupClearBits(g_mcu.sys_event_group, IMU_ENABLED_BIT);
    clear_sys_event(IMU_ENABLED);

    device_delay_ms(200);
    ESP_LOGI(TAG, "IMU ctrl_pin is set to %d", gpio_get_level(g_imu.ctrl_pin));

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

COMMAND_FUNCTION(shutdown) {
    ESP_LOGI(TAG, "Executing command : IMU_SHUTDOWN");
    sys_enter_deep_sleep();
    return ESP_OK;
}