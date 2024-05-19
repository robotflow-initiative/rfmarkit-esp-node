//
// Created by liyutong on 2024/4/30.
//
#include <esp_wifi.h>

#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_attr.h"
#include "esp_log.h"

#include "apps.h"
#include "settings.h"
#include "sys.h"
#include "hi229.h"
#include "blink.h"
#include "ble_srv.h"
#include "rest_controller.h"
#include "spatial.h"

static SemaphoreHandle_t sync_mutex = NULL;
RTC_DATA_ATTR power_mgmt_ctx_t g_power_mgmt_ctx;
static TimerHandle_t power_save_timer = NULL;

static const char *TAG = "sys.pm          ";

static void power_mgmt_handle_transition(__attribute__((unused)) TimerHandle_t xTimer);

static esp_err_t arm_power_save_timer() {
    if (power_save_timer == NULL) {
        power_save_timer = xTimerCreate("power_save_timer", pdMS_TO_TICKS(CONFIG_POWER_SAVE_TIMEOUT_S * 1000), pdFALSE, NULL, power_mgmt_handle_transition);
        if (xTimerStart(power_save_timer, 0) == pdPASS) {
            ESP_LOGI(TAG, "power save timer armed for %d seconds", CONFIG_POWER_SAVE_TIMEOUT_S);
            return ESP_OK;
        } else {
            ESP_LOGE(TAG, "failed to arm power save timer");
            return ESP_FAIL;
        }
    } else {
        xTimerReset(power_save_timer, 0);
        ESP_LOGI(TAG, "power save timer re-armed for %d seconds", CONFIG_POWER_SAVE_TIMEOUT_S);
    }
    return ESP_FAIL;
}

static void disarm_power_save_timer() {
    if (power_save_timer != NULL) {
        xTimerStop(power_save_timer, 0);
        xTimerDelete(power_save_timer, 0);
        power_save_timer = NULL;
        ESP_LOGI(TAG, "power save timer disarmed");
    }
}

void reset_power_save_timer() {
    if (power_save_timer != NULL) {
        xTimerReset(power_save_timer, 0);
        ESP_LOGI(TAG, "power save timer reset");
    }
}

esp_err_t power_mgmt_init() {
    /** Cancel hold **/
    gpio_hold_dis(CONFIG_IMU_CTRL_PIN);
    gpio_hold_dis(CONFIG_BLINK_PIN);
    gpio_deep_sleep_hold_dis();
    /** Enable gpio hold in deep sleep **/
    gpio_deep_sleep_hold_en();

    ++g_power_mgmt_ctx.boot_count;
    ESP_LOGI(TAG, "system booted %d times", g_power_mgmt_ctx.boot_count);

    if (g_power_mgmt_ctx.initialized) {
        if (g_power_mgmt_ctx.state == POWER_DEEP_SLEEP) {
            g_power_mgmt_ctx.state = POWER_WAKEN;
            // Do not change power mode
            g_power_mgmt_ctx.next_mode = g_power_mgmt_ctx.mode;
            g_power_mgmt_ctx.mutex = xSemaphoreCreateMutex();
            memset(&g_power_mgmt_ctx.peripheral_state, 0, sizeof(power_peripheral_state_t));
            return ESP_OK;
        } else {
            g_power_mgmt_ctx.state = POWER_UNKNOWN;
            g_power_mgmt_ctx.mutex = NULL;
            memset(&g_power_mgmt_ctx.peripheral_state, 0, sizeof(power_peripheral_state_t));
            return ESP_FAIL;
        }
    } else {
        g_power_mgmt_ctx.initialized = true;
        g_power_mgmt_ctx.state = POWER_NORMAL_BOOT;
        g_power_mgmt_ctx.mode = POWER_MODE_NORMAL;
        g_power_mgmt_ctx.next_mode = g_power_mgmt_ctx.mode;
        g_power_mgmt_ctx.mutex = xSemaphoreCreateMutex();
        memset(&g_power_mgmt_ctx.peripheral_state, 0, sizeof(power_peripheral_state_t));
        return ESP_OK;
    }
}

power_mgmt_state_t power_mgmt_wake_up_handler() {
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_EXT0) {
        ESP_LOGI(TAG, "wakeup from button");
        return POWER_NORMAL_BOOT;
    } else {
        // TODO: update this logic after PCB fix
        ESP_LOGI(TAG, "wakeup from timer");
        imu_init(g_imu);
        imu_dgram_t imu_status;
        for (esp_err_t err = ESP_FAIL; err != ESP_OK; taskYIELD()) {
            err = imu_read(&g_imu, &imu_status, true);
            ESP_LOGD(TAG, "imu_read err=%d", err);
        }
        Vector3 ref = {0, 0, -1};
        Vector3 acc_diff;
        spatial_vector_multiply_plus((const Vector3 *) &imu_status.imu[0].acc, &ref, -1, &acc_diff);
        float acc_diff_norm = spatial_vector_norm(&acc_diff);
        ESP_LOGI(
            TAG,
            "acc=[%f, %f, %f], norm=%f, diff=[%f, %f, %f],",
            imu_status.imu[0].acc[0], imu_status.imu[0].acc[1], imu_status.imu[0].acc[2], acc_diff_norm, acc_diff.x, acc_diff.y, acc_diff.z
        );

        return POWER_DEEP_SLEEP;
    }
}

esp_err_t power_mgmt_on_enter_standby() {
    /** Setup Button **/
    if (!g_power_mgmt_ctx.peripheral_state.button_initialized) {
        sys_init_buttons();
        g_power_mgmt_ctx.peripheral_state.button_initialized = true;
    }

    /** Init LED **/
    if (!g_power_mgmt_ctx.peripheral_state.led_initialized) {
        blink_msp_init();
        g_power_mgmt_ctx.peripheral_state.led_initialized = true;
    }

    /** Init global imu struct g_imu **/
    if (!g_power_mgmt_ctx.peripheral_state.imu_initialized) {
        imu_init(g_imu);
        g_power_mgmt_ctx.peripheral_state.imu_initialized = true;
    } else {
        if (!g_imu.enabled) {
            imu_enable(&g_imu);
        }
    }

    /** Init nimble BLE **/
    if (!g_power_mgmt_ctx.peripheral_state.ble_enabled && g_power_mgmt_ctx.mode != POWER_MODE_LOW_ENERGY) {
        sys_start_ble_srv();
        g_power_mgmt_ctx.peripheral_state.ble_enabled = true;
    } else if (g_power_mgmt_ctx.mode == POWER_MODE_LOW_ENERGY) {
        sys_stop_ble_srv();
        g_power_mgmt_ctx.peripheral_state.ble_enabled = false;
    }

    /** Initialize Wi-Fi **/
    if (!g_power_mgmt_ctx.peripheral_state.wifi_initialized) {
        sys_wifi_msp_init();
        g_power_mgmt_ctx.peripheral_state.wifi_initialized = true;
    }
    /** Configure Wi-Fi tx power **/
    ESP_LOGI(TAG, "set wifi tx power level: %d", CONFIG_MAX_TX_POWER);
    esp_wifi_set_max_tx_power(CONFIG_NORMAL_TX_POWER);

    /** Launch RESTful controller **/
    if (!g_power_mgmt_ctx.peripheral_state.controller_initialized) {
        g_mcu.rest_controller = rest_controller_start(CONFIG_CONTROLLER_BASE_PATH);
        g_power_mgmt_ctx.peripheral_state.controller_initialized = true;
    }

    /** Turn the power save timer on **/
    if (g_power_mgmt_ctx.mode != POWER_MODE_PERFORMANCE) {
        arm_power_save_timer();
    } else {
        disarm_power_save_timer();
    }

    g_power_mgmt_ctx.state = POWER_STANDBY;
    return ESP_OK;
}

esp_err_t power_mgmt_on_enter_active() {
    ESP_LOGI(TAG, "set wifi tx power level: %d", CONFIG_MAX_TX_POWER);
    esp_wifi_set_max_tx_power(CONFIG_MAX_TX_POWER);

    g_power_mgmt_ctx.state = POWER_ACTIVE;
    return ESP_OK;
}

esp_err_t power_mgmt_on_enter_power_save() {
    /** De-initialize BLE **/
    if (g_power_mgmt_ctx.peripheral_state.ble_enabled) {
        ESP_LOGI(TAG, "disabling BLE");
        sys_stop_ble_srv();
        g_power_mgmt_ctx.peripheral_state.ble_enabled = false;
    }

    /** De-initialize Wi-Fi **/
    if (g_power_mgmt_ctx.peripheral_state.wifi_initialized) {
        ESP_LOGI(TAG, "disabling Wi-Fi");
        sys_wifi_msp_deinit();
        g_power_mgmt_ctx.peripheral_state.wifi_initialized = false;
    }

    /** Turn led off **/
    ESP_LOGI(TAG, "disabling LED");
    blink_led_off();

    /** Turn off RESTful Controller **/
    if (g_power_mgmt_ctx.peripheral_state.controller_initialized) {
        ESP_LOGI(TAG, "disabling RESTful controller");
        rest_controller_stop(g_mcu.rest_controller);
        g_power_mgmt_ctx.peripheral_state.controller_initialized = false;
    }

    g_power_mgmt_ctx.state = POWER_SAVE;
    return ESP_OK;
}

_Noreturn esp_err_t power_mgmt_on_enter_deep_sleep(bool wakeup) {
    ESP_LOGI(TAG, "disabling IMU and led");

    imu_disable(&g_imu);
    g_mcu.state.led_manual = true;
    os_delay_ms(100);

    /** Operate LED to indicate deep sleep **/
    if (g_power_mgmt_ctx.peripheral_state.led_initialized) {
        blink_led_on();
        os_delay_ms(2000);
        blink_led_off();
    }

    /** Hold pins in deep sleep **/
    gpio_hold_en(CONFIG_IMU_CTRL_PIN);
    gpio_hold_en(CONFIG_BLINK_PIN);

    if (wakeup) {
        esp_sleep_enable_timer_wakeup(CONFIG_POWER_WAKE_UP_DURATION_S * 1000000LL);
        ESP_LOGI(TAG, "entering deep sleep for %d seconds...", CONFIG_POWER_WAKE_UP_DURATION_S);
    } else {
        ESP_LOGI(TAG, "entering deep sleep forever...");
    }
    /** Allow wakeup from button **/
    esp_sleep_enable_ext0_wakeup(CONFIG_BUTTON_FN_GPIO_PIN, CONFIG_BUTTON_FN_ACTIVE_LEVEL);

    g_power_mgmt_ctx.state = POWER_DEEP_SLEEP;
    esp_deep_sleep_start();
}

__attribute__((unused)) static void app_power_save(__attribute__((unused)) void *pvParameter) {
    sys_stop_tasks();
    sys_stop_timers();

    ESP_LOGI(TAG, "entering power save mode. ble_status=%d, wifi_status=%d", g_power_mgmt_ctx.peripheral_state.ble_enabled, g_power_mgmt_ctx.peripheral_state.wifi_initialized);
    power_mgmt_on_enter_power_save();

    imu_dgram_t imu_status;
    float old_acc[3] = {0, 0, 0};

    uart_flush(g_imu.port);
    for (esp_err_t err = ESP_FAIL; err != ESP_OK; taskYIELD(), os_delay_ms(100)) {
        err = imu_read(&g_imu, &imu_status, true);
        ESP_LOGD(TAG, "imu_read_init err=%d", err);
    }
    memcpy(old_acc, imu_status.imu[0].acc, sizeof(old_acc));

    int cycle_count = 0;

    while (1) {
        ESP_LOGI(TAG, "light sleep for 10 seconds");
        esp_sleep_enable_timer_wakeup(10 * 1000000UL); // 3 seconds
        imu_disable(&g_imu);
        os_delay_ms(10);
        esp_light_sleep_start();
        ESP_LOGI(TAG, "Returned from light sleep, reason:%s\n", esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0 ? "pin" : "timer");

        imu_enable(&g_imu);
        imu_buffer_reset(&g_imu);
        for (esp_err_t err = ESP_FAIL; err != ESP_OK; taskYIELD(), os_delay_ms(100)) {
            err = imu_read(&g_imu, &imu_status, true);
            ESP_LOGD(TAG, "imu_read err=%d", err);
        }

        Vector3 acc_diff;
        spatial_vector_multiply_plus((const Vector3 *) &imu_status.imu[0].acc, (const Vector3 *) &old_acc, -1, &acc_diff);
        float acc_diff_norm = spatial_vector_norm(&acc_diff);

        if (acc_diff_norm > CONFIG_POWER_SAVE_MOTION_LIMIT) {
            ESP_LOGI(TAG, "acc_diff_norm=%f, threshold=%f, wake up", acc_diff_norm, CONFIG_POWER_SAVE_MOTION_LIMIT);
            break;
        } else {
            ESP_LOGI(
                TAG,
                "acc=[%f, %f, %f], norm=%f, diff=[%f, %f, %f],",
                imu_status.imu[0].acc[0], imu_status.imu[0].acc[1], imu_status.imu[0].acc[2], acc_diff_norm, acc_diff.x, acc_diff.y, acc_diff.z
            );
            memcpy(old_acc, imu_status.imu[0].acc, sizeof(old_acc));
        }

        cycle_count++;
        if (cycle_count > CONFIG_POWER_SAVE_CYCLE_MAX_COUNT) {
            ESP_LOGI(TAG, "power save cycle limit reached, going to deep sleep");
            power_mgmt_on_enter_deep_sleep(true);
        }
    }
    power_mgmt_on_enter_standby();
    sys_start_tasks();
    vTaskDelete(NULL);
}

static void power_mgmt_handle_transition(__attribute__((unused)) TimerHandle_t xTimer) {
    // TODO: due to PCE failure, the ESP32 module's SPI CSC is connected to IMU's SYNC_IN by mistake, thus power_save function cannot work properly
    // xTaskCreate(app_power_save, "app_power_save", 4096, NULL, 12, NULL);
    power_mgmt_on_enter_deep_sleep(false);
}


void sys_power_mgmt_handler(__attribute__((unused)) void *handler_args, __attribute__((unused)) esp_event_base_t base, int32_t id, __attribute__((unused)) void *event_data) {
    /** TODO: Implement logic**/
    if (!sync_mutex) {
        sync_mutex = xSemaphoreCreateMutex();
    }
    if (xSemaphoreTake(sync_mutex, portMAX_DELAY) == pdTRUE) {
        switch (id) {
            case SYSTEM_POWER_MGMT_EVENT:
                ESP_LOGI(TAG, "power management event triggered for regular update");
                /** Work for mode transition **/
                if (g_power_mgmt_ctx.next_mode != g_power_mgmt_ctx.mode) {
                    switch (g_power_mgmt_ctx.next_mode) {
                        case POWER_MODE_PERFORMANCE:
                            ESP_LOGI(TAG, "switch to performance mode");
                            g_power_mgmt_ctx.mode = POWER_MODE_PERFORMANCE;
                            disarm_power_save_timer();
                            break;
                        case POWER_MODE_NORMAL:
                            ESP_LOGI(TAG, "switch to normal mode");
                            g_power_mgmt_ctx.mode = POWER_MODE_NORMAL;
                            arm_power_save_timer();
                            break;
                        case POWER_MODE_LOW_ENERGY:
                            ESP_LOGI(TAG, "switch to low energy mode");
                            g_power_mgmt_ctx.mode = POWER_MODE_LOW_ENERGY;
                            if (g_power_mgmt_ctx.peripheral_state.ble_enabled) {
                                sys_stop_ble_srv();
                                g_power_mgmt_ctx.peripheral_state.ble_enabled = false;
                            }
                            arm_power_save_timer();
                            break;
                        default:
                            break;
                    }
                }

                /** Work on current event **/
                switch (g_power_mgmt_ctx.state) {
                    case POWER_STANDBY:
                        if (g_power_mgmt_ctx.no_sleep) {
                            reset_power_save_timer();
                        }
                        break;
                    case POWER_ACTIVE:
                        reset_power_save_timer();
                        break;
                    case POWER_NORMAL_BOOT:
                    case POWER_SAVE:
                    case POWER_DEEP_SLEEP:
                    case POWER_WAKEN:
                    default:
                        break;
                }
                break;
            case SYSTEM_MODE_CHANGE_EVENT:
                ESP_LOGI(TAG, "power management event triggered for mode change");
                if (g_mcu.state.active) {
                    power_mgmt_on_enter_active();
                    g_power_mgmt_ctx.no_sleep = true;
                } else {
                    power_mgmt_on_enter_standby();
                    g_power_mgmt_ctx.no_sleep = false;
                }
                break;
            case SYSTEM_OTA_EVENT:
                ESP_LOGI(TAG, "power management event triggered for ota event");
                g_power_mgmt_ctx.no_sleep = true; // Disable deep sleep / light sleep
                break;
            default:
                break;
        }
        xSemaphoreGive(sync_mutex);
    }
}


