#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_task_wdt.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include <driver/adc.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "nvs_flash.h"

#include "apps.h"
#include "settings.h"
#include "imu.h"
#include "sys.h"
#include "tcp.h"

static const char* TAG = "app_playground";

#define for_countup(it, max) \
        for(size_t (it) = 0; (it) < (max); ++(it))

#define for_countdown(it, max) \
        for(size_t (it) = max; (it) > 0; --(it))

#if CONFIG_IMU_TYPE == IMU_TYPE_HI229

static esp_err_t test_check_echo(hi229_t* p_gy, uint8_t* msg, size_t len) {
    char rx_buf[10];
    if (msg == NULL) {
        return ESP_FAIL;
    } else {
        if (len <= 0) len = strlen((const char*)msg);
        int cursor = 0;
        for_countup(it, 5000) {
            uart_read_bytes(p_gy->port, &rx_buf[cursor], 1, 0xF);
            // printf("0x%x.", rx_buf[cursor]);
            if (rx_buf[cursor] != msg[cursor]) {
                bzero(rx_buf, sizeof(rx_buf));
                cursor = 0;
                continue;
            }
            cursor++;
            if (cursor >= len) {
                return ESP_OK;
            }
        }
    }
    return ESP_FAIL;
}

static esp_err_t test_send(hi229_t* p_gy, uint8_t* ctrl_msg, uint8_t* echo) {

    for_countup(n_retry, 10) {
        uart_write_bytes_with_break((p_gy->port), ctrl_msg, sizeof(ctrl_msg), 0xF);
        uart_wait_tx_done((p_gy->port), portMAX_DELAY);

        esp_err_t err = test_check_echo(p_gy, echo, 2);

        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Echo Succeed");
            return ESP_OK;
        } else {
            ESP_LOGE(TAG, "Echo Failed");
        }
        sys_delay_ms(100);
    }


    return ESP_FAIL;
}

static esp_err_t test_read(hi229_t* p_gy) {
    hi229_read(p_gy);
    return ESP_OK;
}

#endif

#define CONFIG_BATTERY_EN_PIN GPIO_NUM_35
#define CONFIG_BATTERY_READ_PIN GPIO_NUM_34
#define CONFIG_BATTERY_EN_VALUE 0

static esp_err_t battery_msp_init() {
    /** Init GPIO **/
    gpio_config_t io_config = {
            .pin_bit_mask = (1ull << CONFIG_BATTERY_EN_PIN),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
    };

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);// FIXME Pseudo channel

    gpio_config(&io_config);
    return ESP_OK;
}

static int battery_read_level() {
    gpio_set_level(CONFIG_BATTERY_EN_PIN, CONFIG_BATTERY_EN_VALUE);
    sys_delay_ms(50);
    int val = adc1_get_raw(ADC1_CHANNEL_0);
    gpio_set_level(CONFIG_BATTERY_EN_PIN, !CONFIG_BATTERY_EN_VALUE);
    return val;
}

void app_playground(void* pvParameters) {
    ESP_LOGI(TAG, "APP Playgournd\n");
    hi229_enable(&g_imu);
    while (1) {
        // test_send(&g_imu, (uint8_t*)"AT+INFO\r\n", (uint8_t*)"OK");
        test_read(&g_imu);
        printf("g_imu->n_bytes=%d", g_imu.n_bytes);
        sys_delay_ms(5000);
    }

}