//
// Created by liyutong on 2024/4/30.

#include <esp_adc_cal.h>
#include "esp_sleep.h"
#include "esp_log.h"

#include "driver/adc.h"
#include "driver/gpio.h"

#include "battery.h"
#include "modelspec.h"

static const char *TAG = "battery         ";

#if defined(CONFIG_IDF_TARGET_ESP32)
#define ADC_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_VREF
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#define ADC_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP_FIT
#endif

#define ADC_ATTEN_DB ADC_ATTEN_DB_11
#define ADC_WIDTH_BIT ADC_WIDTH_BIT_12

static esp_adc_cal_characteristics_t adc1_chars;
static bool cali_enable = false;

static bool adc_calibration_init(void) {
    esp_err_t ret;

    ret = esp_adc_cal_check_efuse(ADC_CALI_SCHEME);
    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Calibration scheme not supported, skip software calibration");
    } else if (ret == ESP_ERR_INVALID_VERSION) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else if (ret == ESP_OK) {
        cali_enable = true;
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB, ADC_WIDTH_BIT, 0, &adc1_chars);
    } else {
        ESP_LOGE(TAG, "Invalid arg");
    }

    return cali_enable;
}

/**
 * @brief Initialize battery management
 * @return ESP_OK on success, ESP_FAIL on error
**/
esp_err_t battery_msp_init() {

    /** Init ADC_EN GPIO **/
#ifdef CONFIG_BATTERY_EN_PIN
    gpio_config_t io_config = {
        .pin_bit_mask = (1ull << CONFIG_BATTERY_EN_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_config);
#endif

    /** Init GPIO **/
    cali_enable = adc_calibration_init();

    /** ADC1 config **/
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(CONFIG_BATTERY_READ_ADC_CHANNEL, ADC_ATTEN_DB));

    /** Self-Test **/
    int lvl = battery_read_level();
    ESP_LOGI(TAG, "battery level: %d", lvl);
    return ESP_OK;
}

/**
 * @brief Read battery level
 * @return mV
**/
int battery_read_level() {
#ifdef CONFIG_BATTERY_EN_PIN
    gpio_set_level(CONFIG_BATTERY_EN_PIN, CONFIG_BATTERY_EN_VALUE);
#endif
    battery_delay_ms(50);
    int adc_raw = adc1_get_raw(CONFIG_BATTERY_READ_ADC_CHANNEL);
    uint32_t voltage;
    if (cali_enable) {
        voltage = esp_adc_cal_raw_to_voltage(adc_raw, &adc1_chars);
        ESP_LOGD(TAG, "cali data: %d mV", voltage);
    } else {
        voltage = adc_raw * 1100 / (1 << ADC_WIDTH_BIT_12);
    }
#ifdef CONFIG_BATTERY_EN_PIN
    gpio_set_level(CONFIG_BATTERY_EN_PIN, !CONFIG_BATTERY_EN_VALUE);

    /**
    It goes like this
    VBAT
    +
    |
    |
    [R1] 15kΩ
    |
    |---- ADC_CHANNEL
    |
    [R2] 10kΩ
    |
    \
    |---- ADC_EN
    /
    |
    -
    GND
     **/

    voltage = voltage / 2 * 5; // the voltage divider ratio is 2
#endif
    return (int) voltage;
}