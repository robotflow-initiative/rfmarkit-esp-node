#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_attr.h"

#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/ledc.h"
#include "nvs_flash.h"

#include "cJSON.h"

#include "blink.h"
#include "sys.h"



static uint8_t s_blink_seq[CONFIG_BLINK_SYNC_SEQ_LEN + CONFIG_BLINK_SEQ_LEN + CONFIG_BLINK_CHKSUM_SEQ_LEN];
// [ 4bit sync | 16 bit data | n bit chksum]

static int s_blink_idx;
uint8_t g_blink_pin;

static const char* TAG = "app_blink";

void blink_led_off(blink_led_t* p_led) {
    if (p_led->is_mono) {
        gpio_set_level(p_led->pins.mono, !p_led->en_val);
    } else {
        for (int idx = 0; idx < sizeof(p_led->pins.rgb) / sizeof(p_led->pins.rgb[0]); ++idx) {
            gpio_set_level(p_led->pins.rgb[idx], !p_led->en_val);
        }
    }
}

void blink_led_on(blink_led_t* p_led) {
    if (p_led->is_mono) {
        gpio_set_level(p_led->pins.mono, p_led->en_val);
    } else {
        for (int idx = 0; idx < sizeof(p_led->pins.rgb) / sizeof(p_led->pins.rgb[0]); ++idx) {
            gpio_set_level(p_led->pins.rgb[idx], p_led->en_val);
        }
    }
}

static bool get_flag(const uint8_t* arr, int item) {
    return (arr[item / 8] & (1 << item % 8)) >> (item % 8);
}


void blink_gpio_msp_init() {
    /** Init GPIO **/
    gpio_config_t io_config = {
            .pin_bit_mask = (1ull << CONFIG_BLINK_PIN),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_config);
}


#define CONFIG_LEDC_HS_TIMER          LEDC_TIMER_0
#define CONFIG_LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0

/*
 * Prepare individual configuration
 * for each channel of LED Controller
 * by selecting:
 * - controller's channel number
 * - output duty cycle, set initially to 0
 * - GPIO number where LED is connected to
 * - speed mode, either high or low
 * - timer servicing selected channel
 *   Note: if different channels use one timer,
 *         then frequency and bit_num of these channels
 *         will be the same
 */
ledc_channel_config_t g_ledc_channel = {
        .channel = CONFIG_LEDC_HS_CH0_CHANNEL,
        .duty = 0,
        .gpio_num = CONFIG_BLINK_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = CONFIG_LEDC_HS_TIMER
};


void blink_pwm_msp_init() {
    static ledc_timer_config_t s_ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = CONFIG_BLINK_MAX_DUTY,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = CONFIG_LEDC_HS_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&s_ledc_timer);
    ledc_channel_config(&g_ledc_channel);
}


static bool blink_timeout(void* args) {
    if (s_blink_idx >= CONFIG_BLINK_SEQ_LEN) s_blink_idx = 0;
#if CONFIG_BLINK_NO_PWM
    gpio_set_level(g_blink_pin, s_blink_seq[s_blink_idx] ? 1 : 0);
#else
    ledc_set_duty(g_ledc_channel.speed_mode, g_ledc_channel.channel, s_blink_seq[s_blink_idx] ? CONFIG_BLINK_MAX_DUTY : CONFIG_BLINK_MIN_DUTY);
    ledc_update_duty(g_ledc_channel.speed_mode, g_ledc_channel.channel);
#endif
    s_blink_idx++;
    return true;
}


void blink_init() {

#if CONFIG_BLINK_NO_PWM
    blink_gpio_msp_init();
#else
    blink_pwm_msp_init();
#endif

    /** After init, test up the led **/
    CONFIG_LED_OFF();
    os_delay_ms(1000);
    CONFIG_LED_ON();

    /** Read blink sequence from nvs **/
    nvs_handle_t blink_handle;
    uint8_t seq;
    nvs_open(CONFIG_BLINK_NVS_TABLE_NAME, NVS_READWRITE, &blink_handle);

    g_blink_pin = CONFIG_BLINK_DEFAULT_PIN;
    nvs_get_u8(blink_handle, "seq", &seq);
    ESP_LOGI(TAG, "Blinking pin: %d", g_blink_pin);
    ESP_LOGI(TAG, "Blinking sequence: %d", seq);
    s_blink_idx = 0;

    bzero(s_blink_seq, sizeof(s_blink_seq));
    uint8_t pattern[2] = { 1, 0 };
    bool value = 0;

    ESP_LOGI(TAG, "Sequence Number: %d", seq);

    /** fill sync bits **/
    s_blink_seq[0] = 1;
    s_blink_seq[1] = 1;
    s_blink_seq[2] = 1;
    s_blink_seq[3] = 0;

    /** fill data bits **/
    int n_ones = 0;
    for (int idx = 0; idx < 8; ++idx) {
        value = get_flag(&seq, 8 - idx);
        ESP_LOGD(TAG, "value: %d\n", value);
        if (value) {
            pattern[0] = !pattern[0];
            pattern[1] = !pattern[1];
            ++n_ones;
        }
        s_blink_seq[CONFIG_BLINK_SYNC_SEQ_LEN + idx * 2] = pattern[0];
        ESP_LOGD(TAG, "s_blink_seq[%d]=%d\n", idx * 2, pattern[0]);
        s_blink_seq[CONFIG_BLINK_SYNC_SEQ_LEN + idx * 2 + 1] = pattern[1];
        ESP_LOGD(TAG, "s_blink_seq[%d]=%d\n", idx * 2 + 1, pattern[1]);
    }

#ifdef CONFIG_BLINK_EN_CHKSUM
#else
#endif

    ESP_LOGW(TAG, "# -------- Begin of blink sequence -------- #");
    for (int idx = 0; idx < sizeof(s_blink_seq); ++idx) {
        printf("%d ", s_blink_seq[idx]);
    }
    printf("\n");
    ESP_LOGW(TAG, "# --------- End of blink sequence --------- #");

    // Arm timers
    timer_config_t config = {
            .alarm_en = 1,
            .counter_en = 0,
            .counter_dir = TIMER_COUNT_UP,
            .auto_reload = 1,
            .divider = TIMER_DIVIDER,
    };
    timer_init(0, 0, &config);
    timer_set_counter_value(CONFIG_BLINK_TIMER_GROUP, CONFIG_BLINK_TIMER_IDX, 0x00ull);
    timer_set_alarm_value(CONFIG_BLINK_TIMER_GROUP, CONFIG_BLINK_TIMER_IDX, TIMER_SCALE * CONFIG_BLINK_INTERVAL_MS / 1000);
    timer_enable_intr(CONFIG_BLINK_TIMER_GROUP, CONFIG_BLINK_TIMER_IDX);
    timer_isr_callback_add(CONFIG_BLINK_TIMER_GROUP, CONFIG_BLINK_TIMER_IDX, blink_timeout, NULL, ESP_INTR_FLAG_IRAM);
}


void blink_init_task(void * vParameters) {
    blink_init();
    blink_start();
    vTaskDelete(NULL);
}


void blink_start() {
    ESP_LOGI(TAG, "Timer started");
    ESP_LOGW(TAG, "# -------- Begin of blink sequence -------- #");
    for (int idx = 0; idx < CONFIG_BLINK_SEQ_LEN; ++idx) {
        printf("%d ", s_blink_seq[idx]);
    }
    printf("\n");
    ESP_LOGW(TAG, "# --------- End of blink sequence --------- #");
    timer_start(CONFIG_BLINK_TIMER_GROUP, CONFIG_BLINK_TIMER_IDX);
    deice_always_on();
}

void blink_stop() {
    ESP_LOGI(TAG, "Timer stopped");
    timer_pause(CONFIG_BLINK_TIMER_GROUP, CONFIG_BLINK_TIMER_IDX);
    CONFIG_LED_ON();
    device_reset_sleep_countup();
}

#define BLINK_SET_OFFSET 10
COMMAND_FUNCTION(blink_set) {
    ESP_LOGI(TAG, "Executing command : IMU_BLINK_SET");
    uint8_t seq = 0;
    int offset = 0;

    /** Open nvs table **/
    nvs_handle_t blink_handle;
    ESP_ERROR_CHECK(nvs_open(CONFIG_BLINK_NVS_TABLE_NAME, NVS_READWRITE, &blink_handle));

    /**
    rx_buffer = "blink_set {"seq":"[0-255]"}
    **/
    char* end_ptr = NULL;
    seq = (int)strtol(&rx_buffer[BLINK_SET_OFFSET], &end_ptr, 10);

    if (end_ptr > &rx_buffer[BLINK_SET_OFFSET]) {
        seq = seq % 0x100;
        snprintf(tx_buffer + offset, tx_len - offset, "Blink seq set to %d, reboot to make effective\n\n", seq);
    } else {
        snprintf(tx_buffer + offset, tx_len - offset, "Blink set failed, invalid seq\n\n");
        goto blink_set_cleanup;
    }
    nvs_set_u8(blink_handle, "seq", seq);
    nvs_commit(blink_handle);
    nvs_close(blink_handle);

    return ESP_OK;

blink_set_cleanup:

    nvs_commit(blink_handle);
    nvs_close(blink_handle);

    return ESP_FAIL;
}

COMMAND_FUNCTION(blink_start) {
    ESP_LOGI(TAG, "Executing command : IMU_BLINK_START");

    blink_start();

    snprintf(tx_buffer, tx_len, "Blink started\n\n");
    return ESP_OK;
}

COMMAND_FUNCTION(blink_stop) {
    ESP_LOGI(TAG, "Executing command : IMU_BLINK_STOP");

    blink_stop();

    snprintf(tx_buffer, tx_len, "Blink stopped\n\n");
    return ESP_OK;
}

COMMAND_FUNCTION(blink_get) {
    ESP_LOGI(TAG, "Executing command : IMU_BLINK_GET");
    uint8_t seq = 0;

    /** Open nvs table **/
    nvs_handle_t blink_handle;
    nvs_open("blink", NVS_READWRITE, &blink_handle);

    nvs_get_u8(blink_handle, "seq", &seq);

    snprintf(tx_buffer, tx_len, "Blink seq is %d, \n\n", seq);

    nvs_close(blink_handle);
    return ESP_OK;
}

COMMAND_FUNCTION(blink_off) {
    ESP_LOGI(TAG, "Executing command : IMU_BLINK_OFF");

    blink_stop();

    CONFIG_LED_OFF();

    snprintf(tx_buffer, tx_len, "Blink pin %d set to OFF\n\n", g_blink_pin);

    return ESP_OK;
}
