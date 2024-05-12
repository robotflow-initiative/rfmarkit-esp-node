#include <string.h>

#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_attr.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "blink.h"
#include "sys.h"

static uint8_t s_seq_enc_pattern[CONFIG_BLINK_SYNC_SEQ_LEN + CONFIG_BLINK_SEQ_LEN];
static uint8_t s_fast_breath_pattern[16] = {0, 32, 64, 96, 128, 160, 192, 224, 255, 224, 192, 160, 128, 96, 64, 32};
static uint8_t s_slow_breath_pattern[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 255};
static uint8_t s_fast_flash_pattern[16] = {0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255};

static uint8_t *s_selected_pattern; // Used in interrupt
static uint8_t s_selected_pattern_len; // Used in interrupt
static int s_blink_idx; // Used in interrupt

blink_config_t g_blink_cfg;
static esp_timer_handle_t s_blink_timer;

static const char *TAG = "blink           ";

/**
 * Function to get bit from byte array
 * @param arr the byte array
 * @param index the item index
 * @return treate array as a bit array and return the value, e.g. {5,5}[6] -> {0,0,0,0,0,1,0,1,0,0,0,0,0,1,0,1}[6] = 1
 **/
static bool get_bit_from_byte_array(const uint8_t *arr, int index) {
    return (arr[index / 8] & (1U << index % 8)) >> (index % 8);
}

/**
 * Function to stop the LED
**/
void blink_led_off() {
    if (g_mcu.state.led_status == LED_FAST_FLASH || g_mcu.state.led_status == LED_SLOW_BREATH || g_mcu.state.led_status == LED_FAST_BREATH ||
        g_mcu.state.led_status == LED_SEQ_ENC) {
        esp_timer_stop(s_blink_timer);
    }
    ledc_stop(g_blink_cfg.ledc_channel.speed_mode, g_blink_cfg.ledc_channel.channel, 0);
    gpio_set_level(g_blink_cfg.pin_num, !g_blink_cfg.en_val);
    g_mcu.state.led_status = LED_OFF;
}

/**
 * Function to turn on the LED
**/
void blink_led_on() {
    if (g_mcu.state.led_status == LED_FAST_FLASH || g_mcu.state.led_status == LED_SLOW_BREATH || g_mcu.state.led_status == LED_FAST_BREATH ||
        g_mcu.state.led_status == LED_SEQ_ENC) {
        esp_timer_stop(s_blink_timer);
    }
    ledc_stop(g_blink_cfg.ledc_channel.speed_mode, g_blink_cfg.ledc_channel.channel, 1);
    gpio_set_level(g_blink_cfg.pin_num, g_blink_cfg.en_val);
    g_mcu.state.led_status = LED_ON;
}

/**
 * Function to turn on the LED and change brightness
 * @param duty the duty cycle of the LED, 0-100
**/
void blink_led_set_duty(uint32_t duty) {
    if (g_mcu.state.led_status == LED_FAST_FLASH || g_mcu.state.led_status == LED_SLOW_BREATH || g_mcu.state.led_status == LED_FAST_BREATH ||
        g_mcu.state.led_status == LED_SEQ_ENC) {
        esp_timer_stop(s_blink_timer);
    }
    if (g_mcu.state.led_status == LED_OFF || g_mcu.state.led_status == LED_ON) {
        ledc_set_freq(g_blink_cfg.ledc_channel.speed_mode, LEDC_TIMER_0, CONFIG_BLINK_PWN_FREQ);
    }
    ledc_set_duty(g_blink_cfg.ledc_channel.speed_mode, g_blink_cfg.ledc_channel.channel, (CONFIG_BLINK_MAX_DUTY * duty) / 100);
    ledc_update_duty(g_blink_cfg.ledc_channel.speed_mode, g_blink_cfg.ledc_channel.channel);
    g_mcu.state.led_status = LED_DUTY;
}

/**
 * Function to blink the LED with fast flash pattern
**/
void blink_led_fast_flash() {
    if (g_mcu.state.led_status == LED_FAST_FLASH) return;
    if (g_mcu.state.led_status == LED_SLOW_BREATH || g_mcu.state.led_status == LED_FAST_BREATH || g_mcu.state.led_status == LED_SEQ_ENC) {
        esp_timer_stop(s_blink_timer);
    }
    s_selected_pattern = s_fast_flash_pattern;
    s_selected_pattern_len = sizeof(s_fast_flash_pattern);
    if (g_mcu.state.led_status == LED_ON || g_mcu.state.led_status == LED_OFF) {
        ledc_set_freq(g_blink_cfg.ledc_channel.speed_mode, LEDC_TIMER_0, CONFIG_BLINK_PWN_FREQ);
    }
    esp_timer_start_periodic(s_blink_timer, 50 * 1000);
    g_mcu.state.led_status = LED_FAST_FLASH;
}

/**
 * Function to blink the LED with fast breath pattern
**/
void blink_led_start_fast_breath_pattern() {
    if (g_mcu.state.led_status == LED_FAST_BREATH) {
        return;
    }
    if (g_mcu.state.led_status == LED_FAST_FLASH || g_mcu.state.led_status == LED_SLOW_BREATH || g_mcu.state.led_status == LED_SEQ_ENC) {
        esp_timer_stop(s_blink_timer);
    }
    s_selected_pattern = s_fast_breath_pattern;
    s_selected_pattern_len = sizeof(s_fast_breath_pattern);
    if (g_mcu.state.led_status == LED_ON || g_mcu.state.led_status == LED_OFF) {
        ledc_set_freq(g_blink_cfg.ledc_channel.speed_mode, LEDC_TIMER_0, CONFIG_BLINK_PWN_FREQ);
    }
    esp_timer_start_periodic(s_blink_timer, 50 * 1000);
    g_mcu.state.led_status = LED_FAST_BREATH;
}

/**
 * Function to blink the LED with slow breath pattern
**/
void blink_led_start_slow_breath_pattern() {
    if (g_mcu.state.led_status == LED_SLOW_BREATH) {
        return;
    }
    if (g_mcu.state.led_status == LED_FAST_FLASH || g_mcu.state.led_status == LED_FAST_BREATH || g_mcu.state.led_status == LED_SEQ_ENC) {
        esp_timer_stop(s_blink_timer);
    }
    s_selected_pattern = s_slow_breath_pattern;
    s_selected_pattern_len = sizeof(s_slow_breath_pattern);
    if (g_mcu.state.led_status == LED_ON || g_mcu.state.led_status == LED_OFF) {
        ledc_set_freq(g_blink_cfg.ledc_channel.speed_mode, LEDC_TIMER_0, CONFIG_BLINK_PWN_FREQ);
    }
    esp_timer_start_periodic(s_blink_timer, 200 * 1000);
    g_mcu.state.led_status = LED_SLOW_BREATH;
}

/**
 * Fill the buffer with hamming encoded sequence number
 * @param send_buf The send buffer
 * @param seq The sequence number [0-255]
 *
 * * Parity Check Mode: Even
**/
static void fill_u8_hamming(uint8_t *send_buf, uint8_t seq) {
    /** Fill sync bits **/
    uint8_t _hamming_idx[8] = {2, 4, 5, 6, 8, 9, 10, 11};

    for (int idx = 0; idx < 8; ++idx) {
        bool value = get_bit_from_byte_array(&seq, 7 - idx);
        ESP_LOGD(TAG, "value: %d\n", value);
        send_buf[_hamming_idx[idx]] = value;
    }

    send_buf[0] = ((send_buf[0] + send_buf[2] + send_buf[4] + send_buf[6] + send_buf[8] + send_buf[10]) % 2) ? 1 : 0;
    send_buf[1] = ((send_buf[1] + send_buf[2] + send_buf[5] + send_buf[6] + send_buf[9] + send_buf[10]) % 2) ? 1 : 0;
    send_buf[3] = ((send_buf[3] + send_buf[4] + send_buf[5] + send_buf[6] + send_buf[11]) % 2) ? 1 : 0;
    send_buf[7] = ((send_buf[7] + send_buf[8] + send_buf[9] + send_buf[10] + send_buf[11]) % 2) ? 1 : 0;
}

/**
 * Blinking callback
 * @param args
**/
static void IRAM_ATTR blink_timeout(void *args) {
    if (s_blink_idx >= s_selected_pattern_len) s_blink_idx = 0;
    ledc_set_duty(g_blink_cfg.ledc_channel.speed_mode, g_blink_cfg.ledc_channel.channel,
                  (s_selected_pattern[s_blink_idx] * CONFIG_BLINK_MAX_DUTY) / 0x100);
    ledc_update_duty(g_blink_cfg.ledc_channel.speed_mode, g_blink_cfg.ledc_channel.channel);
    s_blink_idx++;
}

/**
 * Prepare the sequence encoding pattern by filling the buffer
**/
static void blink_prepare_pattern() {
    ESP_LOGI(TAG, "blinking pin: %d", CONFIG_BLINK_DEFAULT_PIN);
    ESP_LOGI(TAG, "blinking sequence: %d", g_mcu.seq);
    s_blink_idx = 0;

    /** fill sync bits **/
    memcpy(s_seq_enc_pattern, (uint8_t[]) {255, 255, 255, 64}, CONFIG_BLINK_SYNC_SEQ_LEN);

    /** fill data bits **/
    uint8_t hamming_seq[12];
    fill_u8_hamming(hamming_seq, g_mcu.seq);
    for (int idx = 0; idx < (CONFIG_BLINK_SEQ_LEN / 2); ++idx) {
        int first = CONFIG_BLINK_SYNC_SEQ_LEN + idx * 2;
        int second = CONFIG_BLINK_SYNC_SEQ_LEN + idx * 2 + 1;
        s_seq_enc_pattern[first] = hamming_seq[idx] ? s_seq_enc_pattern[first - 1] : s_seq_enc_pattern[first - 2];
        s_seq_enc_pattern[second] = hamming_seq[idx] ? s_seq_enc_pattern[second - 3] : s_seq_enc_pattern[second - 2];
    }

    ESP_LOGW(TAG, "# -------- Begin of blink sequence -------- #");
    for (int idx = 0; idx < sizeof(s_seq_enc_pattern); ++idx) {
        printf("%d|", s_seq_enc_pattern[idx]);
    }
    printf("\n");
    ESP_LOGW(TAG, "# --------- End of blink sequence --------- #");

    /** Arm timers **/
    const esp_timer_create_args_t blink_timer_args = {
        .callback = &blink_timeout,
        /** name is optional, but may help identify the timer when debugging */
        .name = "timeout",
        .skip_unhandled_events = true,
    };

    ESP_ERROR_CHECK(esp_timer_create(&blink_timer_args, &s_blink_timer));
}

/**
 * Initialize the LED controller
**/
void blink_msp_init() {
    /**
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
    **/
    g_blink_cfg = (blink_config_t) {
        .en_val = 1,
        .pin_num = CONFIG_BLINK_PIN,
        .ledc_channel = {
            .channel = CONFIG_LEDC_HS_CH0_CHANNEL,
            .duty = 0,
            .gpio_num = CONFIG_BLINK_PIN,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .hpoint = 0,
            .timer_sel = CONFIG_LEDC_HS_TIMER
        }
    };

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = CONFIG_BLINK_PWN_FREQ,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = CONFIG_LEDC_HS_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    /** Set configuration of timer0 for high speed channels **/
    ledc_timer_config(&ledc_timer);
    ledc_channel_config(&g_blink_cfg.ledc_channel);
    ledc_set_freq(g_blink_cfg.ledc_channel.speed_mode, CONFIG_BLINK_TIMER_IDX, CONFIG_BLINK_PWN_FREQ);

    /** Prepare pattern **/
    blink_prepare_pattern();

    /** During init, test the led **/
    blink_led_fast_flash();
}

/**
 * Start the sequence encoding pattern
**/
void blink_start_seq_enc_pattern() {
    ESP_LOGI(TAG, "blink timer started");
    if (g_mcu.state.led_status == LED_SEQ_ENC) {
        return;
    }
    if (g_mcu.state.led_status == LED_FAST_FLASH || g_mcu.state.led_status == LED_FAST_BREATH || g_mcu.state.led_status == LED_SLOW_BREATH) {
        esp_timer_stop(s_blink_timer);
    }
    s_selected_pattern = s_seq_enc_pattern;
    s_selected_pattern_len = sizeof(s_seq_enc_pattern);
    if (g_mcu.state.led_status == LED_ON || g_mcu.state.led_status == LED_OFF) {
        ledc_set_freq(g_blink_cfg.ledc_channel.speed_mode, LEDC_TIMER_0, CONFIG_BLINK_PWN_FREQ);
    }
    esp_timer_start_periodic(s_blink_timer, CONFIG_BLINK_TIMER_INTERVAL_MS * 1000);
    g_mcu.state.led_status = LED_SEQ_ENC;
}

