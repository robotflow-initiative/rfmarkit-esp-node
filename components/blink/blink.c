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


//static uint8_t s_blink_seq[CONFIG_BLINK_SYNC_SEQ_LEN + CONFIG_BLINK_SEQ_LEN + CONFIG_BLINK_CHKSUM_SEQ_LEN]; // /
static uint8_t *s_blink_seq;
static int s_blink_seq_len;
static int s_blink_idx; // Used in interrupt
uint8_t g_blink_pin; // Used in interrupt
static const char *TAG = "app_blink";
blink_config_t g_blink_cfg = {
        .is_mono = false,
        .is_binary = true,
        .en_val =1,
        .pins.mono = CONFIG_BLINK_DEFAULT_PIN,
        .gain.mono = UINT8_MAX,
        .ledc_channel = {
                .channel = CONFIG_LEDC_HS_CH0_CHANNEL,
                .duty = 0,
                .gpio_num = CONFIG_BLINK_PIN,
                .speed_mode = LEDC_HIGH_SPEED_MODE,
                .hpoint = 0,
                .timer_sel = CONFIG_LEDC_HS_TIMER
        }
};
static esp_timer_handle_t g_blink_timer;


static bool get_flag(const uint8_t *arr, int item) {
    return (arr[item / 8] & (1 << item % 8)) >> (item % 8);
}


void blink_gpio_off(blink_config_t *p_cfg) {
    if (p_cfg->is_mono) {
        gpio_set_level(p_cfg->pins.mono, !p_cfg->en_val);
    } else {
        for (int idx = 0; idx < sizeof(p_cfg->pins.rgb) / sizeof(p_cfg->pins.rgb[0]); ++idx) {
            gpio_set_level(p_cfg->pins.rgb[idx], !p_cfg->en_val);
        }
    }
}

void blink_gpio_on(blink_config_t *p_cfg) {
    if (p_cfg->is_mono) {
        gpio_set_level(p_cfg->pins.mono, p_cfg->en_val);
    } else {
        for (int idx = 0; idx < sizeof(p_cfg->pins.rgb) / sizeof(p_cfg->pins.rgb[0]); ++idx) {
            gpio_set_level(p_cfg->pins.rgb[idx], p_cfg->en_val);
        }
    }
}

void blink_pwm_off(blink_config_t *p_cfg) {
    ledc_set_duty(p_cfg->ledc_channel.speed_mode, p_cfg->ledc_channel.channel, 0);
    ledc_update_duty(p_cfg->ledc_channel.speed_mode, p_cfg->ledc_channel.channel);
    ledc_stop(p_cfg->ledc_channel.speed_mode, p_cfg->ledc_channel.channel, 0);
}

void blink_pwm_on(blink_config_t *p_cfg) {
    ledc_set_duty(p_cfg->ledc_channel.speed_mode, p_cfg->ledc_channel.channel, CONFIG_BLINK_MAX_DUTY);
    ledc_update_duty(p_cfg->ledc_channel.speed_mode, p_cfg->ledc_channel.channel);
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
    ledc_channel_config(&g_blink_cfg.ledc_channel);
}


static void blink_timeout(void *args) {
    if (s_blink_idx >= s_blink_seq_len) s_blink_idx = 0;
#if CONFIG_BLINK_NO_PWM
    gpio_set_level(g_blink_pin, s_blink_seq[s_blink_idx] ? 1 : 0);
#else
    ledc_set_duty(g_blink_cfg.ledc_channel.speed_mode, g_blink_cfg.ledc_channel.channel, s_blink_seq[s_blink_idx] ? CONFIG_BLINK_MAX_DUTY : CONFIG_BLINK_MIN_DUTY);
    ledc_update_duty(g_blink_cfg.ledc_channel.speed_mode, g_blink_cfg.ledc_channel.channel);
#endif
    s_blink_idx++;
}

/**
 * 
 * @param send_buf The send buffer
 * @param info The infomation
 * 
 * * Parity Check Mode: Even
 */
static void blink_fill_u8_hamming(uint8_t *send_buf, uint8_t info) {
    /** fill sync bits **/
    uint8_t _hamming_idx[8] = {2, 4, 5, 6, 8, 9, 10, 11};


    for (int idx = 0; idx < 8; ++idx) {
        bool value = get_flag(&info, 7 - idx);
        ESP_LOGD(TAG, "value: %d\n", value);
        send_buf[_hamming_idx[idx]] = value;
    }

    send_buf[0] = ((send_buf[0] + send_buf[2] + send_buf[4] + send_buf[6] + send_buf[8] + send_buf[10]) % 2) ? 1 : 0;
    send_buf[1] = ((send_buf[1] + send_buf[2] + send_buf[5] + send_buf[6] + send_buf[9] + send_buf[10]) % 2) ? 1 : 0;
    send_buf[3] = ((send_buf[3] + send_buf[4] + send_buf[5] + send_buf[6] + send_buf[11]) % 2) ? 1 : 0;
    send_buf[7] = ((send_buf[7] + send_buf[8] + send_buf[9] + send_buf[10] + send_buf[11]) % 2) ? 1 : 0;
}

void blink_init() {

#if CONFIG_BLINK_NO_PWM
    blink_gpio_msp_init();
#else
    blink_pwm_msp_init();
#endif

    /** Read blink sequence from nvs **/
    nvs_handle_t blink_handle;
    uint8_t seq;
    nvs_open(CONFIG_BLINK_NVS_TABLE_NAME, NVS_READWRITE, &blink_handle);

    g_blink_pin = CONFIG_BLINK_DEFAULT_PIN;
    nvs_get_u8(blink_handle, "seq", &seq);
    ESP_LOGI(TAG, "Blinking pin: %d", g_blink_pin);
    ESP_LOGI(TAG, "Blinking sequence: %d", seq);
    s_blink_idx = 0;

    s_blink_seq_len = CONFIG_BLINK_SYNC_SEQ_LEN + (g_mcu.use_hamming ? 24 : 16) + CONFIG_BLINK_CHKSUM_SEQ_LEN;
    s_blink_seq = (uint8_t *) calloc(s_blink_seq_len, 1);
    uint8_t pattern[2] = {1, 0};
    bool value = 0;

    ESP_LOGI(TAG, "Sequence Number: %d", seq);

    /** fill sync bits **/
    s_blink_seq[0] = 1;
    s_blink_seq[1] = 1;
    s_blink_seq[2] = 1;
    s_blink_seq[3] = 0;
    
    uint8_t hamming_seq[12];
    blink_fill_u8_hamming(hamming_seq, seq);

    /** fill data bits **/
    int n_ones = 0;

    for (int idx = 0; idx < (s_blink_seq_len / 2); ++idx) {
        if (g_mcu.use_hamming) {
            value = hamming_seq[idx];
        } else {
            value = get_flag(&seq, 7 - idx);
        }

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
    
    ESP_LOGW(TAG, "USE_HAMMING=%d", g_mcu.use_hamming);

    ESP_LOGW(TAG, "# -------- Begin of blink sequence -------- #");
    for (int idx = 0; idx < s_blink_seq_len; ++idx) {
        printf("%d ", s_blink_seq[idx]);
    }
    printf("\n");
    ESP_LOGW(TAG, "# --------- End of blink sequence --------- #");

    // Arm timers
    const esp_timer_create_args_t blink_timer_args = {
            .callback = &blink_timeout,
            /** name is optional, but may help identify the timer when debugging */
            .name = "timeout"
    };
    
    ESP_ERROR_CHECK(esp_timer_create(&blink_timer_args, &g_blink_timer));

}


void blink_init_task(void *vParameters) {
    blink_init();
    // blink_start();
    blink_stop();
    vTaskDelete(NULL);
}


void blink_start() {
    ESP_LOGI(TAG, "Timer started");
    ESP_LOGW(TAG, "# -------- Begin of blink sequence -------- #");
    for (int idx = 0; idx < s_blink_seq_len; ++idx) {
        printf("%d ", s_blink_seq[idx]);
    }
    printf("\n");
    ESP_LOGW(TAG, "# --------- End of blink sequence --------- #");
    ESP_ERROR_CHECK(esp_timer_start_periodic(g_blink_timer, CONFIG_BLINK_INTERVAL_MS * 1000));
    deice_always_on();
}

void blink_stop() {
    ESP_LOGI(TAG, "Timer stopped");
    esp_timer_stop(g_blink_timer);
    hal_led_on();
    deice_cancel_always_on();
    device_reset_sleep_countup();
}

void blink_off() {
    ESP_LOGI(TAG, "Timer stopped");
    esp_timer_stop(g_blink_timer);
    hal_led_off();
    deice_cancel_always_on();
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
    char *end_ptr = NULL;
    seq = (int) strtol(&rx_buffer[BLINK_SET_OFFSET], &end_ptr, 10);

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

COMMAND_FUNCTION(blink_off) {
    ESP_LOGI(TAG, "Executing command : IMU_BLINK_OFF");

    blink_off();

    snprintf(tx_buffer, tx_len, "Blink pin %d set to OFF\n\n", g_blink_pin);
    return ESP_OK;
}