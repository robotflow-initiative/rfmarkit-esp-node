#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/timer.h"
#include "nvs_flash.h"

#include "cJSON.h"

#include "apps.h"
#include "blink.h"
#include "device.h"

#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define LED_ON() gpio_set_level(g_blink_pin, CONFIG_BLINK_LED_ENABLE_VALUE)
#define LED_OFF() gpio_set_level(g_blink_pin, !CONFIG_BLINK_LED_ENABLE_VALUE)

#define CONFIG_BLINK_SYNC_SEQ_LEN 4
#ifdef CONFIG_BLINK_EN_CHKSUM
#define CONFIG_BLINK_CHKSUM_SEQ_LEN 4
#else 
#define CONFIG_BLINK_CHKSUM_SEQ_LEN 0
#endif

static char s_blink_seq[CONFIG_BLINK_SYNC_SEQ_LEN + CONFIG_BLINK_SEQ_LEN + CONFIG_BLINK_CHKSUM_SEQ_LEN];
// [ 4bit sync | 16 bit data | 4 bit chksum]

static int s_blink_idx;
uint8_t g_blink_pin;

static const char* TAG = "app_blink";

static bool blink_timeout(void* args) {
    if (s_blink_idx >= CONFIG_BLINK_SEQ_LEN) s_blink_idx = 0;
    gpio_set_level(g_blink_pin, s_blink_seq[s_blink_idx] ? 1 : 0);
    s_blink_idx++;
    // printf("%d", s_blink_seq[s_blink_idx]);

    return false;
}

static bool get_flag(uint8_t* arr, int item) {
    return (arr[item / 8] & (1 << item % 8)) >> (item % 8);
}

void blink_init() {
    /** Read blink sequence from nvs **/
    nvs_handle_t blink_handle;
    uint8_t seq;
    nvs_open(CONFIG_BLINK_NVS_TABLE_NAME, NVS_READWRITE, &blink_handle);

    /** FIXME: Temporarily suppress pin config
    nvs_get_u8(blink_handle, "pin", &g_blink_pin);
    if (g_blink_pin != CONFIG_BLINK_BLUE_PIN && g_blink_pin != CONFIG_BLINK_GREEN_PIN && g_blink_pin != CONFIG_BLINK_RED_PIN) {
        g_blink_pin = CONFIG_BLINK_DEFAULT_PIN;
        nvs_set_u8(blink_handle, "pin", g_blink_pin);
        nvs_commit(blink_handle);
    }
    **/
    g_blink_pin = CONFIG_BLINK_DEFAULT_PIN;
    nvs_get_u8(blink_handle, "seq", &seq);
    ESP_LOGI(TAG, "Blinking pin: %d", g_blink_pin);
    ESP_LOGI(TAG, "Blinking sequence: %d", seq);
    s_blink_idx = 0;

    bzero(s_blink_seq, sizeof(s_blink_seq));
    bool pattern[2] = { 1,0 };
    bool value = 0;

    /** Init GPIO **/
    gpio_config_t io_config = {
        .pin_bit_mask = (1ull << CONFIG_BLINK_RED_PIN) | (1ull << CONFIG_BLINK_GREEN_PIN) | (1ull << CONFIG_BLINK_BLUE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_config);

    /** After init, light up the led **/
    LED_ALLOFF();
    LED_ON();

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

void blink_start() {
    ESP_LOGI(TAG, "Timer started");
    ESP_LOGW(TAG, "# -------- Begin of blink sequence -------- #");
    for (int idx = 0; idx < CONFIG_BLINK_SEQ_LEN; ++idx) {
        printf("%d ", s_blink_seq[idx]);
    }
    printf("\n");
    ESP_LOGW(TAG, "# --------- End of blink sequence --------- #");
    timer_start(CONFIG_BLINK_TIMER_GROUP, CONFIG_BLINK_TIMER_IDX);
}

void blink_stop() {
    ESP_LOGI(TAG, "Timer stopped");
    timer_pause(CONFIG_BLINK_TIMER_GROUP, CONFIG_BLINK_TIMER_IDX);
    LED_ON();
}

COMMAND_FUNCTION(blink_set) {
    esp_err_t err = ESP_OK;
    ESP_LOGI(TAG, "Executing command : IMU_BLINK_SET");
    uint8_t seq = 0;
    uint8_t pin = CONFIG_BLINK_DEFAULT_PIN;
    int offset = 0;

    /** Open nvs table **/
    nvs_handle_t blink_handle;
    ESP_ERROR_CHECK(nvs_open(CONFIG_BLINK_NVS_TABLE_NAME, NVS_READWRITE, &blink_handle));

    /**
    rx_buffer = "blink_set {"pin":"[R|G|B|r|g|b]","seq":"[0-255]"}
    rx_buffer = "blink_set {"seq":"[0-255]"}
    **/

    cJSON* pRoot = cJSON_Parse(rx_buffer + sizeof("blink_set"));
    cJSON* pSeq = NULL;
    cJSON* pPin = NULL;
    if (pRoot == NULL) {
        ESP_LOGE(TAG, "Parse failed");
        err = ESP_FAIL;
        goto blink_set_cleanup;
    } else {
        pPin = cJSON_GetObjectItem(pRoot, "pin");
        pSeq = cJSON_GetObjectItem(pRoot, "seq");
        if ((pSeq == NULL)) {
            ESP_LOGE(TAG, "Parse failed, invalid keys");
            err = ESP_FAIL;
            goto blink_set_cleanup;
        }
    }

    if (pPin != NULL) {
        if (cJSON_IsString(pPin)) {
            switch (pPin->valuestring[0]) {
            case 'R':
            case 'r':
                pin = CONFIG_BLINK_RED_PIN;
                snprintf(tx_buffer + offset, tx_len - offset, "Blink pin set to RED_PIN; ");
                offset = strlen(tx_buffer);
                break;
            case 'G':
            case 'g':
                pin = CONFIG_BLINK_GREEN_PIN;
                snprintf(tx_buffer + offset, tx_len - offset, "Blink pin set to GREEN_PIN; ");
                offset = strlen(tx_buffer);
                break;
            case 'B':
            case 'b':
                pin = CONFIG_BLINK_BLUE_PIN;
                snprintf(tx_buffer + offset, tx_len - offset, "Blink pin set to BLUE_PIN; ");
                offset = strlen(tx_buffer);
                break;
            default:
                snprintf(tx_buffer + offset, tx_len - offset, "Blink set failed\n\n");
                offset = strlen(tx_buffer);
                err = ESP_FAIL;
                goto blink_set_cleanup;
            }
        } else if (cJSON_IsNumber(pPin)) {
            pin = pPin->valueint;
            if (GPIO_IS_VALID_GPIO(pin)) {
                snprintf(tx_buffer + offset, tx_len - offset, "Blink pin set to %d; ");
                offset = strlen(tx_buffer);
                err = ESP_FAIL;
                goto blink_set_cleanup;
            } else {
                snprintf(tx_buffer + offset, tx_len - offset, "Blink set failed, invalid gpio\n\n");
                offset = strlen(tx_buffer);
            }
        } else {
            ESP_LOGI(TAG, "Parse failed, invalid pin");
            err = ESP_FAIL;
            goto blink_set_cleanup;
        }
        nvs_set_u8(blink_handle, "pin", pin);
        nvs_commit(blink_handle);

    }


    if (cJSON_IsNumber(pSeq)) {
        seq = pSeq->valueint % 0x100;
        snprintf(tx_buffer + offset, tx_len - offset, "Blink seq set to %d, reboot to make effective\n\n", seq);
    } else {
        snprintf(tx_buffer + offset, tx_len - offset, "Blink set failed, invalid seq\n\n");
        offset = strlen(tx_buffer);
        err = ESP_FAIL;
        goto blink_set_cleanup;
    }
    nvs_set_u8(blink_handle, "seq", seq);
    nvs_commit(blink_handle);

    err = ESP_OK;

blink_set_cleanup:

    nvs_commit(blink_handle);
    nvs_close(blink_handle);

    if (pRoot != NULL) {
        cJSON_free(pRoot);
    }
    return err;
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
    uint8_t pin = 0;

    /** Open nvs table **/
    nvs_handle_t blink_handle;
    nvs_open("blink", NVS_READWRITE, &blink_handle);

    nvs_get_u8(blink_handle, "pin", &pin);
    nvs_get_u8(blink_handle, "seq", &seq);

    snprintf(tx_buffer, tx_len, "Blink pin is %d, seq is %d, \n\n", pin, seq);

    nvs_close(blink_handle);
    return ESP_OK;
}

COMMAND_FUNCTION(blink_off) {
    ESP_LOGI(TAG, "Executing command : IMU_BLINK_OFF");

    blink_stop();

    gpio_set_level(g_blink_pin, !CONFIG_BLINK_LED_ENABLE_VALUE);

    snprintf(tx_buffer, tx_len, "Blink pin %d set to OFF\n\n", g_blink_pin);

    return ESP_OK;
}
