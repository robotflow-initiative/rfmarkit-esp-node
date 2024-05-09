#ifndef BLINK_H_
#define BLINK_H_

#include "sys.h"
#include "modelspec.h"
#include "driver/ledc.h"
#include "hal/ledc_types.h"

/** Blink settings **/
#define CONFIG_BLINK_TIMER_INTERVAL_MS  50  // Blink interval in ms
#define CONFIG_BLINK_SEQ_LEN            24  // Hamming code
#define CONFIG_BLINK_SYNC_SEQ_LEN       4   // Sync sequence length

#define CONFIG_BLINK_TIMER_GROUP        0
#define CONFIG_BLINK_DEFAULT_PIN        CONFIG_BLINK_PIN
#define CONFIG_BLINK_TIMER_IDX          LEDC_TIMER_0
#define CONFIG_LEDC_HS_TIMER            LEDC_TIMER_0
#define CONFIG_LEDC_HS_CH0_CHANNEL      LEDC_CHANNEL_0

#define CONFIG_BLINK_PWN_FREQ           4000
#define CONFIG_BLINK_MAX_DUTY           4000

typedef struct {
    uint8_t en_val;
    uint8_t pin_num;
    ledc_channel_config_t ledc_channel;
    ledc_timer_config_t ledc_timer;
} blink_config_t;

extern blink_config_t g_blink_cfg;

void blink_led_off(void);

void blink_led_on(void);

void blink_led_set_duty(uint32_t duty);

void blink_led_fast_flash(void);

void blink_led_start_fast_breath_pattern(void);

void blink_led_start_slow_breath_pattern(void);

void blink_msp_init(void);

void blink_start_seq_enc_pattern(void);

#endif