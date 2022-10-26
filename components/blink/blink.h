#ifndef BLINK_H_
#define BLINK_H_

#include "sys.h"
#include "modelspec.h"
#include "driver/ledc.h"

/** Blink settings **/
#define CONFIG_BLINK_INTERVAL_MS 50 // Blink interval in ms

#define CONFIG_USE_HAMMING 0
#if CONFIG_USE_HAMMING
#define CONFIG_BLINK_SEQ_LEN 24
#else
#define CONFIG_BLINK_SEQ_LEN 16 // Blink sequence length
#endif
#define CONFIG_BLINK_SYNC_SEQ_LEN 4
#define CONFIG_BLINK_CHKSUM_SEQ_LEN 0

#define CONFIG_BLINK_TIMER_GROUP 0
#define CONFIG_BLINK_DEFAULT_PIN CONFIG_BLINK_PIN
#define CONFIG_BLINK_TIMER_IDX 0
#define CONFIG_LEDC_HS_TIMER          LEDC_TIMER_0
#define CONFIG_LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0

#define CONFIG_BLINK_NVS_TABLE_NAME "blink"

typedef struct {
    uint8_t en_val;
    uint8_t pin_num;
    ledc_channel_config_t ledc_channel;
    ledc_timer_config_t ledc_timer;
} blink_config_t;

extern uint8_t g_blink_pin;
//extern ledc_channel_config_t g_ledc_channel;
extern blink_config_t g_blink_cfg;

void blink_off(blink_config_t *p_cfg);

void blink_on(blink_config_t *p_cfg);

#define hal_led_on()    blink_on(&g_blink_cfg)
#define hal_led_off()   blink_off(&g_blink_cfg)

#if CONFIG_BLINK_USE_PWM
#define CONFIG_BLINK_LED_ENABLE_VALUE 1
#define CONFIG_BLINK_MAX_DUTY 4000
#define CONFIG_BLINK_MIN_DUTY 2000
#else
#define CONFIG_BLINK_LED_ENABLE_VALUE 1 // set low to enable led
#endif

void blink_msp_init(void);

void blink_init(void);

void blink_init_task_0(void *);

void blink_init_task_1(void *);

void blink_start(void);

void blink_stop(void);

COMMAND_FUNCTION(blink_set);

COMMAND_FUNCTION(blink_get);

COMMAND_FUNCTION(blink_start);

COMMAND_FUNCTION(blink_stop);

COMMAND_FUNCTION(blink_mute);

#endif