#ifndef BLINK_H_
#define BLINK_H_

#include "sys.h"
#include "modelspec.h"
#include "driver/ledc.h"

/** Blink settings **/
#define CONFIG_BLINK_INTERVAL_MS 50 // Blink interval in ms
//#define CONFIG_USE_HAMMING 1
//#if CONFIG_USE_HAMMING
//#define CONFIG_BLINK_SEQ_LEN 24
//#else
//#define CONFIG_BLINK_SEQ_LEN 16 // Blink sequence length
//#endif
#define CONFIG_BLINK_TIMER_GROUP 0
#define CONFIG_BLINK_DEFAULT_PIN CONFIG_BLINK_PIN
#define CONFIG_BLINK_TIMER_IDX 0
#define CONFIG_BLINK_NVS_TABLE_NAME "blink"
#define CONFIG_BLINK_SYNC_SEQ_LEN 4
#define CONFIG_BLINK_CHKSUM_SEQ_LEN 0
#define CONFIG_LEDC_HS_TIMER          LEDC_TIMER_0
#define CONFIG_LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0

typedef struct {
    bool is_mono; // 1 - mono, only pin0, 0 - not mono, rgb
    bool is_binary; // 1 - mono, only pin0, 0 - not mono, rgb
    uint8_t en_val; // Value to enable, typically 0 - 1
    union {
        uint8_t mono;
        uint8_t rgb[3];
    } pins;
    union {
        uint8_t mono;
        uint8_t rgb[3];
    } gain;
    ledc_channel_config_t ledc_channel;
} blink_config_t;

extern uint8_t g_blink_pin;
//extern ledc_channel_config_t g_ledc_channel;
extern blink_config_t g_blink_cfg;

void blink_gpio_off(blink_config_t *p_cfg);

void blink_gpio_on(blink_config_t *p_cfg);

void blink_pwm_off(blink_config_t *p_cfg);

void blink_pwm_on(blink_config_t *p_cfg);

#ifdef CONFIG_BLINK_NO_PWM
#define CONFIG_BLINK_LED_ENABLE_VALUE 1 // set low to enable led
#define hal_led_on() blink_gpio_on(&g_blink_led)
#define hal_led_off()  blink_gpio_off(&g_blink_led)
#else
#define CONFIG_BLINK_LED_ENABLE_VALUE 1
#define CONFIG_BLINK_MAX_DUTY 4000
#define CONFIG_BLINK_MIN_DUTY 2000
#define hal_led_on()  blink_pwm_on(&g_blink_cfg)
#define hal_led_off() blink_pwm_off(&g_blink_cfg)
#endif

void blink_init();

void blink_init_task(void *);

void blink_start();

void blink_stop();

void blink_off();

COMMAND_FUNCTION(blink_set);

COMMAND_FUNCTION(blink_get);

COMMAND_FUNCTION(blink_start);

COMMAND_FUNCTION(blink_stop);

COMMAND_FUNCTION(blink_off);

#endif