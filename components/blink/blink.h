#ifndef BLINK_H_
#define BLINK_H_

#include "sys.h"
#include "modelspec.h"
#include "driver/ledc.h"

/** Blink settings **/
#define CONFIG_BLINK_INTERVAL_MS 50 // Blink interval in ms
#define CONFIG_BLINK_SEQ_LEN 16 // Blink sequence length
#define CONFIG_BLINK_TIMER_GROUP 0
#define CONFIG_BLINK_DEFAULT_PIN CONFIG_BLINK_PIN
#define CONFIG_BLINK_TIMER_IDX 0
#define CONFIG_BLINK_NVS_TABLE_NAME "blink"

extern uint8_t g_blink_pin;
extern ledc_channel_config_t g_ledc_channel;

#ifdef CONFIG_BLINK_NO_PWM
#define CONFIG_BLINK_LED_ENABLE_VALUE 1 // set low to enable led
#define CONFIG_LED_ON() gpio_set_level(g_blink_pin, CONFIG_BLINK_LED_ENABLE_VALUE)
#define CONFIG_LED_OFF() gpio_set_level(g_blink_pin, !CONFIG_BLINK_LED_ENABLE_VALUE)
#else 
#define CONFIG_BLINK_MAX_DUTY 4000
#define CONFIG_BLINK_MIN_DUTY 2000
#define CONFIG_LED_ON() \
        ledc_set_duty(g_ledc_channel.speed_mode, g_ledc_channel.channel, CONFIG_BLINK_MAX_DUTY); \
        ledc_update_duty(g_ledc_channel.speed_mode, g_ledc_channel.channel)
#define CONFIG_LED_OFF() \
        ledc_set_duty(g_ledc_channel.speed_mode, g_ledc_channel.channel, 0); \
        ledc_update_duty(g_ledc_channel.speed_mode, g_ledc_channel.channel)
#endif

#define CONFIG_BLINK_SYNC_SEQ_LEN 4
#define CONFIG_BLINK_CHKSUM_SEQ_LEN 0

typedef struct {
    bool is_mono; // 1 - mono, only pin0, 0 - not mono, rgb
    bool is_binary; // 1 - mono, only pin0, 0 - not mono, rgb
    uint8_t en_val; // Value to enable, typically 0 - 1
    union {
        uint8_t mono;
        uint8_t rgb[3];
    } pins;
    union {
        uint8_t mono[3];
        uint8_t rgb[3];
    } gain;
} blink_led_t;

void blink_led_off(blink_led_t* led_t);

void blink_led_on(blink_led_t* led_t);

void blink_init();

void blink_init_task(void *);

void blink_start();

void blink_stop();

COMMAND_FUNCTION(blink_set);

COMMAND_FUNCTION(blink_start);

COMMAND_FUNCTION(blink_stop);

COMMAND_FUNCTION(blink_get);

COMMAND_FUNCTION(blink_off);
#endif