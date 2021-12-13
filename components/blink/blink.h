#ifndef _BLINK_H
#define _BLINK_H

#include "device.h"

/** Blink settings **/
#define CONFIG_BLINK_INTERVAL_MS 100 // Blink interval in ms
#define CONFIG_BLINK_SEQ_LEN 16 // Blink sequence length
#define CONFIG_BLINK_TIMER_GROUP 0
#define CONFIG_BLINK_DEFAULT_PIN CONFIG_BLINK_BLUE_PIN
#define CONFIG_BLINK_TIMER_IDX 0
#define CONFIG_BLINK_NVS_TABLE_NAME "blink"
#include "pins.h"

/** @warning Dev board and product have diffenrent definition **/
#define CONFIG_BLINK_LED_ENABLE_VALUE 1 // set low to enable led
#define LED_ALLOFF() gpio_set_level(CONFIG_BLINK_RED_PIN, !CONFIG_BLINK_LED_ENABLE_VALUE);gpio_set_level(CONFIG_BLINK_GREEN_PIN, !CONFIG_BLINK_LED_ENABLE_VALUE);gpio_set_level(CONFIG_BLINK_BLUE_PIN, !CONFIG_BLINK_LED_ENABLE_VALUE)


void blink_init();
void blink_start();
void blink_stop();

COMMAND_FUNCTION(blink_set);
COMMAND_FUNCTION(blink_start);
COMMAND_FUNCTION(blink_stop);
COMMAND_FUNCTION(blink_get);
COMMAND_FUNCTION(blink_off); // TODO: Seperate blink functions

#endif