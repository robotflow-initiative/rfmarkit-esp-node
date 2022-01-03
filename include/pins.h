#ifndef _PIN_H
#define _PIN_H

#include "driver/gpio.h"

#define TYPE_ESP_C3_32S 32
#define TYPE_ESP_C3_12F 12

#define CONFIG_ESP_C3_TYPE TYPE_ESP_C3_32S
#if CONFIG_ESP_C3_TYPE == TYPE_ESP_C3_32S

#define CONFIG_BUTTON_GPIO_PIN GPIO_NUM_0

#define CONFIG_BLINK_RED_PIN GPIO_NUM_2
#define CONFIG_BLINK_GREEN_PIN GPIO_NUM_2
#define CONFIG_BLINK_BLUE_PIN GPIO_NUM_2 // TODO: Remove redundeant pins

#elif CONFIG_ESP_C3_TYPE == TYPE_ESP_C3_12F

#endif
#endif