#ifndef MODELSPEC_H_
#define MODELSPEC_H_

#include "driver/gpio.h"
#include "driver/uart.h"

#define TYPE_ESP_32S 32
#define TYPE_ESP_C3_32S 132
#define TYPE_ESP_C3_12F 112

/****
 * Define chip model here
 */
#define CONFIG_ESP_C3_TYPE TYPE_ESP_32S

#if CONFIG_ESP_C3_TYPE == TYPE_ESP_C3_32S

#define CONFIG_BUTTON_GPIO_PIN GPIO_NUM_0
#define CONFIG_BLINK_PIN GPIO_NUM_2
#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds

#elif CONFIG_ESP_C3_TYPE == TYPE_ESP_C3_12F

#define CONFIG_BUTTON_GPIO_PIN GPIO_NUM_0
#define CONFIG_BLINK_PIN GPIO_NUM_2
#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds

#elif CONFIG_ESP_C3_TYPE == TYPE_ESP_32S

#define CONFIG_BUTTON_GPIO_PIN GPIO_NUM_0
#define CONFIG_BLINK_PIN GPIO_NUM_2
#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds

#define CONFIG_IMU_CTRL_PIN GPIO_NUM_4
#define CONFIG_IMU_RX_PIN GPIO_NUM_17
#define CONFIG_IMU_TX_PIN GPIO_NUM_16
#define CONFIG_IMU_UART_PORT UART_NUM_1
#define CONFIG_IMU_BAUD 115200
#endif

#endif