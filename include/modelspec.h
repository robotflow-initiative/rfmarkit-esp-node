#ifndef MODELSPEC_H_
#define MODELSPEC_H_

#include "driver/gpio.h"
#include "driver/uart.h"

#define CONFIG_BUTTON_FN_GPIO_PIN           GPIO_NUM_0
#define CONFIG_BUTTON_FN_ACTIVE_LEVEL       0
#define CONFIG_BLINK_PIN                    GPIO_NUM_2

#define TIMER_DIVIDER                       16              //  Hardware timer clock divider
#define TIMER_SCALE                         (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds

/** If UART is used **/
#define CONFIG_IMU_CTRL_PIN                 GPIO_NUM_4      // Need RTC GPIO
#define CONFIG_IMU_RX_PIN                   GPIO_NUM_17
#define CONFIG_IMU_TX_PIN                   GPIO_NUM_16
#define CONFIG_IMU_SYNC_IN_PIN              GPIO_NUM_18
#define CONFIG_IMU_SYNC_OUT_PIN             GPIO_NUM_19
#define CONFIG_IMU_RESET_PIN                GPIO_NUM_15
#define CONFIG_IMU_UART_PORT                UART_NUM_1
#define CONFIG_IMU_BAUD                     115200

/** If SPI is used **/
#define CONFIG_IMU_WAKE_PIN                 GPIO_NUM_4    // Need RTC GPIO, OUTPUT
#define CONFIG_IMU_MOSI_PIN                 GPIO_NUM_17
#define CONFIG_IMU_MISO_PIN                 GPIO_NUM_18
#define CONFIG_IMU_SCLK_PIN                 GPIO_NUM_19
#define CONFIG_IMU_CS_PIN                   GPIO_NUM_16
#define CONFIG_IMU_INT_PIN                  GPIO_NUM_15     // Need RTC GPIO
#define CONFIG_IMU_RST_PIN                  GPIO_NUM_13     // Need RTC GPIO

#if defined(CONFIG_IDF_TARGET_ESP32S)
#define CONFIG_BATTERY_EN_PIN               GPIO_NUM_25
#define CONFIG_BATTERY_READ_ADC1_CHANNEL    ADC1_CHANNEL_6  //
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#define CONFIG_BATTERY_EN_PIN               GPIO_NUM_10
#define CONFIG_BATTERY_READ_ADC1_CHANNEL    ADC1_CHANNEL_9  //
#endif

#endif