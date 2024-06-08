#ifndef _HI229_H
#define _HI229_H

#include "driver/gpio.h"

#include "imu.h"
#include "modelspec.h"

/** HI229 related settings **/
#define CONFIG_HI229_PAYLOAD_LEN        82
#define CONFIG_HI229_UART_RX_BUF_LEN    5120 // 4096
#define CONFIG_HI229_UART_TX_BUF_LEN    0
#define CONFIG_HI229_ADDR               0xe5
#define CONFIG_HI229_ENABLE_LVL         0
#define CONFIG_HI229_DISABLE_LVL        1
#define CONFIG_HI229_SELF_TEST_RETRY    4

/** Modify this section to adapt different board **/
#define CONFIG_HI229_CTRL_PIN           CONFIG_IMU_CTRL_PIN
#define CONFIG_HI229_RX                 CONFIG_IMU_RX_PIN
#define CONFIG_HI229_TX                 CONFIG_IMU_TX_PIN
#define CONFIG_HI229_SYNC_IN            CONFIG_IMU_SYNC_IN_PIN
#define CONFIG_HI229_SYNC_OUT           CONFIG_IMU_SYNC_OUT_PIN
#define CONFIG_HI229_RTS                UART_PIN_NO_CHANGE
#define CONFIG_HI229_CTS                UART_PIN_NO_CHANGE
#define CONFIG_HI229_UART_PORT          CONFIG_IMU_UART_PORT

/** HI229 related settings **/
#define CONFIG_HI299_MAX_READ_NUM       512
#define CONFIG_HI299_BLOCK_READ_NUM     32

typedef struct {
    imu_config_t base;

    /** uart configuration **/
    int port;
    int baud;

    /** pin configuration **/
    gpio_num_t ctrl_pin;
    gpio_num_t rx_pin;
    gpio_num_t tx_pin;
    gpio_num_t rts_pin;
    gpio_num_t cts_pin;
    gpio_num_t sync_in_pin;
    gpio_num_t sync_out_pin;
} hi229_config_t;

/** Exposed API **/
#define imu_interface_init_external(imu)                    \
    {                                                       \
        hi229_config_t cfg = {                              \
            .port = CONFIG_HI229_UART_PORT,                 \
            .baud = g_mcu.imu_baud,                         \
            .ctrl_pin = CONFIG_HI229_CTRL_PIN,              \
            .rx_pin = CONFIG_HI229_RX,                      \
            .tx_pin = CONFIG_HI229_TX,                      \
            .rts_pin = CONFIG_HI229_RTS,                    \
            .cts_pin = CONFIG_HI229_CTS,                    \
            .sync_in_pin = CONFIG_HI229_SYNC_IN,            \
            .sync_out_pin = CONFIG_HI229_SYNC_OUT,          \
        };                                                  \
        imu_interface_init((imu), (imu_config_t*) &cfg);    \
    }NULL

#endif
