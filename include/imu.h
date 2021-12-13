#ifndef _IMU_H
#define _IMU_H

#include "gy95.h"
#include "device.h"
#include "hi229.h"

typedef struct {
    int port;
    int ctrl_pin;
    int rx_pin;
    int tx_pin;
    int rtx_pin;
    int ctx_pin;
    int addr;
} imu_config_t;

#define CONFIG_USE_GY95 1

#if CONFIG_USE_GY95
#define CONFIG_IMU_NAME GY95
#define CONFIG_IMU_PAYLOAD_LEN CONFIG_GY95_PAYLOAD_LEN
#define CONFIG_IMU_CTRL_PIN CONFIG_GY95_CTRL_PIN
#define CONFIG_IMU_NVS_TABLE_NAME CONFIG_GY95_NVS_TABLE_NAME

#define imu_dgram_t gy95_dgram_t
#define imu_multiplier_t gy95_multiplier_t
#define imu_res_t gy95_res_t
#define imu_status_t gy95_status_t

#define imu_t gy95_t
extern imu_t g_imu;

#define imu_read(imu) \
        gy95_read((imu_t*)(imu))

#define imu_enable(imu) \
        gy95_enable((imu_t*)(imu))

#define imu_disable(imu) \
        gy95_disable((imu_t*)(imu))

#define imu_self_test(imu) \
        gy95_self_test((imu_t*)(imu))

#define imu_init(imu) \
        { \
            gy95_init(&imu, CONFIG_GY95_UART_PORT,  CONFIG_GY95_CTRL_PIN,        CONFIG_GY95_RX, CONFIG_GY95_TX,     CONFIG_GY95_RTS, CONFIG_GY95_CTS,      CONFIG_GY95_ADDR); \
            gy95_msp_init(&imu); \
            gy95_disable(&imu); \
            gy95_enable(&imu); \
        }


/** @brief func_parse **/
#define imu_parse(imu, reading, res, buffer, len) \
        gy95_parse((imu_t*) imu, \
                   (imu_dgram_t*) reading, \
                   (imu_res_t*) res, \
                   (char*) buffer, \
                   (int) len)

#define imu_tag(reading, payload, len) \
        gy95_tag((imu_dgram_t*) reading, \
                  (uint8_t *) payload, \
                  (int) len);

COMMAND_FUNCTION(imu_cali_reset); // EXTERNAL
COMMAND_FUNCTION(imu_cali_acc); // EXTERNAL
COMMAND_FUNCTION(imu_cali_gyro); // EXTERNAL
COMMAND_FUNCTION(imu_cali_mag); // EXTERNAL
COMMAND_FUNCTION(imu_enable); // EXTERNAL
COMMAND_FUNCTION(imu_disable); // EXTERNAL
COMMAND_FUNCTION(imu_status); // EXTERNAL
COMMAND_FUNCTION(imu_imm); // EXTERNAL
COMMAND_FUNCTION(imu_setup); // EXTERNAL
COMMAND_FUNCTION(imu_scale); // EXTERNAL

#elif CONFIG_HIPNUC
#define CONFIG_IMU_NAME HI229
#define CONFIG_IMU_PAYLOAD_LEN CONFIG_HI229_PAYLOAD_LEN
#define CONFIG_IMU_CTRL_PIN CONFIG_HI229_CTRL_PIN
#define CONFIG_IMU_NVS_TABLE_NAME CONFIG_HI229_NVS_TABLE_NAME

#define imu_dgram_t hip229_dgram_t
#define imu_multiplier_t hip229_multiplier_t
#define imu_res_t hip229_res_t
#define imu_status_t hip229_status_t

#define imu_t hip229_t
extern imu_t g_imu;

#define imu_read(imu) \
        hi229_read((imu_t*)(imu))

#define imu_enable(imu) \
        hi229_enable((imu_t*)(imu))

#define imu_disable(imu) \
        hi229_disable((imu_t*)(imu))

#define imu_self_test(imu) \
        hi229_self_test((imu_t*)(imu))

#define imu_init(imu) \
        { \
            hi229_init(&imu, CONFIG_GY95_UART_PORT,  CONFIG_GY95_CTRL_PIN,        CONFIG_GY95_RX, CONFIG_GY95_TX,     CONFIG_GY95_RTS, CONFIG_GY95_CTS,      CONFIG_GY95_ADDR); \
            hi229_msp_init(&imu); \
            hi229_disable(&imu); \
            hi229_enable(&imu); \
        }


/** @brief func_parse **/
#define imu_parse(imu, reading, res, buffer, len) \
        hi229_parse((imu_t*) imu, \
                   (imu_dgram_t*) reading, \
                   (imu_res_t*) res, \
                   (char*) buffer, \
                   (int) len)

#define imu_tag(reading, payload, len) \
        hi229_tag((imu_dgram_t*) reading, \
                  (uint8_t *) payload, \
                  (int) len);

COMMAND_FUNCTION(imu_cali_reset); // EXTERNAL
COMMAND_FUNCTION(imu_cali_acc); // EXTERNAL
COMMAND_FUNCTION(imu_cali_gyro); // EXTERNAL
COMMAND_FUNCTION(imu_cali_mag); // EXTERNAL
COMMAND_FUNCTION(imu_enable); // EXTERNAL
COMMAND_FUNCTION(imu_disable); // EXTERNAL
COMMAND_FUNCTION(imu_status); // EXTERNAL
COMMAND_FUNCTION(imu_imm); // EXTERNAL
COMMAND_FUNCTION(imu_setup); // EXTERNAL
COMMAND_FUNCTION(imu_scale); // EXTERNAL
```
#endif