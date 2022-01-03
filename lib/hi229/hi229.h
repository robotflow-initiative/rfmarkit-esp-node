#ifndef _HI229_H
#define _HI229_H

#include <driver/uart.h>
#include <driver/gpio.h>

#include "device.h"
#include "hi229_serial.h" // FIXME Replace with custom solution


/** HI229 related settings **/
#define CONFIG_HI229_PAYLOAD_LEN 512
#define CONFIG_HI229_NVS_TABLE_NAME "hi229_scale"
#define CONFIG_HI229_UART_RX_BUF_LEN 4096
#define CONFIG_HI229_ADDR 0xe5


/** Modify this section to adapt different board **/
#define CONFIG_HI229_CTRL_PIN GPIO_NUM_4// FIXME: GPIO_NUM_5 cause reset, should be GPIO_NUM_4
#define CONFIG_HI229_CTRL_PIN_MASK (1ULL << CONFIG_HI229_CTRL_PIN)
#define CONFIG_HI229_RX GPIO_NUM_17
#define CONFIG_HI229_TX GPIO_NUM_16
#define CONFIG_HI229_RTS UART_PIN_NO_CHANGE
#define CONFIG_HI229_CTS UART_PIN_NO_CHANGE
#define CONFIG_HI229_UART_PORT (1) // UART_NUM_X

#define CONFIG_HI229_DEFAULT_FREQ 100
#define CONFIG_HI229_DEFAULT_BAUDRATE 921600
#define CONFIG_HI229_MAX_CHECK_TICKS 1024
#define CONFIG_HI229_RETRY_N 10
#define CONFIG_HI229_PAYLOAD_HEAD_IDX 4
#define CONFIG_HI229_PAYLOAD_TAIL_IDX 31

typedef struct {
        size_t n_bytes;
        uint8_t data[CONFIG_HI229_PAYLOAD_LEN];
        int64_t time_us;
        int64_t start_time_us;
        int uart_buffer_len;
} hi229_dgram_t; // EXTERNAL

typedef struct {
        float accel[3];
        float gyro[3];
        float mag[3];
        float rpy[3];
        float quat[4];
        float pressure;
        uint32_t timestamp;
} hi229_multiplier_t, hi229_data_t; // INTERNAL

typedef enum {
        HI229_OK,
        HI229_RECV_COMPLETE,
        HI229_FAIL,
        HI229_READY,
} hi229_status_t;

typedef struct {
        int port;

        /** pin configuration **/
        int ctrl_pin;
        int rx_pin;
        int tx_pin;
        int rts_pin;
        int cts_pin;

        uint8_t addr;
        hi229_status_t status;

        SemaphoreHandle_t mux;

        raw_t raw;
        size_t n_bytes; /* number of bytes in message buffer */
        size_t len; /* message length (bytes) */
        uint8_t buf[CONFIG_HI229_PAYLOAD_LEN];  /* message raw buffer */

        hi229_data_t data;
        uint8_t item_code[8]; /* item code recv in one frame */
        uint8_t nitem_code;   /* # of item code */

} hi229_t;

void hi229_msp_init(hi229_t* p_gy);

void hi229_init(hi229_t* p_gy,
                int port,
                int ctrl_pin,
                int rx_pin,
                int tx_pin,
                int rts_pin,
                int cts_pin,
                int addr
);

uint8_t hi229_setup(hi229_t* p_gy);

esp_err_t hi229_read(hi229_t* p_gy);

void hi229_enable(hi229_t* p_gy);

void hi229_disable(hi229_t* p_gy);

esp_err_t hi229_self_test(hi229_t* p_gy);

esp_err_t hi229_parse(hi229_t* p_gy,
                      hi229_dgram_t* p_reading,
                      hi229_data_t* p_parsed,
                      char* buffer, int len);

int hi229_tag(hi229_dgram_t* p_reading, uint8_t* payload, int len);

/** Exposed API **/

#define CONFIG_IMU_NAME HI229
#define CONFIG_IMU_PAYLOAD_LEN CONFIG_HI229_PAYLOAD_LEN
#define CONFIG_IMU_CTRL_PIN CONFIG_HI229_CTRL_PIN
#define CONFIG_IMU_NVS_TABLE_NAME CONFIG_HI229_NVS_TABLE_NAME
#define CONFIG_IMU_SUPPORT_SCALE 0

#define imu_dgram_t hi229_dgram_t
#define imu_multiplier_t hi229_multiplier_t
#define imu_res_t hi229_res_t
#define imu_status_t hi229_status_t

#define imu_t hi229_t
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
            hi229_init(&imu, \
                        CONFIG_HI229_UART_PORT, \
                        CONFIG_HI229_CTRL_PIN, \
                        CONFIG_HI229_RX, \
                        CONFIG_HI229_TX, \
                        CONFIG_HI229_RTS, \
                        CONFIG_HI229_CTS, \
                        CONFIG_HI229_ADDR); \
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
COMMAND_FUNCTION(imu_self_test);
#endif