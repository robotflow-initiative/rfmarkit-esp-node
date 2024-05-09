#ifndef HI229_H_
#define HI229_H_

#include <driver/uart.h>
#include <driver/gpio.h>

#include "sys.h"
#include "modelspec.h"
#include "hi229_serial.h"


/** HI229 related settings **/
#define CONFIG_HI229_PAYLOAD_LEN 80
#define CONFIG_HI229_UART_RX_BUF_LEN 1024 // 4096
#define CONFIG_HI229_UART_TX_BUF_LEN 0
#define CONFIG_HI229_ADDR 0xe5
#define CONFIG_HI229_ENABLE_LVL 0
#define CONFIG_HI229_DISABLE_LVL 1
#define CONFIG_HI229_SELF_TEST_RETRY 4

/** Modify this section to adapt different board **/
#define CONFIG_HI229_CTRL_PIN CONFIG_IMU_CTRL_PIN
#define CONFIG_HI229_CTRL_PIN_MASK (1ULL << CONFIG_HI229_CTRL_PIN)
#define CONFIG_HI229_RX CONFIG_IMU_RX_PIN
#define CONFIG_HI229_TX CONFIG_IMU_TX_PIN
#define CONFIG_HI229_RTS UART_PIN_NO_CHANGE
#define CONFIG_HI229_CTS UART_PIN_NO_CHANGE
#define CONFIG_HI229_UART_PORT CONFIG_IMU_UART_PORT

#define CONFIG_HI229_DEFAULT_FREQ 100
#define CONFIG_HI229_DEFAULT_BAUD_RATE 115200
#define CONFIG_HI229_MAX_CHECK_TICKS 1024
#define CONFIG_HI229_RETRY_N 10
#define CONFIG_HI299_MAX_READ_NUM 512
#define CONFIG_HI299_BLOCK_READ_NUM 64

typedef struct {
    ch_imu_data_t imu[1];
    int64_t time_us;
    int64_t tsf_time_us;
    uint32_t seq;
    int uart_buffer_len;
} hi229_dgram_t; // EXTERNAL

typedef enum {
    IMU_STATUS_UNKNOWN = -1,
    IMU_STATUS_FAIL,
    IMU_STATUS_READY,
} hi229_status_t;

typedef enum {
    IMU_MUX_IDLE,
    IMU_MUX_STREAM,
    IMU_MUX_DEBUG
} hi229_mux_t;

typedef struct {
    int port;
    int baud;

    /** pin configuration **/
    int ctrl_pin;
    int rx_pin;
    int tx_pin;
    int rts_pin;
    int cts_pin;

    uint8_t addr;
    hi229_status_t status;
    bool enabled;

    hi229_mux_t mux;
    SemaphoreHandle_t mutex;
    raw_t raw;

} hi229_t;

void hi229_init(hi229_t *p_gy,
                int port,
                int baud,
                int ctrl_pin,
                int rx_pin,
                int tx_pin,
                int addr
);

esp_err_t hi229_read(hi229_t *p_gy, hi229_dgram_t *out);

void hi229_enable(hi229_t *p_gy);

void hi229_disable(hi229_t *p_gy);

esp_err_t hi229_self_test(hi229_t *p_gy);

int hi229_tag(hi229_dgram_t *p_reading, uint8_t *payload, int len);

void hi229_reset(hi229_t *p_gy);

/** Exposed API **/
#define CONFIG_IMU_NAME "HI229"

#define imu_dgram_t hi229_dgram_t
#define imu_status_t hi229_status_t
#define imu_mux_t hi229_mux_t

#define imu_t hi229_t
extern imu_t g_imu;

#define imu_read(imu, out) \
        hi229_read((imu_t*)(imu), (imu_dgram_t*)(out))

#define imu_enable(imu) \
        hi229_enable((imu_t*)(imu))

#define imu_reset(imu) \
        hi229_reset((imu_t*)(imu))

#define imu_disable(imu) \
        hi229_disable((imu_t*)(imu))

#define imu_self_test(imu) \
        hi229_self_test((imu_t*)(imu))

#define imu_init(imu) \
        { \
            hi229_init(&(imu), \
                        CONFIG_HI229_UART_PORT, \
                        g_mcu.imu_baud, \
                        CONFIG_HI229_CTRL_PIN, \
                        CONFIG_HI229_RX, \
                        CONFIG_HI229_TX, \
                        CONFIG_HI229_ADDR); \
        }NULL
#endif
