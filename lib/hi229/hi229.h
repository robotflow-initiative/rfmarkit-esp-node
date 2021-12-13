#ifndef _HI229_H
#define _HI229_H

#include <driver/uart.h>
#include <driver/gpio.h>


#define CONFIG_HI229_PAYLOAD_LEN 100
#define CONFIG_HI229_DEFAULT_FREQ 200
#define CONFIG_HI229_CTRL_PIN GPIO_NUM_5
#define CONFIG_HI229_NVS_TABLE_NAME "hi229"

typedef struct {
    uint8_t data[CONFIG_HI229_PAYLOAD_LEN];
    int64_t time_us;
    int64_t start_time_us;
    int uart_buffer_len;
} hi229_dgram_t; // EXTERNAL

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float roll;
    float pitch;
    float yaw;
    float temp;
    float mag_x;
    float mag_y;
    float mag_z;
} hi229_multiplier_t, hi229_res_t; // INTERNAL

typedef enum {
    HI229_OK,
    HI229_RECV_COMPLETE,
    HI229_FAIL,
    HI229_READY,
} hi229_status_t;

typedef struct {
    int port;

    int ctrl_pin;
    int rx_pin;
    int tx_pin;
    int rts_pin;
    int cts_pin;

    uint8_t addr;
    int cursor;
    int start_reg;
    int length;
    hi229_status_t status;

    uint8_t acc_scale;
    uint8_t gyro_scale;
    uint8_t mag_scale;
    uint8_t scale;

    SemaphoreHandle_t mux;

    uint8_t buf[CONFIG_HI229_PAYLOAD_LEN];

} hi229_t;


#endif