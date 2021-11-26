#ifndef _GY95_H
#define _GY95_H

#include "esp_system.h"
#include "freertos/semphr.h"
#include "settings.h"

/** GY95 related settings **/
#define CONFIG_GY95_CTRL_MSG_LEN 4
#define CONFIG_GY95_PAYLOAD_LEN 32
#define CONFIG_GY95_CTRL_PIN GPIO_NUM_5
#define CONFIG_GY95_CTRL_PIN_MASK (1ULL << GY95_CTRL_PIN)
#define CONFIG_GY95_ADDR 0xa4
#define CONFIG_GY95_READ_OP 0x03
#define CONFIG_GY95_REG_THRESH 0x2c
#define CONFIG_GY95_DEFAULT_START_REG 0x14
#define CONFIG_GY95_UART_PORT (1) // UART_NUM_X
#define CONFIG_GY95_RX GPIO_NUM_6
#define CONFIG_GY95_TX GPIO_NUM_7
#define CONFIG_GY95_RTS GPIO_NUM_4
#define CONFIG_GY95_CTS GPIO_NUM_8
#define CONFIG_GY95_DEFAULT_FREQ 100
#define CONFIG_GY95_DEFAULT_BAUDRATE 115200
#define CONFIG_GY95_MAX_CHECK_TICKS 1024
#define CONFIG_GY95_NVS_TABLE_NAME "gy_scale"
#define CONFIG_GY95_RETRY_N 10
#define CONFIG_GY95_PAYLOAD_HEAD_IDX 4
#define CONFIG_GY95_PAYLOAD_TAIL_IDX 31

#define CONFIG_IMU_NAME GY95
#define CONFIG_IMU_PAYLOAD_LEN CONFIG_GY95_PAYLOAD_LEN
#define CONFIG_IMU_DEFAULT_FREQ CONFIG_GY95_DEFAULT_FREQ
#define CONFIG_IMU_CTRL_PIN CONFIG_GY95_CTRL_PIN
#define CONFIG_IMU_NVS_TABLE_NAME CONFIG_GY95_NVS_TABLE_NAME

typedef struct imu_dgram_t {
    uint8_t data[CONFIG_IMU_PAYLOAD_LEN];
    int64_t time_us;
    int64_t start_time_us;
    int uart_buffer_len;
} imu_dgram_t;

typedef struct imu_holder_t {
    int32_t accel_x;
    int32_t accel_y;
    int32_t accel_z;
    int32_t gyro_x;
    int32_t gyro_y;
    int32_t gyro_z;
    int32_t roll;
    int32_t pitch;
    int32_t yaw;
    int32_t temp;
    int32_t mag_x;
    int32_t mag_y;
    int32_t mag_z;
} imu_holder_t;

typedef struct imu_key_t {
    uint8_t accel_x;
    uint8_t accel_y;
    uint8_t accel_z;
    uint8_t gyro_x;
    uint8_t gyro_y;
    uint8_t gyro_z;
    uint8_t roll;
    uint8_t pitch;
    uint8_t yaw;
    uint8_t temp;
    uint8_t mag_x;
    uint8_t mag_y;
    uint8_t mag_z;
} imu_key_t;

typedef struct _imu_float_t {
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
} imu_multiplier_t, imu_res_t;

typedef enum gy95_status_t {
    GY95_OK,
    GY95_RECV_COMPLETE,
    GY95_FAIL,
    GY95_READY,
} gy95_status_t;

typedef struct gy95_t {
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
    gy95_status_t status;

    uint8_t acc_scale;
    uint8_t gyro_scale;
    uint8_t mag_scale;
    uint8_t scale;

    SemaphoreHandle_t mux;

    uint8_t buf[CONFIG_IMU_PAYLOAD_LEN];

} gy95_t;

void gy95_msp_init(gy95_t* p_gy);

void gy95_init(gy95_t* p_gy,
               int port,
               int ctrl_pin,
               int rx_pin,
               int tx_pin,
               int rts_pin,
               int cts_pin,
               int addr
);



uint8_t gy95_setup(gy95_t* p_gy);

esp_err_t gy95_cali_acc(gy95_t* p_gy);

void gy95_cali_mag(gy95_t* p_gy);

esp_err_t gy95_cali_reset(gy95_t* p_gy);

void gy95_read(gy95_t* p_gy);

void gy95_imm(gy95_t* p_gy);

void gy95_enable(gy95_t* p_gy);

void gy95_disable(gy95_t* p_gy);

#define IMU_INIT(imu) \
    gy95_init(&imu, CONFIG_GY95_UART_PORT, CONFIG_GY95_CTRL_PIN, CONFIG_GY95_RX, CONFIG_GY95_TX, CONFIG_GY95_RTS, CONFIG_GY95_CTS, CONFIG_GY95_ADDR); \
    gy95_msp_init(&imu); \
    gy95_disable(&imu); \
    gy95_enable(&imu);

/** @brief func_parse **/
esp_err_t parse_imu_reading(gy95_t* p_gy,
                            imu_dgram_t* p_reading,
                            imu_res_t* p_parsed,
                            char* buffer, int len);
int tag_imu_reading(imu_dgram_t* p_reading, uint8_t* payload, int len);

COMMAND_FUNCTION(cali_reset);
COMMAND_FUNCTION(cali_acc);
COMMAND_FUNCTION(cali_mag);
COMMAND_FUNCTION(imu_enable);
COMMAND_FUNCTION(imu_disable);
COMMAND_FUNCTION(imu_status);
COMMAND_FUNCTION(imu_imm);
COMMAND_FUNCTION(imu_setup);
COMMAND_FUNCTION(imu_scale);

#endif