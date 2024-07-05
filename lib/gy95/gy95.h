#ifndef GY95_H_
#define GY95_H_

#include "freertos/semphr.h"

#include "esp_system.h"

#include "cJSON.h"

#include "sys.h"

/** GY95 related settings **/
#define CONFIG_GY95_UART_RX_BUF_LEN 5120
#define CONFIG_GY95_CTRL_MSG_LEN 4
#define CONFIG_GY95_PAYLOAD_LEN 32
#define CONFIG_GY95_CTRL_PIN_MASK (1ULL << GY95_CTRL_PIN)
#define CONFIG_GY95_ADDR 0xa4
#define CONFIG_GY95_READ_OP 0x03
#define CONFIG_GY95_REG_THRESH 0x2c
#define CONFIG_GY95_DEFAULT_START_REG 0x14
#define CONFIG_GY95_UART_PORT (1) // UART_NUM_X

/** Modify this section to adapt different board **/
#define CONFIG_GY95_CTRL_PIN GPIO_NUM_5
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

typedef struct {
    size_t n_bytes;
    uint8_t data[CONFIG_GY95_PAYLOAD_LEN];
    int64_t time_us;
    int64_t start_time_us;
    int uart_buffer_len;
} gy95_dgram_t; // EXTERNAL


typedef struct {
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
} gy95_holder_t;

typedef struct {
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
} gy95_key_t;

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
} gy95_multiplier_t, gy95_data_t; // INTERNAL

typedef enum {
    GY95_OK,
    GY95_RECV_COMPLETE,
    GY95_FAIL,
    GY95_READY,
} gy95_status_t;

typedef struct {
    int port;

    int ctrl_pin;
    int rx_pin;
    int tx_pin;
    int rts_pin;
    int cts_pin;

    uint8_t addr;
    int start_reg;
    int length;
    gy95_status_t status;

    uint8_t acc_scale;
    uint8_t gyro_scale;
    uint8_t mag_scale;
    uint8_t scale;

    SemaphoreHandle_t mux;

    int n_bytes;
    uint8_t buf[CONFIG_GY95_PAYLOAD_LEN];

} gy95_t;


void gy95_msp_init(gy95_t* p_gy); // EXTERNAL

void gy95_init(gy95_t* p_gy,
               int port,
               int ctrl_pin,
               int rx_pin,
               int tx_pin,
               int rts_pin,
               int cts_pin,
               int addr
); // EXTERNAL

uint8_t gy95_setup(gy95_t* p_gy); // EXTERNAL

void gy95_read(gy95_t* p_gy);  // EXTERNAL

void gy95_enable(gy95_t* p_gy);  // EXTERNAL

void gy95_disable(gy95_t* p_gy);  // EXTERNAL

esp_err_t gy95_self_test(gy95_t* p_gy);  // EXTERNAL

/** @brief func_parse **/
esp_err_t gy95_parse(gy95_t* p_gy,
                     gy95_dgram_t* p_reading,
                     gy95_data_t* p_parsed,
                     char* buffer, int len);// EXTERNAL
int gy95_tag(gy95_dgram_t* p_reading, uint8_t* payload, int len);// EXTERNAL


/** Exposed API **/
#define CONFIG_IMU_NAME GY95
#define CONFIG_IMU_PAYLOAD_LEN CONFIG_GY95_PAYLOAD_LEN
#define CONFIG_IMU_EN_PIN CONFIG_GY95_EN_PIN
#define CONFIG_IMU_NVS_TABLE_NAME CONFIG_GY95_NVS_TABLE_NAME
#define CONFIG_IMU_SUPPORT_SCALE 1

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
COMMAND_FUNCTION(imu_self_test);

#endif