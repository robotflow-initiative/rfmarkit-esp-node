#ifndef _TYPES_H
#define _TYPES_H
#include <esp_system.h>

#define GY95_MSG_LEN 40

typedef struct imu_msg_holder_t {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    uint8_t level;
    int16_t temp;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
} imu_msg_holder_t;

typedef struct imu_msg_raw_t {
    uint8_t data[GY95_MSG_LEN];
    int64_t time_us;
} imu_msg_raw_t;

typedef struct imu_msg_key_t {
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
    
} imu_msg_key_t;

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
} imu_msg_multiplier_t, imu_msg_parsed_t;

// #define MAX_DESC_LEN 32
// typedef struct command_t {
//     server_command_t cmd;
//     char desc[MAX_DESC_LEN];
//     esp_err_t (*pfunc)(void);
// } command_t;

#endif