#ifndef _TYPES_H
#define _TYPES_H
#include <esp_system.h>

#include "settings.h"

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

typedef struct imu_dgram_t {
    uint8_t data[GY95_PAYLOAD_LEN];
    int64_t time_us;
} imu_dgram_t;

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

#endif