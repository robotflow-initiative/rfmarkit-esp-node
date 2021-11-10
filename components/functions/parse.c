#include "functions.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_log.h"
#include "cJSON.h"

#include "types.h"
#include "settings.h"
#include "globals.h"

static const char* TAG = "func_parse";

static imu_msg_key_t default_key = {
    .accel_x = 4,
    .accel_y = 6,
    .accel_z = 8,
    .gyro_x = 10,
    .gyro_y = 12,
    .gyro_z = 14,
    .roll = 16,
    .pitch = 18,
    .yaw = 20,
    .temp = 23,
    .mag_x = 25,
    .mag_y = 27,
    .mag_z = 29,
};

static imu_msg_multiplier_t default_multiplier = {
    .accel_x = 0.000244140625, // 16 / 65536
    .accel_y = 0.000244140625, // 16 / 65536
    .accel_z = 0.000244140625, // 16 / 65536
    .gyro_x = 0.06103515625, // 4000 / 65536
    .gyro_y = 0.06103515625, // 4000 / 65536
    .gyro_z = 0.06103515625, // 4000 / 65536
    .roll = 0.01,
    .pitch = 0.01,
    .yaw = 0.01,
    .temp = 0.01,
    .mag_x = 0.00006103515625, // 4 / 65536
    .mag_y = 0.00006103515625, // 4 / 65536
    .mag_z = 0.00006103515625, // 4 / 65536
};

// static imu_msg_multiplier_t default_multiplier = {
//     .accel_x = 1,
//     .accel_y = 1,
//     .accel_z = 1,
//     .gyro_x = 1,
//     .gyro_y = 1,
//     .gyro_z = 1,
//     .roll = 1,
//     .pitch = 1,
//     .yaw = 1,
//     .temp = 1,
//     .mag_x = 1,
//     .mag_y = 1,
//     .mag_z = 1,
// };

/**
 * @brief Parse imu raw reading
 *
 * @param p_reading
 * @param buffer
 * @param len
 * @return esp_err_t
 * @warning This function dynamically allocates memory, remember to free them
 */
esp_err_t parse_imu_reading(imu_msg_raw_t* p_reading, char* buffer, int len) {
    imu_msg_holder_t msg_holder = { 0 };
    esp_err_t res;

#if CONFIG_EN_PARSER_DEBUG
    ESP_LOGW(TAG, "\n# ---- Begin of raw reading ---- #\n");
    for (int idx = 0; idx < GY95_PAYLOAD_LEN; ++idx) {
        printf("0x%02x, ", p_reading->data[idx]);
    };
    ESP_LOGW(TAG, "\n# ----- End of raw reading ----- #\n");
#endif

    /** memcopy **/
    msg_holder.accel_x = (p_reading->data[default_key.accel_x] + (p_reading->data[default_key.accel_x + 1] << 8));
    msg_holder.accel_y = (p_reading->data[default_key.accel_y] + (p_reading->data[default_key.accel_y + 1] << 8));
    msg_holder.accel_z = (p_reading->data[default_key.accel_z] + (p_reading->data[default_key.accel_z + 1] << 8));
    msg_holder.gyro_x = (p_reading->data[default_key.gyro_x] + (p_reading->data[default_key.gyro_x + 1] << 8));
    msg_holder.gyro_y = (p_reading->data[default_key.gyro_y] + (p_reading->data[default_key.gyro_y + 1] << 8));
    msg_holder.gyro_z = (p_reading->data[default_key.gyro_z] + (p_reading->data[default_key.gyro_z + 1] << 8));
    msg_holder.roll = (p_reading->data[default_key.roll] + (p_reading->data[default_key.roll + 1] << 8));
    msg_holder.pitch = (p_reading->data[default_key.pitch] + (p_reading->data[default_key.pitch + 1] << 8));
    msg_holder.yaw = (p_reading->data[default_key.yaw] + (p_reading->data[default_key.yaw + 1] << 8));
    msg_holder.temp = (p_reading->data[default_key.temp] + (p_reading->data[default_key.temp + 1] << 8));
    msg_holder.mag_x = (p_reading->data[default_key.mag_x] + (p_reading->data[default_key.mag_x + 1] << 8));
    msg_holder.mag_y = (p_reading->data[default_key.mag_y] + (p_reading->data[default_key.mag_y + 1] << 8));
    msg_holder.mag_z = (p_reading->data[default_key.mag_z] + (p_reading->data[default_key.mag_z + 1] << 8));

    msg_holder.accel_x = (msg_holder.accel_x >= 32768) ? (-(65536 - msg_holder.accel_x)) : msg_holder.accel_x;
    msg_holder.accel_y = (msg_holder.accel_y >= 32768) ? (-(65536 - msg_holder.accel_y)) : msg_holder.accel_y;
    msg_holder.accel_z = (msg_holder.accel_z >= 32768) ? (-(65536 - msg_holder.accel_z)) : msg_holder.accel_z;
    msg_holder.gyro_x = (msg_holder.gyro_x >= 32768) ? (-(65536 - msg_holder.gyro_x)) : msg_holder.gyro_x;
    msg_holder.gyro_y = (msg_holder.gyro_y >= 32768) ? (-(65536 - msg_holder.gyro_y)) : msg_holder.gyro_y;
    msg_holder.gyro_z = (msg_holder.gyro_z >= 32768) ? (-(65536 - msg_holder.gyro_z)) : msg_holder.gyro_z;
    msg_holder.roll = (msg_holder.roll >= 32768) ? (-(65536 - msg_holder.roll)) : msg_holder.roll;
    msg_holder.pitch = (msg_holder.pitch >= 32768) ? (-(65536 - msg_holder.pitch)) : msg_holder.pitch;
    msg_holder.yaw = (msg_holder.yaw >= 32768) ? (-(65536 - msg_holder.yaw)) : msg_holder.yaw;
    msg_holder.temp = (msg_holder.temp >= 32768) ? (-(65536 - msg_holder.temp)) : msg_holder.temp;
    msg_holder.mag_x = (msg_holder.mag_x >= 32768) ? (-(65536 - msg_holder.mag_x)) : msg_holder.mag_x;
    msg_holder.mag_y = (msg_holder.mag_y >= 32768) ? (-(65536 - msg_holder.mag_y)) : msg_holder.mag_y;
    msg_holder.mag_z = (msg_holder.mag_z >= 32768) ? (-(65536 - msg_holder.mag_z)) : msg_holder.mag_z;

    /** apply multiplier **/

    cJSON* pRoot = cJSON_CreateObject();

    cJSON_AddStringToObject(pRoot, "id", g_device_id);
    cJSON_AddNumberToObject(pRoot, "timestamp", p_reading->time_us);
    cJSON_AddNumberToObject(pRoot, "accel_x", msg_holder.accel_x * default_multiplier.accel_x);
    cJSON_AddNumberToObject(pRoot, "accel_y", msg_holder.accel_y * default_multiplier.accel_y);
    cJSON_AddNumberToObject(pRoot, "accel_z", msg_holder.accel_z * default_multiplier.accel_z);
    cJSON_AddNumberToObject(pRoot, "gyro_x", msg_holder.gyro_x * default_multiplier.gyro_x);
    cJSON_AddNumberToObject(pRoot, "gyro_y", msg_holder.gyro_y * default_multiplier.gyro_y);
    cJSON_AddNumberToObject(pRoot, "gyro_z", msg_holder.gyro_z * default_multiplier.gyro_z);
    cJSON_AddNumberToObject(pRoot, "roll", msg_holder.roll * default_multiplier.roll);
    cJSON_AddNumberToObject(pRoot, "pitch", msg_holder.pitch * default_multiplier.pitch);
    cJSON_AddNumberToObject(pRoot, "yaw", msg_holder.yaw * default_multiplier.yaw);
    cJSON_AddNumberToObject(pRoot, "temp", msg_holder.temp * default_multiplier.temp);
    cJSON_AddNumberToObject(pRoot, "mag_x", msg_holder.mag_x * default_multiplier.mag_x);
    cJSON_AddNumberToObject(pRoot, "mag_y", msg_holder.mag_y * default_multiplier.mag_y);
    cJSON_AddNumberToObject(pRoot, "mag_z", msg_holder.mag_z * default_multiplier.mag_z);


    res = (cJSON_PrintPreallocated(pRoot, buffer, len, 0) == 0) ? ESP_OK : ESP_FAIL;

#if CONFIG_EN_PARSER_DEBUG
    ESP_LOGI(TAG, "JSON String: %s\nRes:%d\n", buffer, res);
    cJSON_Delete(pRoot);
#endif
    return res;
}

/** Tag imu_reading with device id **/
int tag_imu_reading(imu_msg_raw_t* p_reading, uint8_t* payload_buffer, int len) {
    int offset = 0;

    ESP_LOGD(TAG, "Tagging imu readings");
#if CONFIG_EN_PARSER_DEBUG
    ESP_LOGW(TAG, "\n# ---- Begin of raw reading ---- #\n");
    for (int idx = 0; idx < GY95_PAYLOAD_LEN; ++idx) {
        printf("0x%02x, ", p_reading->data[idx]);
    };
    ESP_LOGW(TAG, "\n# ----- End of raw reading ----- #\n");
#endif

    if ((offset + GY95_PAYLOAD_LEN + sizeof(p_reading->time_us) + 12) > len) {
        return -1;
    }
    memcpy(payload_buffer + offset, p_reading->data, GY95_PAYLOAD_LEN);
    offset += GY95_PAYLOAD_LEN;

    memcpy(payload_buffer + offset, &p_reading->time_us, sizeof(p_reading->time_us));
    offset += sizeof(p_reading->time_us);

    memcpy(payload_buffer + offset, g_device_id, 12);
    offset += 12;

    uint32_t sum = 0;
    for (int idx = 0; idx < offset; ++idx) {
        sum += payload_buffer[idx];
    }
    payload_buffer[offset++] = sum % 0x100;

#if CONFIG_EN_PARSER_DEBUG
    ESP_LOGW(TAG, "\n# ---- Begin of payload ---- #\n");
    for (int idx = 0; idx < offset; ++idx) {
        printf("0x%02x, ", payload_buffer[idx]);
    };
    ESP_LOGW(TAG, "\n# ----- End of payload ----- #\n");
#endif

    return offset;
}