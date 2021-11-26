#include <string.h>

#include "freertos/FreeRTOS.h"

#include "esp_system.h"
#include "esp_log.h"

#include "cJSON.h"

#include "gy95.h"
#include "device.h"

static const char* TAG = "func_parse";

static imu_key_t default_key = {
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

static imu_multiplier_t default_multiplier = {
    .accel_x = 0.00006103515625, // 4 / 65536
    .accel_y = 0.00006103515625, // 4 / 65536
    .accel_z = 0.00006103515625, // 4 / 65536
    .gyro_x = 0.00762939453125, // 500 / 65536
    .gyro_y = 0.00762939453125, // 500 / 65536
    .gyro_z = 0.00762939453125, // 500 / 65536
    .roll = 0.01,
    .pitch = 0.01,
    .yaw = 0.01,
    .temp = 0.01,
    .mag_x = 0.00006103515625, // 4 / 65536
    .mag_y = 0.00006103515625, // 4 / 65536
    .mag_z = 0.00006103515625, // 4 / 65536
};

/**
 * @brief Parse imu raw reading
 *
 * @param p_reading
 * @param buffer
 * @param len
 * @return esp_err_t
 * @warning This function dynamically allocates memory, remember to free them
**/
esp_err_t gy95_parse(gy95_t* p_gy,
                            imu_dgram_t* p_reading,
                            imu_res_t* p_res,
                            char* buffer, int len) {
    imu_holder_t holder = { 0 };
    imu_res_t res = { 0 };
    esp_err_t err = ESP_OK;

#if CONFIG_EN_PARSER_DEBUG
    ESP_LOGW(TAG, "\n# ---- Begin of raw reading ---- #\n");
    for (int idx = 0; idx < sizeof(p_reading->data); ++idx) {
        printf("0x%02x, ", p_reading->data[idx]);
    };
    ESP_LOGW(TAG, "\n# ----- End of raw reading ----- #\n");
#endif

    /** MEMCOPY **/
    holder.accel_x = (p_reading->data[default_key.accel_x] + (p_reading->data[default_key.accel_x + 1] << 8));
    holder.accel_y = (p_reading->data[default_key.accel_y] + (p_reading->data[default_key.accel_y + 1] << 8));
    holder.accel_z = (p_reading->data[default_key.accel_z] + (p_reading->data[default_key.accel_z + 1] << 8));
    holder.gyro_x = (p_reading->data[default_key.gyro_x] + (p_reading->data[default_key.gyro_x + 1] << 8));
    holder.gyro_y = (p_reading->data[default_key.gyro_y] + (p_reading->data[default_key.gyro_y + 1] << 8));
    holder.gyro_z = (p_reading->data[default_key.gyro_z] + (p_reading->data[default_key.gyro_z + 1] << 8));
    holder.roll = (p_reading->data[default_key.roll] + (p_reading->data[default_key.roll + 1] << 8));
    holder.pitch = (p_reading->data[default_key.pitch] + (p_reading->data[default_key.pitch + 1] << 8));
    holder.yaw = (p_reading->data[default_key.yaw] + (p_reading->data[default_key.yaw + 1] << 8));
    holder.temp = (p_reading->data[default_key.temp] + (p_reading->data[default_key.temp + 1] << 8));
    holder.mag_x = (p_reading->data[default_key.mag_x] + (p_reading->data[default_key.mag_x + 1] << 8));
    holder.mag_y = (p_reading->data[default_key.mag_y] + (p_reading->data[default_key.mag_y + 1] << 8));
    holder.mag_z = (p_reading->data[default_key.mag_z] + (p_reading->data[default_key.mag_z + 1] << 8));

    /** UINT -> INT CONVERSION **/
    holder.accel_x = (holder.accel_x >= 32768) ? (-(65536 - holder.accel_x)) : holder.accel_x;
    holder.accel_y = (holder.accel_y >= 32768) ? (-(65536 - holder.accel_y)) : holder.accel_y;
    holder.accel_z = (holder.accel_z >= 32768) ? (-(65536 - holder.accel_z)) : holder.accel_z;
    holder.gyro_x = (holder.gyro_x >= 32768) ? (-(65536 - holder.gyro_x)) : holder.gyro_x;
    holder.gyro_y = (holder.gyro_y >= 32768) ? (-(65536 - holder.gyro_y)) : holder.gyro_y;
    holder.gyro_z = (holder.gyro_z >= 32768) ? (-(65536 - holder.gyro_z)) : holder.gyro_z;
    holder.roll = (holder.roll >= 32768) ? (-(65536 - holder.roll)) : holder.roll;
    holder.pitch = (holder.pitch >= 32768) ? (-(65536 - holder.pitch)) : holder.pitch;
    holder.yaw = (holder.yaw >= 32768) ? (-(65536 - holder.yaw)) : holder.yaw;
    holder.temp = (holder.temp >= 32768) ? (-(65536 - holder.temp)) : holder.temp;
    holder.mag_x = (holder.mag_x >= 32768) ? (-(65536 - holder.mag_x)) : holder.mag_x;
    holder.mag_y = (holder.mag_y >= 32768) ? (-(65536 - holder.mag_y)) : holder.mag_y;
    holder.mag_z = (holder.mag_z >= 32768) ? (-(65536 - holder.mag_z)) : holder.mag_z;

    /** APPLY MULTIPLIER **/
    res.accel_x = holder.accel_x * default_multiplier.accel_x * (1U << p_gy->acc_scale);
    res.accel_y = holder.accel_y * default_multiplier.accel_y * (1U << p_gy->acc_scale);
    res.accel_z = holder.accel_z * default_multiplier.accel_z * (1U << p_gy->acc_scale);
    res.gyro_x = holder.gyro_x * default_multiplier.gyro_x * (1U << p_gy->gyro_scale);
    res.gyro_y = holder.gyro_y * default_multiplier.gyro_y * (1U << p_gy->gyro_scale);
    res.gyro_z = holder.gyro_z * default_multiplier.gyro_z * (1U << p_gy->gyro_scale);
    res.roll = holder.roll * default_multiplier.roll;
    res.pitch = holder.pitch * default_multiplier.pitch;
    res.yaw = holder.yaw * default_multiplier.yaw;
    res.temp = holder.temp * default_multiplier.temp;
    res.mag_x = holder.mag_x * default_multiplier.mag_x * (1U << p_gy->mag_scale);
    res.mag_y = holder.mag_y * default_multiplier.mag_y * (1U << p_gy->mag_scale);
    res.mag_z = holder.mag_z * default_multiplier.mag_z * (1U << p_gy->mag_scale);

    /** catch result with pointer **/
    if (p_res != NULL) {
        memcpy(p_res, &res, sizeof(res));
    }

    if (buffer != NULL) {
        cJSON* pRoot = cJSON_CreateObject();

        cJSON_AddStringToObject(pRoot, "id", g_mcu.device_id);
        cJSON_AddNumberToObject(pRoot, "timestamp", p_reading->time_us);
        cJSON_AddNumberToObject(pRoot, "accel_x", res.accel_x);
        cJSON_AddNumberToObject(pRoot, "accel_y", res.accel_y);
        cJSON_AddNumberToObject(pRoot, "accel_z", res.accel_z);
        cJSON_AddNumberToObject(pRoot, "gyro_x", res.gyro_x);
        cJSON_AddNumberToObject(pRoot, "gyro_y", res.gyro_y);
        cJSON_AddNumberToObject(pRoot, "gyro_z", res.gyro_z);
        cJSON_AddNumberToObject(pRoot, "roll", res.roll);
        cJSON_AddNumberToObject(pRoot, "pitch", res.pitch);
        cJSON_AddNumberToObject(pRoot, "yaw", res.yaw);
        cJSON_AddNumberToObject(pRoot, "temp", res.temp);
        cJSON_AddNumberToObject(pRoot, "mag_x", res.mag_x);
        cJSON_AddNumberToObject(pRoot, "mag_y", res.mag_y);
        cJSON_AddNumberToObject(pRoot, "mag_z", res.mag_z);

        err = (cJSON_PrintPreallocated(pRoot, buffer, len, 0) == 0) ? ESP_OK : ESP_FAIL;
#if CONFIG_EN_PARSER_DEBUG
    ESP_LOGI(TAG, "JSON String: %s\nRes:%d\n", buffer, res);
#endif
        cJSON_free(pRoot);

    }

    return err;
}

/**
 * @brief Tag imu_reading with device id
 * 
 * @param p_reading 
 * @param payload_buffer 
 * @param len 
 * @return int 
 * @warning must guarentee the payload_buffer length
**/
int gy95_tag(imu_dgram_t* p_reading, uint8_t* payload_buffer, int len) {
    int offset = 0;

    ESP_LOGD(TAG, "Tagging imu readings");
#if CONFIG_EN_PARSER_DEBUG
    ESP_LOGW(TAG, "\n# ---- Begin of raw reading ---- #\n");
    for (int idx = 0; idx < sizeof(p_reading->data); ++idx) {
        printf("0x%02x, ", p_reading->data[idx]);
    };
    ESP_LOGW(TAG, "\n# ----- End of raw reading ----- #\n");
#endif

    /**
     * @brief Format of packet:
     * 
     * | 0xa4 | ... | chk_sum | timestamp |     id     | gy_scale | start_timestamp | uart_buffer_len | chk_sum |
     *    0              31     32  -  39   40  -  51       52         53  -  60         61  -  64         65
     */
    memcpy(payload_buffer + offset, p_reading->data, sizeof(p_reading->data));
    offset += sizeof(p_reading->data);

    memcpy(payload_buffer + offset, &p_reading->time_us, sizeof(p_reading->time_us));
    offset += sizeof(p_reading->time_us);

    memcpy(payload_buffer + offset, g_mcu.device_id, sizeof(g_mcu.device_id));
    offset += CONFIG_DEVICE_ID_LEN;

    payload_buffer[offset++] = g_imu.scale;
    
    memcpy(payload_buffer + offset, &p_reading->start_time_us, sizeof(p_reading->start_time_us));
    offset += sizeof(p_reading->start_time_us);

    memcpy(payload_buffer + offset, &p_reading->uart_buffer_len, sizeof(p_reading->uart_buffer_len));
    offset += sizeof(p_reading->uart_buffer_len);

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