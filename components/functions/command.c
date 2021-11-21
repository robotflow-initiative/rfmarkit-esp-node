#include "functions.h"

#include "esp_err.h"
#include "esp_log.h"
#include "cJSON.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "globals.h"
#include "apps.h"

static const char* TAG = "func_command";

esp_err_t command_func_restart(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_RESTART");
    esp_restart();
    return ESP_OK;
}

esp_err_t command_func_ping(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_PING");
    return ESP_OK;
}

esp_err_t command_func_shutdown(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_SHUTDOWN");
    esp_enter_deep_sleep();
    return ESP_OK;
}

esp_err_t command_func_update(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_UPDATE");
    xEventGroupSetBits(g_sys_event_group, UART_BLOCK_BIT);
    esp_err_t ret = esp_do_ota();
    return ret;
}

esp_err_t command_func_cali_reset(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_RESET");
    esp_err_t err = gy95_cali_reset(&g_imu);

    if (err == ESP_OK) {
        snprintf(tx_buffer, tx_len, "CALI_RESET_OK\n\n");

    } else {
        snprintf(tx_buffer, tx_len, "CALI_RESET_FAIL\n\n");

    }
    return ESP_OK;
}

esp_err_t command_func_cali_acc(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_CALI_ACC");
    esp_err_t err = gy95_cali_acc(&g_imu);
    if (err == ESP_OK) {
        snprintf(tx_buffer, tx_len, "CALI_ACC_OK\n\n");

    } else {
        snprintf(tx_buffer, tx_len, "CALI_ACC_FAIL\n\n");

    }
    return ESP_OK;
}

esp_err_t command_func_cali_mag(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_CALI_MAG");
    gy95_cali_mag(&g_imu);
    return ESP_OK;
}

esp_err_t command_func_start(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_START");
    xEventGroupClearBits(g_sys_event_group, UART_BLOCK_BIT);
    EventBits_t bits = xEventGroupWaitBits(g_sys_event_group, UART_ACTIVE_BIT, pdFALSE, pdFALSE, CONFIG_MAIN_LOOP_COUNT_PERIOD_MS / portTICK_PERIOD_MS);
    if (bits & UART_ACTIVE_BIT) {
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}

esp_err_t command_func_stop(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_STOP");
    xEventGroupSetBits(g_sys_event_group, UART_BLOCK_BIT);
    return ESP_OK;
}

esp_err_t command_func_gy_enable(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_ENABLE");
    gy95_enable(&g_imu);
    xEventGroupSetBits(g_sys_event_group, GY95_ENABLED_BIT);
    return ESP_OK;
}

esp_err_t command_func_gy_disable(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_DISABLE");
    gy95_disable(&g_imu);
    xEventGroupClearBits(g_sys_event_group, GY95_ENABLED_BIT);
    return ESP_OK;
}

/** FIXME: The status of  gy is output via serial debug port **/
esp_err_t command_func_gy_status(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_STATUS");

    int ret = gpio_get_level(g_imu.ctrl_pin);
    ESP_LOGI(TAG, "GY95 control pin %s\n\n", ret ? "HIGH" : "LOW");
    snprintf(tx_buffer, tx_len, "GY95 control pin %s\n\n", ret ? "HIGH" : "LOW");
    return ESP_OK;
}


esp_err_t command_func_gy_imm(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_IMM");
    imu_dgram_t imu_data;
    imu_res_t imu_res = { 0 };

    gy95_safe_read(&g_imu);
    memcpy(imu_data.data, g_imu.buf, GY95_PAYLOAD_LEN);

    int offset = 0;
    parse_imu_reading(&g_imu, &imu_data, &imu_res, tx_buffer, tx_len);

    offset = strlen(tx_buffer);
    snprintf(tx_buffer + offset, tx_len - offset, "\n{\n\t\"acc_scale\":%d,\n\t\"gyro_scale\":%d,\n\t\"mag_scale\":%d\n}\n\n", g_imu.acc_scale, g_imu.gyro_scale, g_imu.mag_scale);
    // for (int idx = 0; idx < GY95_PAYLOAD_LEN; ++idx) {
    //     snprintf(tx_buffer + offset, tx_len - offset, "0x%02x, ", imu_data.data[idx]);
    //     offset = strlen(tx_buffer);
    // };
    // snprintf(tx_buffer + offset, tx_len - offset, "\n\n");

    return ESP_OK;
}

esp_err_t command_func_gy_setup(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_SETUP");
    uint8_t ret = gy95_setup(&g_imu);

    snprintf(tx_buffer, tx_len, "SETUP returned: 0x%x\n\n", ret);
    
    return ESP_OK;
}

#define SET_ATTR(p, var, name) \
if ((p) != NULL) { \
        if (cJSON_IsNumber((p))) { \
            (var) = (p)->valueint; \
            if ((var) <= 3) { \
                nvs_set_u8(gy_scale_handle, (name), (var)); \
                nvs_commit(gy_scale_handle); \
                snprintf(tx_buffer + offset, tx_len - offset, ""name"_SCALE set to %d; \n", (var)); \
                offset = strlen(tx_buffer); \
            } \
        } else { \
            snprintf(tx_buffer + offset, tx_len - offset, ""name"_SCALE set failed; \n"); \
            offset = strlen(tx_buffer); \
            err = ESP_FAIL; \
        } \
    } \

esp_err_t command_func_gy_scale(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    esp_err_t err = ESP_OK;
    ESP_LOGI(TAG, "Executing command : IMU_GY_SCALE");
    int offset = 0;
    uint8_t acc = 0;
    uint8_t gyro = 0;
    uint8_t mag = 0;

    /** Open nvs table **/
    nvs_handle_t gy_scale_handle;
    ESP_ERROR_CHECK(nvs_open(CONFIG_GY95_SCALE_NVS_TABLE_NAME, NVS_READWRITE, &gy_scale_handle));

    /**
    rx_buffer = "gy_scale {"acc":[0-3],"gyro":[0-3], "mag":[0-3]}"
    **/
    cJSON* pRoot = cJSON_Parse(rx_buffer + sizeof("gy_scale"));
    cJSON* pAcc = NULL;
    cJSON* pGyro = NULL;
    cJSON* pMag = NULL;

    if (pRoot == NULL) {
        ESP_LOGE(TAG, "Parse failed");
        err = ESP_FAIL;
        goto gy_scale_cleanup;
    } else {
        pAcc = cJSON_GetObjectItem(pRoot, "acc");
        pGyro = cJSON_GetObjectItem(pRoot, "gyro");
        pMag = cJSON_GetObjectItem(pRoot, "mag");
        if ((pAcc == NULL) && (pGyro == NULL) && (pMag == NULL)) {
            ESP_LOGE(TAG, "Parse failed, invalid keys");
            err = ESP_FAIL;
            goto gy_scale_cleanup;
        }
    }

    SET_ATTR(pAcc, acc, "acc");
    SET_ATTR(pGyro, gyro, "gyro");
    SET_ATTR(pMag, mag, "mag");

    snprintf(tx_buffer + offset, tx_len - offset, "Finished, re-run gy_setup to take effect; \n\n");
    offset = strlen(tx_buffer);

gy_scale_cleanup:
    nvs_commit(gy_scale_handle);
    nvs_close(gy_scale_handle);

    if (pRoot != NULL) {
        cJSON_free(pRoot);
    }
    return err;
}

esp_err_t command_func_id(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_ID");
    snprintf(tx_buffer, tx_len, "%s\n\n", g_device_id);
    return ESP_OK;
}

esp_err_t command_func_ver(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_VER");
    snprintf(tx_buffer, tx_len, "%s\n\n", CONFIG_FIRMWARE_VERSION);
    return ESP_OK;
}

esp_err_t command_func_blink_set(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    esp_err_t err = ESP_OK;
    ESP_LOGI(TAG, "Executing command : IMU_BLINK_SET");
    uint8_t seq = 0;
    uint8_t pin = CONFIG_BLINK_DEFAULT_PIN;
    int offset = 0;

    /** Open nvs table **/
    nvs_handle_t blink_handle;
    ESP_ERROR_CHECK(nvs_open(CONFIG_BLINK_NVS_TABLE_NAME, NVS_READWRITE, &blink_handle));

    /**
    rx_buffer = "blink_set {"pin":"[R|G|B|r|g|b]","seq":"[0-255]"}
    rx_buffer = "blink_set {"seq":"[0-255]"}
    **/

    cJSON* pRoot = cJSON_Parse(rx_buffer + sizeof("blink_set"));
    cJSON* pSeq = NULL;
    cJSON* pPin = NULL;
    if (pRoot == NULL) {
        ESP_LOGE(TAG, "Parse failed");
        err = ESP_FAIL;
        goto blink_set_cleanup;
    } else {
        pPin = cJSON_GetObjectItem(pRoot, "pin");
        pSeq = cJSON_GetObjectItem(pRoot, "seq");
        if ((pSeq == NULL)) {
            ESP_LOGE(TAG, "Parse failed, invalid keys");
            err = ESP_FAIL;
            goto blink_set_cleanup;
        }
    }

    if (pPin != NULL) {
        if (cJSON_IsString(pPin)) {
            switch (pPin->valuestring[0]) {
            case 'R':
            case 'r':
                pin = CONFIG_BLINK_RED_PIN;
                snprintf(tx_buffer + offset, tx_len - offset, "Blink pin set to RED_PIN; ");
                offset = strlen(tx_buffer);
                break;
            case 'G':
            case 'g':
                pin = CONFIG_BLINK_GREEN_PIN;
                snprintf(tx_buffer + offset, tx_len - offset, "Blink pin set to GREEN_PIN; ");
                offset = strlen(tx_buffer);
                break;
            case 'B':
            case 'b':
                pin = CONFIG_BLINK_BLUE_PIN;
                snprintf(tx_buffer + offset, tx_len - offset, "Blink pin set to BLUE_PIN; ");
                offset = strlen(tx_buffer);
                break;
            default:
                snprintf(tx_buffer + offset, tx_len - offset, "Blink set failed\n\n");
                offset = strlen(tx_buffer);
                err = ESP_FAIL;
                goto blink_set_cleanup;
            }
        } else if (cJSON_IsNumber(pPin)) {
            pin = pPin->valueint;
            if (GPIO_IS_VALID_GPIO(pin)) {
                snprintf(tx_buffer + offset, tx_len - offset, "Blink pin set to %d; ");
                offset = strlen(tx_buffer);
                err = ESP_FAIL;
                goto blink_set_cleanup;
            } else {
                snprintf(tx_buffer + offset, tx_len - offset, "Blink set failed, invalid gpio\n\n");
                offset = strlen(tx_buffer);
            }
        } else {
            ESP_LOGI(TAG, "Parse failed, invalid pin");
            err = ESP_FAIL;
            goto blink_set_cleanup;
        }
        nvs_set_u8(blink_handle, "pin", pin);
        nvs_commit(blink_handle);

    }


    if (cJSON_IsNumber(pSeq)) {
        seq = pSeq->valueint % 0x100;
        snprintf(tx_buffer + offset, tx_len - offset, "Blink seq set to %d, reboot to make effective\n\n", seq);
    } else {
        snprintf(tx_buffer + offset, tx_len - offset, "Blink set failed, invalid seq\n\n");
        offset = strlen(tx_buffer);
        err = ESP_FAIL;
        goto blink_set_cleanup;
    }
    nvs_set_u8(blink_handle, "seq", seq);
    nvs_commit(blink_handle);

    err = ESP_OK;

blink_set_cleanup:

    nvs_commit(blink_handle);
    nvs_close(blink_handle);

    if (pRoot != NULL) {
        cJSON_free(pRoot);
    }
    return err;
}

esp_err_t command_func_blink_start(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_BLINK_START");

    app_blink_start();

    snprintf(tx_buffer, tx_len, "Blink started\n\n");
    return ESP_OK;
}

esp_err_t command_func_blink_stop(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_BLINK_STOP");

    app_blink_stop();

    snprintf(tx_buffer, tx_len, "Blink stopped\n\n");
    return ESP_OK;
}

esp_err_t command_func_blink_get(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_BLINK_GET");
    uint8_t seq = 0;
    uint8_t pin = 0;

    /** Open nvs table **/
    nvs_handle_t blink_handle;
    nvs_open("blink", NVS_READWRITE, &blink_handle);

    nvs_get_u8(blink_handle, "pin", &pin);
    nvs_get_u8(blink_handle, "seq", &seq);

    snprintf(tx_buffer, tx_len, "Blink pin is %d, seq is %d, \n\n", pin, seq);

    nvs_close(blink_handle);
    return ESP_OK;
}

esp_err_t command_func_blink_off(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_BLINK_OFF");

    app_blink_stop();

    gpio_set_level(g_blink_pin, !CONFIG_BLINK_LED_ENABLE_VALUE);

    snprintf(tx_buffer, tx_len, "Blink pin %d set to OFF\n\n", g_blink_pin);

    return ESP_OK;
}

esp_err_t command_func_self_test(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_SELF_TEST");

    esp_err_t err = esp_self_test();

    if (err == ESP_OK) {
        snprintf(tx_buffer, tx_len, "Self-test OK\n\n", g_blink_pin);
        return err;
    } else {
        snprintf(tx_buffer, tx_len, "Self-test FAIL\n\n", g_blink_pin);
        return err;
    }

}

esp_err_t command_func_always_on(char* rx_buffer, int rx_len, char* tx_buffer, int tx_len) {
    ESP_LOGI(TAG, "Executing command : IMU_ALWAYS_ON");

    g_sleep_countup = INT32_MIN;

    return ESP_OK;
}