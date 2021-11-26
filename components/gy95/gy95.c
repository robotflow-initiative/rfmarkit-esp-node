#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "nvs_flash.h"


#include "apps.h"
#include "esp_log.h"
#include "device.h"
#include "gy95.h"

static const char* TAG = "GY95";
// static portMUX_TYPE s_gy95_mux = portMUX_INITIALIZER_UNLOCKED;

#define ENTER_CONFIGURATION(p_gy)    \
    xSemaphoreTake(p_gy->mux, portMAX_DELAY);

#define EXIT_CONFIGURATION(p_gy)    \
    xSemaphoreGive(p_gy->mux);

gy95_t g_imu = { 0 };

void uart_service_init(int port, int rx, int tx, int rts, int cts) {
    uart_config_t uart_config = {
        .baud_rate = CONFIG_GY95_DEFAULT_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,

    };
    int intr_alloc_flags = 0;
    ESP_LOGI(TAG, "Initiate uart service at port %d, rx:%d, tx:%d", port, rx, tx);
    ESP_ERROR_CHECK(uart_driver_install(port, CONFIG_GY95_UART_RX_BUF_LEN, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(port, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(port, tx, rx, rts, cts));
}


void gy95_msp_init(gy95_t* p_gy) {
    uart_service_init(p_gy->port, p_gy->rx_pin, p_gy->tx_pin, p_gy->rts_pin, p_gy->cts_pin);

    gpio_config_t io_conf = {
        .intr_type = GPIO_PIN_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pin_bit_mask = (1ULL << p_gy->ctrl_pin),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };

    gpio_config(&io_conf);
    gpio_set_level(p_gy->ctrl_pin, 0);
    bool ret = rtc_gpio_is_valid_gpio(p_gy->ctrl_pin);
    if (ret) {
        ESP_LOGI(TAG, "GPIO: %d is valid rtc gpio", p_gy->ctrl_pin);
    } else {
        ESP_LOGW(TAG, "GPIO: %d is not valid rtc gpio", p_gy->ctrl_pin);
    }
}

static void gy95_read_scale(gy95_t* p_gy) {
    nvs_handle_t gy_scale_handle;
    ESP_ERROR_CHECK(nvs_open(CONFIG_IMU_NVS_TABLE_NAME, NVS_READWRITE, &gy_scale_handle));

    p_gy->acc_scale = 0;
    p_gy->gyro_scale = 0;
    p_gy->mag_scale = 0;
    p_gy->scale = 0x8b;

    uint8_t scale = 0;
    scale |= 1ULL << 7; // According to manual, set mount to horizontal

    nvs_get_u8(gy_scale_handle, "acc", &p_gy->acc_scale);
    nvs_get_u8(gy_scale_handle, "gyro", &p_gy->gyro_scale);
    nvs_get_u8(gy_scale_handle, "mag", &p_gy->mag_scale);

    if (p_gy->gyro_scale <= 3) {
        scale |= p_gy->gyro_scale;
        ESP_LOGI(TAG, "Setting gyro scale to %d", p_gy->gyro_scale);
    }
    
    if (p_gy->acc_scale <= 3) {
        scale |= p_gy->acc_scale << 2;
        ESP_LOGI(TAG, "Setting acc scale to %d", p_gy->acc_scale);
    }

    if (p_gy->mag_scale <= 3) {
        scale |= p_gy->mag_scale << 4;
        ESP_LOGI(TAG, "Setting mag scale to %d", p_gy->mag_scale);
    }

    ESP_LOGI(TAG, "Scale byte is %x", scale);
    p_gy->scale = scale;

    nvs_close(gy_scale_handle);

}

void gy95_init(gy95_t* p_gy,
               int port,
               int ctrl_pin,
               int rx_pin,
               int tx_pin,
               int rts_pin,
               int cts_pin,
               int addr
) {
    p_gy->port = port;
    p_gy->ctrl_pin = ctrl_pin;
    p_gy->rx_pin = rx_pin;
    p_gy->tx_pin = tx_pin;
    p_gy->rts_pin = rts_pin;
    p_gy->cts_pin = cts_pin;

    p_gy->addr = addr;
    p_gy->cursor = 0;
    p_gy->start_reg = 0;
    p_gy->length = 0;
    p_gy->status = GY95_READY;

    gy95_read_scale(p_gy);

    p_gy->mux = xSemaphoreCreateMutex();
    if (p_gy->mux == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        esp_restart();
    }

    bzero(p_gy->buf, CONFIG_GY95_PAYLOAD_LEN);

}

/**
 * @brief Clean GY95 buffer
 *
 * @param p_gy
 */
static void gy95_clean(gy95_t* p_gy) {
    bzero(p_gy->buf, CONFIG_GY95_PAYLOAD_LEN);
    p_gy->cursor = 0;
    p_gy->start_reg = 0;
    p_gy->length = 0;
    p_gy->status = GY95_READY;
}

#define CONFIG_GY95_MAX_CHECK_TIMEOUT 2000 / portTICK_PERIOD_MS
static esp_err_t gy95_check_echo(gy95_t* p_gy, uint8_t* msg, int len) {
    /** Clean old message **/
    uint8_t rx_buf[CONFIG_GY95_CTRL_MSG_LEN] = { 0 };
    int cursor = 0;

    TickType_t start_tick = xTaskGetTickCount();
    while (xTaskGetTickCount() - start_tick < CONFIG_GY95_MAX_CHECK_TIMEOUT) {

        uart_read_bytes(p_gy->port, &rx_buf[cursor], 1, 0xF);
#if CONFIG_EN_GY95_DEBUG
        printf("0x%x.", rx_buf[cursor]);
#endif
        if (rx_buf[cursor] != msg[cursor]) {
            bzero(rx_buf, sizeof(rx_buf));
            cursor = 0;
            continue;
        }
        cursor++;
        if (cursor >= len) {
            return ESP_OK;
        }
    }
    return ESP_FAIL;
}

/**
 * @brief Send msg with chksum appended
 *
 * @param p_gy
 * @param ctrl_msg
 */
#define CONFIG_GY95_CHECK_ECHO_N_RERY 2
static esp_err_t gy95_send(gy95_t* p_gy, uint8_t ctrl_msg[4], uint8_t* echo) {
    uint8_t ctrl_msg_with_chksum[CONFIG_GY95_CTRL_MSG_LEN + 1] = { 0 };

    long int sum = 0;
    for (int idx = 0; idx < CONFIG_GY95_CTRL_MSG_LEN; ++idx) {
        sum += ctrl_msg[idx];
        ctrl_msg_with_chksum[idx] = ctrl_msg[idx];
    }
    ctrl_msg_with_chksum[CONFIG_GY95_CTRL_MSG_LEN] = sum % 0x100;

    int n_retry = CONFIG_GY95_CHECK_ECHO_N_RERY;
    while (n_retry > 0) {
        uart_write_bytes_with_break((p_gy->port), ctrl_msg_with_chksum, 5, 0xF);
        uart_wait_tx_done((p_gy->port), portMAX_DELAY);
        esp_err_t err = ESP_FAIL;

        if (echo == NULL) {
            err = gy95_check_echo(p_gy, ctrl_msg_with_chksum, CONFIG_GY95_CTRL_MSG_LEN);
        } else {
            err = gy95_check_echo(p_gy, echo, CONFIG_GY95_CTRL_MSG_LEN); // Compare with custom echo message
        }
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "GY95 Echo Succeed");
            return ESP_OK;
        } else {
            ESP_LOGE(TAG, "GY95 Echo Failed");
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
        n_retry--;
    }

    return ESP_FAIL;
}

uint8_t gy95_setup(gy95_t* p_gy) {

    esp_err_t err = ESP_OK;
    uint8_t ret = 0;

    ENTER_CONFIGURATION(p_gy);

    gy95_read_scale(p_gy);

    ESP_LOGI(TAG, "Set rate to 100hz");
    err = gy95_send(p_gy, (uint8_t*)"\xa4\x06\x02\x02", NULL);
    if (err == ESP_OK) {
        ret |= BIT0;
    }

    // ESP_LOGI(TAG, "Set calibration method"); // TODO: Test this function
    // err = gy95_send(p_gy, (uint8_t*)"\xa4\x06\x06\x13", 4));
    // if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "Set mount to horizontal and sensibility: %x", p_gy->scale);
    uint8_t msg[4] = { 0xa4, 0x06, 0x07, p_gy->scale };
    err = gy95_send(p_gy, msg, NULL);
        if (err == ESP_OK) {
        ret |= BIT1;
    }


    ESP_LOGI(TAG, "Set continous output");
    err = gy95_send(p_gy, (uint8_t*)"\xa4\x06\x03\x00", NULL);
    if (err == ESP_OK) {
        ret |= BIT2;
    }


    ESP_LOGI(TAG, "Update with cali_acc");
    err = gy95_send(p_gy, (uint8_t*)"\xa4\x06\x05\x57", (uint8_t*)"\xa4\x06\x05\x00");
    if (err == ESP_OK) {
        ret |= BIT3;
    }


    ESP_LOGI(TAG, "Save module configuration");
    err = gy95_send(p_gy, (uint8_t*)"\xa4\x06\x05\x55", NULL);
    if (err == ESP_OK) {
        ret |= BIT4;
        printf("Ret: 0x%x\n", ret);
    }


    EXIT_CONFIGURATION(p_gy);
    return ret;
}

esp_err_t gy95_cali_acc(gy95_t* p_gy) {
    ENTER_CONFIGURATION(p_gy);
    ESP_LOGI(TAG, "Gyro-Accel calibrate");
    esp_err_t err = gy95_send(p_gy, (uint8_t*)"\xa4\x06\x05\x57", (uint8_t*)"\xa4\x06\x05\x00");

    EXIT_CONFIGURATION(p_gy);
    return err;
}
// if self.ser.writable():
//     self.ser.write(append_chksum(bytearray([0xa4, 0x06, 0x02, 0x02])))  # Rate 100Hz
//     time.sleep(0.1)
//     self.ser.write(append_chksum(bytearray([0xa4, 0x06, 0x03, 0x00])))  # Auto update
//     time.sleep(0.1)
//     self.ser.write(append_chksum(bytearray([0xa4, 0x06, 0x07, 0x8b])))  # Mount horizontally
//     time.sleep(0.1)
//     self.ser.write(append_chksum(bytearray([0xa4, 0x06, 0x05, 0x57])))  # Calibrate
//     time.sleep(7)

#define CONFIG_GY95_CALI_MAG_DELAY_MS 15000
void gy95_cali_mag(gy95_t* p_gy) {
    ENTER_CONFIGURATION(p_gy);

    ESP_LOGI(TAG, "Mag calibrate");
    /** Start calibration **/
    gy95_send(p_gy, (uint8_t*)"\xa4\x06\x05\x58", NULL);
    ESP_LOGI(TAG, "Point the IMU to all directions in next 15 seconds");

    esp_delay_ms(CONFIG_GY95_CALI_MAG_DELAY_MS);

    /** Stop calibration **/
    gy95_send(p_gy, (uint8_t*)"\xa4\x06\x05\x59", NULL);
    /** Save calibration result**/
    gy95_send(p_gy, (uint8_t*)"\xa4\x06\x05\x5A", (uint8_t*)"\xa4\x06\x05\x5A");
    /** Save module configuration **/
    gy95_send(p_gy, (uint8_t*)"\xa4\x06\x05\x55", (uint8_t*)"\xa4\x06\x05\x55");

    EXIT_CONFIGURATION(p_gy);
}

esp_err_t gy95_cali_reset(gy95_t* p_gy) {
    ENTER_CONFIGURATION(p_gy);
    esp_err_t err = gy95_send(p_gy, (uint8_t*)"\xa4\x06\x05\xaa", (uint8_t*)"\xa4\x06\x05\x00");

    if (err != ESP_OK) {
        return err;
    }

    err = gy95_setup(p_gy);

    EXIT_CONFIGURATION(p_gy);

    return err;
}

static bool gy95_chksum(gy95_t* p_gy) {
    long int sum = 0;
    for (int idx = 0; idx < p_gy->cursor; ++idx) {
        sum += p_gy->buf[idx];
    }
    return (sum % 0x100 == p_gy->buf[p_gy->cursor]) ? true : false;
}

#define CONFIG_GY95_MAXFAILED_BYTES 48
void gy95_read(gy95_t* p_gy) {
    /** Acqurie lock **/
    // ESP_LOGI(TAG, "Acquiring Lock");
    xSemaphoreTake(p_gy->mux, portMAX_DELAY);
    // ESP_LOGI(TAG, "Lock Acquired");

    gy95_clean(p_gy);
    int failed_bytes = 0;
    while (failed_bytes < CONFIG_GY95_MAXFAILED_BYTES) {
        uart_read_bytes(p_gy->port, &p_gy->buf[p_gy->cursor], 1, 0xF);
        ESP_LOGD(TAG, "%d:%d:%d\t", p_gy->port, p_gy->buf[p_gy->cursor], p_gy->cursor);

        switch (p_gy->cursor) {
        case 0:
            if (p_gy->buf[p_gy->cursor] != p_gy->addr) {
                failed_bytes += p_gy->cursor + 1;
                gy95_clean(p_gy);
                ESP_LOGD(TAG, "GYT95 reset buffer");
                continue;
            }
            break;
        case 1:
            if (p_gy->buf[p_gy->cursor] != CONFIG_GY95_READ_OP) {
                failed_bytes += p_gy->cursor + 1;
                gy95_clean(p_gy);
                ESP_LOGD(TAG, "GYT95 reset buffer");
                continue;
            }
            break;
        case 2:
            if (p_gy->buf[p_gy->cursor] < CONFIG_GY95_REG_THRESH) {
                p_gy->start_reg = p_gy->buf[p_gy->cursor];
            } else {
                failed_bytes += p_gy->cursor + 1;
                gy95_clean(p_gy);
                ESP_LOGD(TAG, "GYT95 reset buffer");
                continue;
            }
            break;
        case 3:
            if (p_gy->start_reg + (p_gy->buf[p_gy->cursor]) < CONFIG_GY95_REG_THRESH) {
                p_gy->length = p_gy->buf[p_gy->cursor];
            } else {
                failed_bytes += p_gy->cursor + 1;
                gy95_clean(p_gy);
                ESP_LOGD(TAG, "GYT95 reset buffer");
                continue;
            }
            break;
        default:
            if (p_gy->length + 4 == p_gy->cursor) {
                p_gy->status = GY95_RECV_COMPLETE;
            }
        }

        if (p_gy->status == GY95_RECV_COMPLETE) {
            if (gy95_chksum(p_gy)) {
                p_gy->status = GY95_OK;
                break;
            } else {
                p_gy->status = GY95_READY;
                ESP_LOGI(TAG, "GYT95 reset buffer");
                failed_bytes += p_gy->cursor;
                gy95_clean(p_gy);
            }
        } else {
            ++p_gy->cursor;
        }

    }

    if (failed_bytes >= CONFIG_GY95_MAXFAILED_BYTES) {
        p_gy->status = GY95_FAIL;
        gy95_clean(p_gy);
    }

    ESP_LOGD(TAG, "Releasing Lock");
    xSemaphoreGive(p_gy->mux);
    ESP_LOGD(TAG, "Lock released");

    /** Failed to read gy95 **/
}

void gy95_imm(gy95_t* p_gy) {
    ESP_LOGD(TAG, "Safe Reading from gy95");

    /** Manually flush input **/
    size_t buffer_len = 0;
    uart_get_buffered_data_len(p_gy->port, &buffer_len);

    if (buffer_len > CONFIG_GY95_UART_RX_BUF_LEN / 4) {
        ESP_LOGW(TAG, "BUFFER: %d", buffer_len);
        for (int i = 0; i < (buffer_len / CONFIG_GY95_PAYLOAD_LEN) / 2; ++i) {
            gy95_read(p_gy);
            esp_delay_ms(10);
        }
    }

    esp_delay_ms(1000);

    for (int i = 0; i < CONFIG_GY95_RETRY_N; ++i) {
        gy95_read(p_gy);
        esp_delay_ms(10);
        if (p_gy->status == GY95_OK) {
            int sum = 0;
            for (int i = CONFIG_GY95_PAYLOAD_HEAD_IDX; i < CONFIG_GY95_PAYLOAD_TAIL_IDX; ++i) {
                sum += p_gy->buf[i];
            }
            if (sum >= 0) {
                break;
            }
        }
    }
}

void gy95_enable(gy95_t* p_gy) {
    gpio_set_level(p_gy->ctrl_pin, 0);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    int ret = gpio_get_level(p_gy->ctrl_pin);
    ESP_LOGI(TAG, "GY95 control pin %d is %s", p_gy->ctrl_pin, ret ? "HIGH" : "LOW");
}

void gy95_disable(gy95_t* p_gy) {
    gpio_set_level(p_gy->ctrl_pin, 1);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    int ret = gpio_get_level(p_gy->ctrl_pin);
    ESP_LOGI(TAG, "GY95 control pin %d is %s", p_gy->ctrl_pin, ret ? "HIGH" : "LOW");
}

esp_err_t gy95_self_test(gy95_t* p_gy) {

    ESP_LOGI(TAG, "Running self test");

    imu_dgram_t imu_data = { 0 };
    imu_res_t imu_res = { 0 };
    double g_mod = 0;

    for (int i = 0; i < 3; ++i) {

        for (int j = 0; j < 3; ++j) {
            gy95_imm(p_gy);
            gy95_parse(p_gy, &imu_data, &imu_res, NULL, 0);
            memcpy(imu_data.data, p_gy->buf, sizeof(p_gy->buf));

            ESP_LOGI(TAG, "accel_x: %f, accel_y: %f, accel_z: %f", imu_res.accel_x, imu_res.accel_y, imu_res.accel_z);
            g_mod = imu_res.accel_x * imu_res.accel_x + imu_res.accel_y * imu_res.accel_y + imu_res.accel_z * imu_res.accel_z;
            if (g_mod > 1.1f || g_mod < 0.9f) {
                continue;
            } else {
                return ESP_OK;
            }
            esp_delay_ms(100);
        }

        gy95_setup(p_gy);
        esp_delay_ms(1000);
        RESET_SLEEP_COUNTUP();

    }
    
    return ESP_FAIL;
}



COMMAND_FUNCTION(imu_cali_reset) {
    ESP_LOGI(TAG, "Executing command : IMU_RESET");
    esp_err_t err = gy95_cali_reset(&g_imu);

    if (err == ESP_OK) {
        snprintf(tx_buffer, tx_len, "CALI_RESET_OK\n\n");

    } else {
        snprintf(tx_buffer, tx_len, "CALI_RESET_FAIL\n\n");

    }
    return ESP_OK;
}

COMMAND_FUNCTION(imu_cali_acc) {
    ESP_LOGI(TAG, "Executing command : IMU_CALI_ACC");
    esp_err_t err = gy95_cali_acc(&g_imu);
    if (err == ESP_OK) {
        snprintf(tx_buffer, tx_len, "CALI_ACC_OK\n\n");

    } else {
        snprintf(tx_buffer, tx_len, "CALI_ACC_FAIL\n\n");

    }
    return ESP_OK;
}

COMMAND_FUNCTION(imu_cali_mag) {
    ESP_LOGI(TAG, "Executing command : IMU_CALI_MAG");
    gy95_cali_mag(&g_imu);
    return ESP_OK;
}


COMMAND_FUNCTION(imu_enable) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_ENABLE");
    gy95_enable(&g_imu);
    xEventGroupSetBits(g_mcu.sys_event_group, GY95_ENABLED_BIT);
    return ESP_OK;
}

COMMAND_FUNCTION(imu_disable) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_DISABLE");
    gy95_disable(&g_imu);
    xEventGroupClearBits(g_mcu.sys_event_group, GY95_ENABLED_BIT);
    return ESP_OK;
}

/** FIXME: The status of  gy is output via serial debug port **/
COMMAND_FUNCTION(imu_status) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_STATUS");

    int ret = gpio_get_level(g_imu.ctrl_pin);
    ESP_LOGI(TAG, "GY95 control pin %s\n\n", ret ? "HIGH" : "LOW");
    snprintf(tx_buffer, tx_len, "GY95 control pin %s\n\n", ret ? "HIGH" : "LOW");
    return ESP_OK;
}


COMMAND_FUNCTION(imu_imm) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_IMM");
    imu_dgram_t imu_data;
    imu_res_t imu_res = { 0 };

    gy95_imm(&g_imu);
    memcpy(imu_data.data, g_imu.buf, sizeof(g_imu.buf));

    int offset = 0;
    gy95_parse(&g_imu, &imu_data, &imu_res, tx_buffer, tx_len);

    offset = strlen(tx_buffer);
    snprintf(tx_buffer + offset, tx_len - offset, "\n{\n\t\"acc_scale\":%d,\n\t\"gyro_scale\":%d,\n\t\"mag_scale\":%d\n}\n\n", g_imu.acc_scale, g_imu.gyro_scale, g_imu.mag_scale);
    // for (int idx = 0; idx < sizeof(g_imu.buf); ++idx) {
    //     snprintf(tx_buffer + offset, tx_len - offset, "0x%02x, ", imu_data.data[idx]);
    //     offset = strlen(tx_buffer);
    // };
    // snprintf(tx_buffer + offset, tx_len - offset, "\n\n");

    return ESP_OK;
}

COMMAND_FUNCTION(imu_setup) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_SETUP");
    uint8_t ret = gy95_setup(&g_imu);

    snprintf(tx_buffer, tx_len, "SETUP returned: 0x%x\n\n", ret);

    return ESP_OK;
}

#define SET_ATTR_U8(p, var, name, handle) \
if ((p) != NULL) { \
        if (cJSON_IsNumber((p))) { \
            (var) = (p)->valueint; \
            if ((var) <= 3) { \
                nvs_set_u8(handle, (name), (var)); \
                nvs_commit(handle); \
                snprintf(tx_buffer + offset, tx_len - offset, ""name"_SCALE set to %d; \n", (var)); \
                offset = strlen(tx_buffer); \
            } \
        } else { \
            snprintf(tx_buffer + offset, tx_len - offset, ""name"_SCALE set failed; \n"); \
            offset = strlen(tx_buffer); \
            err = ESP_FAIL; \
        } \
    } \

COMMAND_FUNCTION(imu_scale) {
    esp_err_t err = ESP_OK;
    ESP_LOGI(TAG, "Executing command : IMU_GY_SCALE");
    int offset = 0;
    uint8_t acc = 0;
    uint8_t gyro = 0;
    uint8_t mag = 0;

    /** Open nvs table **/
    nvs_handle_t gy_scale_handle;
    ESP_ERROR_CHECK(nvs_open(CONFIG_IMU_NVS_TABLE_NAME, NVS_READWRITE, &gy_scale_handle));

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

    SET_ATTR_U8(pAcc, acc, "acc", gy_scale_handle);
    SET_ATTR_U8(pGyro, gyro, "gyro", gy_scale_handle);
    SET_ATTR_U8(pMag, mag, "mag", gy_scale_handle);

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

COMMAND_FUNCTION(self_test) {
    ESP_LOGI(TAG, "Executing command : IMU_SELF_TEST");

    esp_err_t err = gy95_self_test(&g_imu);

    if (err == ESP_OK) {
        snprintf(tx_buffer, tx_len, "Self-test OK\n\n", g_mcu.blink_pin);
        return err;
    } else {
        snprintf(tx_buffer, tx_len, "Self-test FAIL\n\n", g_mcu.blink_pin);
        return err;
    }

}