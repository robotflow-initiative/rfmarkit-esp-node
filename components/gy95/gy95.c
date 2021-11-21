#include "apps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include <string.h>

#include "settings.h"
#include "types.h"
#include "esp_log.h"
#include "gy95.h"
#include "functions.h"

static const char* TAG = "GY95";
// static portMUX_TYPE s_gy95_mux = portMUX_INITIALIZER_UNLOCKED;

#define ENTER_CONFIGURATION(p_gy)    \
    xSemaphoreTake(p_gy->mux, portMAX_DELAY);

#define EXIT_CONFIGURATION(p_gy)    \
    xSemaphoreGive(p_gy->mux);

void uart_service_init(int port, int rx, int tx, int rts, int cts) {
    uart_config_t uart_config = {
        .baud_rate = GY95_DEFAULT_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,

    };
    int intr_alloc_flags = 0;
    ESP_LOGI(TAG, "Initiate uart service at port %d, rx:%d, tx:%d", port, rx, tx);
    ESP_ERROR_CHECK(uart_driver_install(port, CONFIG_UART_RX_BUF_LEN, 0, 0, NULL, intr_alloc_flags));
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
    ESP_ERROR_CHECK(nvs_open(CONFIG_GY95_SCALE_NVS_TABLE_NAME, NVS_READWRITE, &gy_scale_handle));

    p_gy->acc_scale = 0;
    p_gy->gyro_scale = 0;
    p_gy->mag_scale = 0;
    p_gy->scale = 0x8b;

    uint8_t scale = 0;
    scale |= 1ULL << 7; // According to manual, set mount to horizontal

    nvs_get_u8(gy_scale_handle, "acc", &p_gy->acc_scale);
    nvs_get_u8(gy_scale_handle, "gyro", &p_gy->gyro_scale);
    nvs_get_u8(gy_scale_handle, "mag", &p_gy->mag_scale);

    if (p_gy->acc_scale <= 3) {
        scale |= p_gy->acc_scale << 2;
        ESP_LOGI(TAG, "Setting acc scale to %d", p_gy->acc_scale);
    }
    if (p_gy->gyro_scale <= 3) {
        scale |= p_gy->gyro_scale;
        ESP_LOGI(TAG, "Setting gyro scale to %d", p_gy->gyro_scale);
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

    bzero(p_gy->buf, GY95_PAYLOAD_LEN);

}

/**
 * @brief Clean GY95 buffer
 *
 * @param p_gy
 */
void gy95_clean(gy95_t* p_gy) {
    bzero(p_gy->buf, GY95_PAYLOAD_LEN);
    p_gy->cursor = 0;
    p_gy->start_reg = 0;
    p_gy->length = 0;
    p_gy->status = GY95_READY;
}

#define CONFIG_GY95_MAX_CHECK_TIMEOUT 2000 / portTICK_PERIOD_MS
static esp_err_t gy95_check_echo(gy95_t* p_gy, uint8_t* msg, int len) {
    /** Clean old message **/
    uint8_t rx_buf[GY95_CTRL_MSG_LEN] = { 0 };
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
esp_err_t gy95_send(gy95_t* p_gy, uint8_t ctrl_msg[4], uint8_t* echo) {
    uint8_t ctrl_msg_with_chksum[GY95_CTRL_MSG_LEN + 1] = { 0 };

    long int sum = 0;
    for (int idx = 0; idx < GY95_CTRL_MSG_LEN; ++idx) {
        sum += ctrl_msg[idx];
        ctrl_msg_with_chksum[idx] = ctrl_msg[idx];
    }
    ctrl_msg_with_chksum[GY95_CTRL_MSG_LEN] = sum % 0x100;

    int n_retry = 3;
    while (n_retry > 0) {
        uart_write_bytes_with_break((p_gy->port), ctrl_msg_with_chksum, 5, 0xF);
        uart_wait_tx_done((p_gy->port), portMAX_DELAY);
        esp_err_t err = ESP_FAIL;

        if (echo == NULL) {
            err = gy95_check_echo(p_gy, ctrl_msg_with_chksum, GY95_CTRL_MSG_LEN);
        } else {
            err = gy95_check_echo(p_gy, echo, GY95_CTRL_MSG_LEN); // Compare with custom echo message
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

esp_err_t gy95_setup(gy95_t* p_gy) { // TODO: Use a bit array to mark return code of each instruction

    esp_err_t err = ESP_OK;

    ENTER_CONFIGURATION(p_gy);

    gy95_read_scale(p_gy);

    ESP_LOGI(TAG, "Set rate to 100hz");
    err = gy95_send(p_gy, (uint8_t*)"\xa4\x06\x02\x02", NULL);

    // ESP_LOGI(TAG, "Set calibration method"); // TODO: Test this function
    // err = gy95_send(p_gy, (uint8_t*)"\xa4\x06\x06\x13", 4));
    // if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "Set mount to horizontal and sensibility: %x", p_gy->scale);
    uint8_t msg[4] = { 0xa4, 0x06, 0x07, p_gy->scale };
    err = gy95_send(p_gy, msg, NULL);

    ESP_LOGI(TAG, "Set continous output");
    err = gy95_send(p_gy, (uint8_t*)"\xa4\x06\x03\x00", NULL);

    ESP_LOGI(TAG, "Update with cali_acc");
    gy95_send(p_gy, (uint8_t*)"\xa4\x06\x05\x57", (uint8_t*)"\xa4\x06\x05\x00");

    ESP_LOGI(TAG, "Save module configuration");
    gy95_send(p_gy, (uint8_t*)"\xa4\x06\x05\x55", NULL);

    EXIT_CONFIGURATION(p_gy);
    return err;
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

void gy95_cali_mag(gy95_t* p_gy) {
    ENTER_CONFIGURATION(p_gy);

    ESP_LOGI(TAG, "Mag calibrate");
    /** Start calibration **/
    gy95_send(p_gy, (uint8_t*)"\xa4\x06\x05\x58", NULL);
    ESP_LOGI(TAG, "Point the IMU to all directions in next 15 seconds");

    esp_delay_ms(15000); // TODO: Magic Delay

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

bool gy95_chksum(gy95_t* p_gy) {
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
            if (p_gy->buf[p_gy->cursor] != GY95_READ_OP) {
                failed_bytes += p_gy->cursor + 1;
                gy95_clean(p_gy);
                ESP_LOGD(TAG, "GYT95 reset buffer");
                continue;
            }
            break;
        case 2:
            if (p_gy->buf[p_gy->cursor] < GY95_REG_THRESH) {
                p_gy->start_reg = p_gy->buf[p_gy->cursor];
            } else {
                failed_bytes += p_gy->cursor + 1;
                gy95_clean(p_gy);
                ESP_LOGD(TAG, "GYT95 reset buffer");
                continue;
            }
            break;
        case 3:
            if (p_gy->start_reg + (p_gy->buf[p_gy->cursor]) < GY95_REG_THRESH) {
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

size_t gy95_get_buffer_len(gy95_t* p_gy) {
    size_t length;
    uart_get_buffered_data_len(p_gy->port, &length);
    return length;
}

void gy95_enable(gy95_t* p_gy) {
    gpio_set_level(p_gy->ctrl_pin, 0);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    int ret = gpio_get_level(p_gy->ctrl_pin);
    ESP_LOGI(TAG, "GY95 control pin %d is %s", p_gy->ctrl_pin, ret ? "HIGH" : "LOW");
    // esp_delay_ms(3000);
    // gy95_setup(p_gy);
}

void gy95_disable(gy95_t* p_gy) {
    gpio_set_level(p_gy->ctrl_pin, 1);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    int ret = gpio_get_level(p_gy->ctrl_pin);
    ESP_LOGI(TAG, "GY95 control pin %d is %s", p_gy->ctrl_pin, ret ? "HIGH" : "LOW");
}