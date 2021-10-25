#include "apps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>
#include "settings.h"
#include "types.h"
#include "esp_log.h"
#include "gy95.h"

static const char* TAG = "GY95";
// static portMUX_TYPE gy95_mmux = portMUX_INITIALIZER_UNLOCKED;

void gy95_msp_init(gy95_t* p_gy) {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << p_gy->ctrl_pin);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_set_level(p_gy->ctrl_pin, 0);
}

void gy95_init(gy95_t* p_gy, int port, int ctrl_pin, int addr) {
    p_gy->port = port;
    p_gy->ctrl_pin = ctrl_pin;

    memset(p_gy->buf, 0, GY95_MSG_LEN);
    p_gy->cursor = 0;
    p_gy->addr = addr;
    p_gy->start_reg = 0;
    p_gy->length = 0;
    p_gy->flag = 0;
}

void gy95_reset(gy95_t* p_gy) {
    memset(p_gy->buf, 0, GY95_MSG_LEN);
    p_gy->cursor = 0;
    p_gy->start_reg = 0;
    p_gy->length = 0;
    p_gy->flag = 0;
}

void gy95_send(gy95_t* p_gy, uint8_t* msg, int len) {
    if (len <= 0) {
        len = strlen((char*)msg);
    }
    long int sum = 0;
    char chksum = 0;
    for (int idx = 0; idx < len; ++idx) {
        sum += msg[idx];
    }
    chksum = sum % 0x100;
    // taskENTER_CRITICAL(&gy95_mmux);
    uart_write_bytes(p_gy->port, msg, len);
    uart_write_bytes(p_gy->port, &chksum, 1);
    // taskEXIT_CRITICAL(&gy95_mmux);
}

void gy95_setup(gy95_t* p_gy) {

    ESP_LOGI(TAG, "Set rate to 100hz");
    // gy95_send(p_gy, (uint8_t*)"\xa4\x06\x02\x02", 4);

    gy95_send(p_gy, (uint8_t*)"\xa4\x06\x02\x01", 4);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Set update policy to auto");
    gy95_send(p_gy, (uint8_t*)"\xa4\x06\x03\x00", 4);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Set mount to horizontal");
    gy95_send(p_gy, (uint8_t*)"\xa4\x06\x07\x8b", 4);
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

void gy95_calibrate(gy95_t* p_gy) {
    ESP_LOGI(TAG, "Gyro-Accel calibrate");
    gy95_send(p_gy, (uint8_t*)"\xa4\x06\x05\x57", 4);
    // Intentially delay 7s
    vTaskDelay(7000 / portTICK_PERIOD_MS); // TODO: Magic Delay
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

bool gy95_chksum(gy95_t* p_gy) {
    long int sum = 0;
    for (int idx = 0; idx < p_gy->cursor; ++idx) {
        sum += p_gy->buf[idx];
    }
    return (sum % 0x100 == p_gy->buf[p_gy->cursor]) ? true : false;
}


void gy95_read(gy95_t* p_gy) {
    gy95_reset(p_gy);
    while (1) {
        uart_read_bytes(p_gy->port, &p_gy->buf[p_gy->cursor], 1, 0xFF);
        ESP_LOGD(TAG, "%d:%d:%d\t", p_gy->port, p_gy->buf[p_gy->cursor], p_gy->cursor);

        switch (p_gy->cursor) {
        case 0:
            if (p_gy->buf[p_gy->cursor] != p_gy->addr) {
                gy95_reset(p_gy);
                ESP_LOGD(TAG, "GYT95 reset buffer");
                continue;
            }
            break;
        case 1:
            if (p_gy->buf[p_gy->cursor] != GY95_READ_OP) {
                gy95_reset(p_gy);
                ESP_LOGD(TAG, "GYT95 reset buffer");
                continue;
            }
            break;
        case 2:
            if (p_gy->buf[p_gy->cursor] < GY95_REG_THRESH) {
                p_gy->start_reg = p_gy->buf[p_gy->cursor];
            } else {
                gy95_reset(p_gy);
                ESP_LOGD(TAG, "GYT95 reset buffer");
                continue;
            }
            break;
        case 3:
            if (p_gy->start_reg + (p_gy->buf[p_gy->cursor]) < GY95_REG_THRESH) {
                p_gy->length = p_gy->buf[p_gy->cursor];
            } else {
                gy95_reset(p_gy);
                ESP_LOGD(TAG, "GYT95 reset buffer");

                continue;
            }
            break;
        default:
            if (p_gy->length + 4 == p_gy->cursor) {
                p_gy->flag = true;
            }
        }

        if (p_gy->flag) {
            p_gy->flag = false;
            if (gy95_chksum(p_gy)) {
                return;
            } else {
                ESP_LOGD(TAG, "GYT95 reset buffer");
                gy95_reset(p_gy);
            }
        } else {
            ++p_gy->cursor;
        }

    }
}

void gy95_enable() {
    gpio_set_level(GY95_CTRL_PIN, 0);
    vTaskDelay(100);
}

void gy95_disable() {
    gpio_set_level(GY95_CTRL_PIN, 1);
    vTaskDelay(100);
}