#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_log.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "nvs_flash.h"

#include "hi229.h"
#include "hi229_serial.h"

#define hi229_delay_ms(x) vTaskDelay((x) / portTICK_PERIOD_MS);
hi229_t g_imu = {0};

static const char *TAG = "imu[hi229]";

/**
 * @brief Initialize the IMU device
 * @param p_gy Pointer to the IMU device
**/
static void hi229_msp_init(hi229_t *p_gy) {
    ESP_LOGI(TAG, "UART .port=%d, .rx_pin=%d, .tx_pin=%d, .rts_pin=%d, .cts_pin=%d", p_gy->port, p_gy->rx_pin, p_gy->tx_pin, p_gy->rts_pin, p_gy->cts_pin);

    uart_config_t uart_config = {
            .baud_rate = p_gy->baud,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,

    };
    int intr_alloc_flags = 0;
    ESP_LOGI(TAG, "initiate uart service at port %d, rx:%d, tx:%d", p_gy->port, p_gy->rx_pin, p_gy->tx_pin);
    ESP_ERROR_CHECK(uart_driver_install(p_gy->port, CONFIG_HI229_UART_RX_BUF_LEN, CONFIG_HI229_UART_TX_BUF_LEN, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(p_gy->port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(p_gy->port, p_gy->tx_pin, p_gy->rx_pin, p_gy->rts_pin, p_gy->cts_pin));

    gpio_config_t ctrl_pin_conf = {
            .intr_type = (gpio_int_type_t) GPIO_PIN_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = (1ULL << p_gy->ctrl_pin),
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&ctrl_pin_conf);
    gpio_set_level(p_gy->ctrl_pin, 0);

    gpio_config_t sync_out_pin_conf = {
        .intr_type = (gpio_int_type_t) GPIO_PIN_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << p_gy->sync_out_pin),
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&sync_out_pin_conf);
    gpio_set_level(p_gy->sync_out_pin, 0);

    gpio_config_t sync_in_pin_conf = {
        .intr_type = (gpio_int_type_t) GPIO_PIN_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << p_gy->sync_in_pin),
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&sync_in_pin_conf);

    p_gy->enabled = true;
    bool ret = rtc_gpio_is_valid_gpio(p_gy->ctrl_pin);
    if (ret) {
        ESP_LOGI(TAG, "GPIO.%d is valid rtc gpio", p_gy->ctrl_pin);
    } else {
        ESP_LOGW(TAG, "GPIO.%d is not valid rtc gpio", p_gy->ctrl_pin);
    }
}

/**
 * @brief Initialize the IMU interface
 * @param p_gy
 * @param port
 * @param baud
 * @param ctrl_pin
 * @param rx_pin
 * @param tx_pin
 * @param addr
 */
void hi229_init(hi229_t *p_gy,
                int port,
                int baud,
                int ctrl_pin,
                int rx_pin,
                int tx_pin,
                int rts_pin,
                int cts_pin,
                int sync_in_pin,
                int sync_out_pin,
                int addr
) {
    p_gy->port = port;
    p_gy->baud = baud;
    p_gy->ctrl_pin = ctrl_pin;
    p_gy->rx_pin = rx_pin;
    p_gy->tx_pin = tx_pin;
    p_gy->rts_pin = rts_pin;
    p_gy->cts_pin = cts_pin;
    p_gy->sync_in_pin = sync_in_pin;
    p_gy->sync_out_pin = sync_out_pin;

    p_gy->addr = addr;
    p_gy->enabled = false;

    memset(&p_gy->raw, 0, sizeof(p_gy->raw));

    p_gy->mutex = xSemaphoreCreateMutex();
    if (p_gy->mutex == NULL) {
        ESP_LOGE(TAG, "failed to create mutex");
        p_gy->status = IMU_STATUS_FAIL;
    } else {
        p_gy->status = IMU_STATUS_READY;
    }

    /** Initialize IMU Hardware **/
    hi229_msp_init(p_gy);
    imu_delay_ms(100);
    hi229_enable(p_gy);
}

/**
 * @brief Setup the IMU device
 *  @param p_gy Pointer to the IMU device
 *  @return ESP_OK if the setup is successful, otherwise ESP_FAIL
**/
__attribute__((unused)) static esp_err_t hi229_setup(hi229_t *p_gy) {
    const char *msg[] = {
            "AT+EOUT=0\r\n",
            "AT+MODE=1\r\n",
            "AT+SETPTL=91\r\n",
            "AT+BAUD=921600\r\n",
            "AT+ODR=200\r\n",
            "AT+EOUT=1\r\n",
            "AT+RST\r\n",
    };
    for (int idx = 0; idx < sizeof(msg) / sizeof(char *); ++idx) {
        uart_write_bytes_with_break((p_gy->port), (uint8_t *) msg[idx], strlen(msg[idx]), 0xF);
        imu_delay_ms(100);
    }
    return ESP_OK;
}

/**
 * @brief Read the IMU device, get a packet
 * @param p_gy Pointer to the IMU device
 * @param out Pointer to the IMU data
 * @return ESP_OK if the read is successful, otherwise ESP_FAIL
**/
esp_err_t IRAM_ATTR hi229_read(hi229_t *p_gy, hi229_dgram_t *out, bool crc_check) {
    uint8_t data[CONFIG_HI299_BLOCK_READ_NUM] = {0};
    size_t try_count = 0;
    size_t read_count = 0;
    while (try_count < CONFIG_HI299_MAX_READ_NUM) {
        int len = uart_read_bytes(p_gy->port, &data, CONFIG_HI299_BLOCK_READ_NUM, 0x10);
        read_count += len;
        try_count += CONFIG_HI299_BLOCK_READ_NUM;

        bool packet_received = false;
        for (int idx = 0; idx < len; ++idx) {
            if (ch_serial_input(&p_gy->raw, data[idx], crc_check) == 1) {
                if (out != NULL) {
                    memcpy(out->imu, &p_gy->raw.imu, sizeof(ch_imu_data_t));
                }
                packet_received = true;
            }
        }

        if (packet_received) {
            ESP_LOGD(TAG, "reading IMU data %d/%d", read_count, try_count);
            return ESP_OK;
        }
    }
    ESP_LOGD(TAG, "reading IMU data %d/%d", read_count, try_count);
    return ESP_FAIL;
}

/**
 * @brief Enable the IMU device (power on)
 * @param p_gy Pointer to the IMU device
**/
void hi229_enable(hi229_t *p_gy) {
    gpio_set_level(p_gy->ctrl_pin, CONFIG_HI229_ENABLE_LVL);

    hi229_delay_ms(1000);
    uart_flush(p_gy->port);
    p_gy->enabled = true;

    int ret = gpio_get_level(p_gy->ctrl_pin);
    ESP_LOGD(TAG, "hi229_enable, control_pin(:%d)=%s", p_gy->ctrl_pin, ret ? "HIGH" : "LOW");
}

/**
 * @brief Disable the IMU device (power on)
 * @param p_gy Pointer to the IMU device
**/
void hi229_disable(hi229_t *p_gy) {
    gpio_set_level(p_gy->ctrl_pin, CONFIG_HI229_DISABLE_LVL);

    p_gy->enabled = false;
    hi229_delay_ms(200);
    uart_flush(p_gy->port);

    int ret = gpio_get_level(p_gy->ctrl_pin);
    ESP_LOGD(TAG, "hi229_disable, control_pin(:%d)=%s", p_gy->ctrl_pin, ret ? "HIGH" : "LOW");
}

/**
 * @brief Self test the IMU device, should run during the init phase
 * @param p_gy Pointer to the IMU device
 * @return ESP_OK if the self test is successful, otherwise ESP_FAIL
**/
esp_err_t hi229_self_test(hi229_t *p_gy) {
    uint8_t data[CONFIG_HI299_MAX_READ_NUM] = {0};
    size_t index_logged[CONFIG_HI229_SELF_TEST_RETRY] = {0};
    size_t index_cursor = 0;
    uart_flush(p_gy->port);
    int len = uart_read_bytes(p_gy->port, &data, CONFIG_HI299_MAX_READ_NUM, 0x100);
    for (int idx = 0; idx < len; ++idx) {
        if ((data[idx] == 0x5A) && (data[idx + 1] == 0xA5) && index_cursor < sizeof(index_logged) / sizeof(size_t)) {
            index_logged[index_cursor++] = idx;
        }
    }
    ESP_LOGI(TAG, "IMU self test finished, addr=[%d, %d, %d, %d]", index_logged[0], index_logged[1], index_logged[2], index_logged[3]);
    if (index_cursor == CONFIG_HI229_SELF_TEST_RETRY) {
        p_gy->status = IMU_STATUS_READY;
        return ESP_OK;
    } else {
        p_gy->status = IMU_STATUS_FAIL;
        return ESP_FAIL;
    }
}


/**
#define INIT_DATA(dest) \
        int dest##_offset = 0

#define ADD_DATA(dest, dest_limit, src, src_len) \
        if (((dest##_offset) + (src_len)) <= (dest_limit)) { \
            memcpy((dest) + (dest##_offset), (src), (src_len)); \
            dest##_offset += (src_len); \
        } \

#define GET_OFFSET(dest) dest##_offset
int hi229_tag(hi229_dgram_t *p_reading, uint8_t *payload_buffer, int len) {
    INIT_DATA(payload_buffer);

     // * @brief Format of packet:
     // *
     // * | g_imu.addr | imu  | seq | time_us | uart_buffer_len | device_id | chk_sum |

    ADD_DATA(payload_buffer, len, &g_imu.addr, 1);

    ADD_DATA(payload_buffer, len, &p_reading->imu, sizeof(p_reading->imu));

    ADD_DATA(payload_buffer, len, &p_reading->time_us, sizeof(p_reading->time_us));

    ADD_DATA(payload_buffer, len, &p_reading->tsf_time_us, sizeof(p_reading->tsf_time_us));

    ADD_DATA(payload_buffer, len, &p_reading->seq, sizeof(p_reading->seq));

    ADD_DATA(payload_buffer, len, &p_reading->uart_buffer_len, sizeof(p_reading->uart_buffer_len));

    ADD_DATA(payload_buffer, len, g_mcu.device_id, CONFIG_DEVICE_ID_LEN);

    uint8_t chksum = compute_checksum(payload_buffer, GET_OFFSET(payload_buffer));

    ADD_DATA(payload_buffer, len, &chksum, 1);

    return GET_OFFSET(payload_buffer);
}
**/

/**
 * @brief Reset the IMU device (soft)
 * @param p_gy
**/
void hi229_chip_soft_reset(hi229_t *p_gy) {
    const char *msg = "AT+RST\r\n";
    uart_write_bytes_with_break((p_gy->port), (uint8_t *) msg, strlen(msg), 0xF);
    uart_wait_tx_done((p_gy->port), portMAX_DELAY);
}


/**
 * @brief Reset the IMU device (hard)
 * @param p_gy
**/
void hi229_chip_hard_reset(hi229_t *p_gy) {
    hi229_disable(p_gy);
    hi229_enable(p_gy);
}

/**
 * @brief Reset the IMU buffer
 * @param p_gy
**/
void hi229_buffer_reset(hi229_t *p_gy) {
    memset(&p_gy->raw, 0, sizeof(p_gy->raw));
}