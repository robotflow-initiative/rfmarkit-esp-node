#include <sys/select.h>
#include <sys/cdefs.h>
#include <string.h>
#include <esp_mesh.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "nvs_flash.h"

#include "imu.h"
#include "hi229.h"
#include "hi229_serial.h"

#define EV_DATA_READY BIT0
#define hi229_delay_ms(x) vTaskDelay((x) / portTICK_PERIOD_MS);

typedef struct {
    imu_t base;
    /** buffer **/
    hi229_config_t config;
    raw_t raw;
    ch_imu_data_t raw_snapshot;
    int64_t raw_timestamp;
    TaskHandle_t data_proc_task_hdl;
    EventGroupHandle_t event_group;
} hi229_t;

imu_interface_t g_imu = {0};
static hi229_t s_hi229 = {0};

static const char *TAG = "imu[hi229]";

esp_err_t hi229_toggle(imu_t *p_imu, bool enable);

_Noreturn void hi229_poll_task(void *);

/**
 * @brief Initialize the IMU device
 * @param p_imu Pointer to the IMU device
**/
static void s_hi229_msp_init(hi229_t *p_imu) {
    ESP_LOGI(
        TAG,
        "UART .port=%d, .rx_pin=%d, .tx_pin=%d, .rts_pin=%d, .cts_pin=%d",
        p_imu->config.port,
        p_imu->config.rx_pin,
        p_imu->config.tx_pin,
        p_imu->config.rts_pin,
        p_imu->config.cts_pin
    );

    uart_config_t uart_config = {
        .baud_rate = p_imu->config.baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;
    ESP_LOGI(TAG, "initiate uart service at port %d, rx:%d, tx:%d", p_imu->config.port, p_imu->config.rx_pin, p_imu->config.tx_pin);
    ESP_ERROR_CHECK(uart_driver_install(p_imu->config.port, CONFIG_HI229_UART_RX_BUF_LEN, CONFIG_HI229_UART_TX_BUF_LEN, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(p_imu->config.port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(p_imu->config.port, p_imu->config.tx_pin, p_imu->config.rx_pin, p_imu->config.rts_pin, p_imu->config.cts_pin));

    gpio_config_t ctrl_pin_conf = {
        .intr_type = (gpio_int_type_t) GPIO_PIN_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << p_imu->config.ctrl_pin),
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&ctrl_pin_conf);
    gpio_set_level(p_imu->config.ctrl_pin, 0);

    gpio_config_t sync_out_pin_conf = {
        .intr_type = (gpio_int_type_t) GPIO_PIN_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << p_imu->config.sync_out_pin),
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&sync_out_pin_conf);
    gpio_set_level(p_imu->config.sync_out_pin, 0);

    gpio_config_t sync_in_pin_conf = {
        .intr_type = (gpio_int_type_t) GPIO_PIN_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << p_imu->config.sync_in_pin),
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&sync_in_pin_conf);

    p_imu->base.enabled = true;
    bool ret = rtc_gpio_is_valid_gpio(p_imu->config.ctrl_pin);
    if (ret) {
        ESP_LOGI(TAG, "GPIO.%d is valid rtc gpio", p_imu->config.ctrl_pin);
    } else {
        ESP_LOGW(TAG, "GPIO.%d is not valid rtc gpio", p_imu->config.ctrl_pin);
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
esp_err_t hi229_init(imu_t *p_imu, imu_config_t *p_config) {
    hi229_t *p_hi229 = (hi229_t *) p_imu;
    hi229_config_t *p_hi229_config = (hi229_config_t *) p_config;

    memcpy(&p_hi229->config, p_hi229_config, sizeof(hi229_config_t));

    p_imu->addr = CONFIG_HI229_ADDR;
    p_imu->enabled = false;

    memset(&p_hi229->raw, 0, sizeof(p_hi229->raw));

    p_imu->mutex = xSemaphoreCreateMutex();
    if (p_imu->mutex  == NULL) {
        ESP_LOGE(TAG, "failed to create mutex");
        p_imu->status = IMU_STATUS_FAIL;
    } else {
        p_imu->status = IMU_STATUS_READY;
    }
    p_hi229->event_group = xEventGroupCreate();

    /** Initialize IMU Hardware **/
    s_hi229_msp_init(p_hi229);
    hi229_delay_ms(100);
    hi229_toggle(p_imu, true);
    xTaskCreate(&hi229_poll_task, "hi229_poll_task", 4096, (void *)p_hi229, 7, &(p_hi229->data_proc_task_hdl)); // launch data proc task
    p_imu->initialized = true;
    return ESP_OK;
}


/**
 * @brief Read the IMU device, get a packet
 * @param p_gy Pointer to the IMU device
 * @param out Pointer to the IMU data
 * @return ESP_OK if the read is successful, otherwise ESP_FAIL
**/
esp_err_t IRAM_ATTR hi229_read(imu_t *p_imu_in, imu_dgram_t *out, bool crc_check) {
    hi229_t *p_hi229 = (hi229_t *) p_imu_in;

    if (xEventGroupWaitBits(p_hi229->event_group, EV_DATA_READY, pdFALSE, pdFALSE, 0x10) == EV_DATA_READY) {
        if (out != NULL) {
            ESP_LOGD(TAG, "IMU data ready, acc_x=%f", p_hi229->raw_snapshot.acc[0]);
            memcpy(&out->imu, &p_hi229->raw_snapshot, sizeof(ch_imu_data_t));
            int64_t time_delta = esp_timer_get_time() - p_hi229->raw_timestamp; // process time delta
            // tsf timestamp
            out->tsf_time_us = esp_mesh_get_tsf_time() - time_delta;

            size_t uart_buffer_len;
            uart_get_buffered_data_len(p_hi229->config.port, &uart_buffer_len);
            out->buffer_delay_us = (int32_t)(1000000 * uart_buffer_len / p_hi229->config.baud);

            // unix timestamp
            struct timeval tv_now = { 0, 0 };
            gettimeofday(&tv_now, NULL);
            out->time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec - time_delta;
            xEventGroupClearBits(p_hi229->event_group, EV_DATA_READY);
        } else {
            ESP_LOGD(TAG, "out is null");
        }
        return ESP_OK;
    }
    return ESP_FAIL;
}

_Noreturn void hi229_poll_task(void *p_imu) {
    hi229_t *p_hi229 = (hi229_t *) p_imu;
    uint8_t data[CONFIG_HI299_BLOCK_READ_NUM] = {0};

    while (1) {
        int len = uart_read_bytes(p_hi229->config.port, &data, CONFIG_HI299_BLOCK_READ_NUM, 0x10);


        for (int idx = 0; idx < len; ++idx) {
            if (ch_serial_input(&p_hi229->raw, data[idx], true) == 1) {
                memcpy(&p_hi229->raw_snapshot, &p_hi229->raw.imu[0], sizeof(ch_imu_data_t));
                p_hi229->raw_timestamp = esp_timer_get_time();
                xEventGroupSetBits(p_hi229->event_group, EV_DATA_READY);
            }
        }

        // we are not getting enough data, wait for a while
        if (len < CONFIG_HI299_BLOCK_READ_NUM) {
            hi229_delay_ms(10);
        }

    }
}
/**
 * @brief Enable/Disable the IMU device (power on)
 * @param p_gy Pointer to the IMU device
**/
esp_err_t hi229_toggle(imu_t *p_imu, bool enable) {
    hi229_t *p_hi229 = (hi229_t *) p_imu;
    gpio_set_level(p_hi229->config.ctrl_pin, enable ? CONFIG_HI229_ENABLE_LVL : CONFIG_HI229_DISABLE_LVL);

    hi229_delay_ms(100);
    uart_flush(p_hi229->config.port);
    p_hi229->base.enabled = enable ? true : false;

    int ret = gpio_get_level(p_hi229->config.ctrl_pin);
    ESP_LOGI(TAG, "hi229_disable, control_pin(:%d)=%s", p_hi229->config.ctrl_pin, ret ? "HIGH" : "LOW");

    return ESP_OK;
}

/**
 * @brief Check if the IMU device is enabled
 * @param p_gy
 * @return
 */
int hi229_is_powered_on(imu_t *p_imu) {
    hi229_t *p_hi229 = (hi229_t *) p_imu;
    return gpio_get_level(p_hi229->config.ctrl_pin) == CONFIG_HI229_ENABLE_LVL;
}

/**
 * @brief Self test the IMU device, should run during the init phase
 * @param p_gy Pointer to the IMU device
 * @return ESP_OK if the self test is successful, otherwise ESP_FAIL
**/
esp_err_t hi229_self_test(imu_t *p_imu) {
    hi229_t *p_hi229 = (hi229_t *) p_imu;
    uint8_t data[CONFIG_HI299_MAX_READ_NUM] = {0};
    size_t index_logged[CONFIG_HI229_SELF_TEST_RETRY] = {0};
    size_t index_cursor = 0;
    uart_flush(p_hi229->config.port);
    int len = uart_read_bytes(p_hi229->config.port, &data, CONFIG_HI299_MAX_READ_NUM, 0x100);
    for (int idx = 0; idx < len; ++idx) {
        if ((data[idx] == 0x5A) && (data[idx + 1] == 0xA5) && index_cursor < sizeof(index_logged) / sizeof(size_t)) {
            index_logged[index_cursor++] = idx;
        }
    }
    ESP_LOGI(TAG, "IMU self test finished, addr=[%d, %d, %d, %d]", index_logged[0], index_logged[1], index_logged[2], index_logged[3]);
    if (index_cursor == CONFIG_HI229_SELF_TEST_RETRY) {
        p_hi229->base.status = IMU_STATUS_READY;
        return ESP_OK;
    } else {
        p_hi229->base.status = IMU_STATUS_FAIL;
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
void hi229_chip_soft_reset(imu_t *p_imu) {
    const char *msg = "AT+RST\r\n";
    hi229_t *p_hi229 = (hi229_t *) p_imu;
    uart_write_bytes_with_break((p_hi229->config.port), (uint8_t *) msg, strlen(msg), 0xF);
    uart_wait_tx_done((p_hi229->config.port), portMAX_DELAY);
}


/**
 * @brief Reset the IMU device (hard)
 * @param p_gy
**/
void hi229_chip_hard_reset(imu_t *p_imu) {
    hi229_toggle(p_imu, false);
    hi229_toggle(p_imu, true);
}

/**
 * @brief Reset the IMU buffer
 * @param p_gy
**/
void hi229_buffer_reset(imu_t *p_imu) {
    hi229_t *p_hi229 = (hi229_t *) p_imu;
    memset(&(p_hi229->raw), 0, sizeof(p_hi229->raw));
    uart_flush(p_hi229->config.port);
}

/**
 *  @brief Get the buffer delay in us
 * @param p_gy
 * @return
 */
int64_t IRAM_ATTR hi229_get_buffer_delay(imu_t *p_imu) {
    size_t uart_buffer_len;
    hi229_t *p_hi229 = (hi229_t *) p_imu;
    uart_get_buffered_data_len(p_hi229->config.port, &uart_buffer_len);
    return (int32_t)(1000000 * uart_buffer_len / p_hi229->config.baud);
}

/**
 * @brief Read bytes from the IMU device
 * @param p_gy Pointer to the IMU device
 * @param out Pointer to the output buffer
 * @param len Length of the output buffer
 * @return Number of bytes read
**/
size_t hi229_read_bytes(imu_t *p_imu, uint8_t *out, size_t len) {
    hi229_t *p_hi229 = (hi229_t *) p_imu;
    return uart_read_bytes(p_hi229->config.port, out, len, 0x100);
}

/**
 * @brief Write bytes to the IMU device
 * @param p_gy
 * @param in
 * @param len
 * @return
 */
esp_err_t hi229_write_bytes(imu_t *p_imu, void *in, size_t len) {
    hi229_t *p_hi229 = (hi229_t *) p_imu;
    uart_write_bytes_with_break(p_hi229->config.port, (const void *) in, len, 0xF);
    uart_wait_tx_done(p_hi229->config.port, portMAX_DELAY);
    return ESP_OK;
}


void imu_interface_init(imu_interface_t *p_interface, imu_config_t * p_config) {
    p_interface->p_imu = (imu_t *) &s_hi229;
    if (p_interface->p_imu->initialized) {
        ESP_LOGW(TAG, "IMU already initialized");
        return;
    } else {
        hi229_init(p_interface->p_imu, p_config);
    }

    p_interface->init = hi229_init;
    p_interface->read = hi229_read;
    p_interface->toggle = hi229_toggle;
    p_interface->enabled = hi229_is_powered_on;
    p_interface->self_test = hi229_self_test;
    p_interface->soft_reset = hi229_chip_soft_reset;
    p_interface->hard_reset = hi229_chip_hard_reset;
    p_interface->buffer_reset = hi229_buffer_reset;
    p_interface->get_delay_us= hi229_get_buffer_delay;
    p_interface->read_bytes = hi229_read_bytes;
    p_interface->write_bytes = hi229_write_bytes;
}