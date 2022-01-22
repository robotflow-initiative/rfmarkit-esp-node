#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_log.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "nvs_flash.h"

#include "apps.h"
#include "device.h"
#include "hi229.h"
#include "hi229_serial.h"

//TODO: Finish hipnuc adapter

static const char* TAG = "hi229";


#define ENTER_CONFIGURATION(p_imu)    \
    xSemaphoreTake(p_imu->mux, portMAX_DELAY);

#define EXIT_CONFIGURATION(p_imu)    \
    xSemaphoreGive(p_imu->mux);

#define hi229_delay_ms(x) vTaskDelay(x / portTICK_PERIOD_MS);
hi229_t g_imu = { 0 };


void hi229_msp_init(hi229_t* p_gy) {
    ESP_LOGI(TAG, "UART .port=%d, .rx_pin=%d, .tx_pin=%d, .rts_pin=%d, .cts_pin=%d", p_gy->port, p_gy->rx_pin, p_gy->tx_pin, p_gy->rts_pin, p_gy->cts_pin);
    // uart_service_init(p_gy->port, p_gy->rx_pin, p_gy->tx_pin, p_gy->rts_pin, p_gy->cts_pin);

    int port = p_gy->port;
    int rx = p_gy->rx_pin;
    int tx = p_gy->tx_pin;

    uart_config_t uart_config = {
            .baud_rate = CONFIG_HI229_DEFAULT_BAUDRATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,

    };
    int intr_alloc_flags = 0;
    ESP_LOGI(TAG, "Initiate uart service at port %d, rx:%d, tx:%d", port, rx, tx);
    ESP_ERROR_CHECK(uart_driver_install(port, CONFIG_HI229_UART_RX_BUF_LEN, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    gpio_config_t io_conf = {
            .intr_type = GPIO_PIN_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = (1ULL << p_gy->ctrl_pin),
            .pull_down_en = 1,
            .pull_up_en = 0,
    };

    gpio_config(&io_conf);
    gpio_set_level(p_gy->ctrl_pin, 0);
    // bool ret = rtc_gpio_is_valid_gpio(p_gy->ctrl_pin); FIXME: After PCB mod, re-enable this feature
    bool ret = false;
    if (ret) {
        ESP_LOGI(TAG, "GPIO: %d is valid rtc gpio", p_gy->ctrl_pin);
    } else {
        ESP_LOGW(TAG, "GPIO: %d is not valid rtc gpio", p_gy->ctrl_pin);
    }
}

void hi229_init(hi229_t* p_gy,
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
    p_gy->rts_pin = UART_PIN_NO_CHANGE;
    p_gy->cts_pin = UART_PIN_NO_CHANGE;

    p_gy->addr = addr;
    p_gy->status = HI229_READY;

    p_gy->mux = xSemaphoreCreateMutex();
    if (p_gy->mux == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        esp_restart();
    }

    bzero(p_gy->buf, CONFIG_HI229_PAYLOAD_LEN);
}

static esp_err_t hi229_send(hi229_t* p_gy, uint8_t* ctrl_msg, uint8_t* echo) {
    return ESP_FAIL;
}

uint8_t hi229_setup(hi229_t* p_gy) {
    hi229_send(p_gy, (uint8_t*)"AT+ODR=200\r\n", (uint8_t*)"AT+OK");
    hi229_send(p_gy, (uint8_t*)"AT+BAUD=115200\r\n", (uint8_t*)"AT+OK");
    hi229_send(p_gy, (uint8_t*)"AT+EOUT=1\r\n", (uint8_t*)"AT+OK");
    hi229_send(p_gy, (uint8_t*)"AT+MODE=1\r\n", (uint8_t*)"AT+OK");
    hi229_send(p_gy, (uint8_t*)"AT+SETPTL=91\r\n", (uint8_t*)"AT+OK");
    // FIXME: Check this part

    return 0b11111;
}

#define CONFIG_HI299_MAX_READ_NUM 0xff
esp_err_t hi229_read(hi229_t* p_gy) {
    uint8_t data = 0x0;
    size_t count = 0;
    while (count < CONFIG_HI299_MAX_READ_NUM) {
        uart_read_bytes(p_gy->port, &data, 1, 0xF);
        ++count;
        // printf("0x%x.", data);
        if (ch_serial_input(&p_gy->raw, data) == 1) {
            // TODO: [MID] Re-Write this part and speed up
            p_gy->n_bytes = sizeof(ch_imu_data_t); // FIXME: Only support 1 IMU !!!
            memcpy(&p_gy->buf, &p_gy->raw.imu, p_gy->n_bytes);
            // for (int i = 0;i < p_gy->n_bytes; ++i) {
            //     printf("0x%x. ", p_gy->buf[i]);
            // }
            break;
        }
    }
    return ESP_OK;
}


void hi229_enable(hi229_t* p_gy) {
    gpio_set_level(p_gy->ctrl_pin, 0);
    hi229_delay_ms(200);
    int ret = gpio_get_level(p_gy->ctrl_pin);
    ESP_LOGI(TAG, "HI229 control pin %d is %s", p_gy->ctrl_pin, ret ? "HIGH" : "LOW");
}

void hi229_disable(hi229_t* p_gy) {
    gpio_set_level(p_gy->ctrl_pin, 1);
    hi229_delay_ms(200);
    int ret = gpio_get_level(p_gy->ctrl_pin);
    ESP_LOGI(TAG, "HI229 control pin %d is %s", p_gy->ctrl_pin, ret ? "HIGH" : "LOW");
}

esp_err_t hi229_self_test(hi229_t* p_gy) {
    // FIXME: Finish test
    return ESP_OK;
}

esp_err_t hi229_parse(hi229_t* p_gy,
                      hi229_dgram_t* p_reading,
                      hi229_data_t* p_parsed,
                      char* buffer, int len) {
    return ESP_FAIL;
}

#define INIT_DATA(dest) \
        int dest##_offset = 0

#define ADD_DATA(dest, dest_limit, src, src_len) \
        if (((dest##_offset) + (src_len)) <= dest_limit) { \
            memcpy((dest) + (dest##_offset), (src), (src_len)); \
            dest##_offset += (src_len); \
        } \

#define GET_OFFSET(dest) dest##_offset

static uint8_t compute_chksum(uint8_t* data, size_t len) {
    uint32_t sum = 0;
    for (int idx = 0; idx < len; ++idx) {
        sum += data[idx];
    }
    return sum % 0x100;
}

int hi229_tag(hi229_dgram_t* p_reading, uint8_t* payload_buffer, int len) {
    INIT_DATA(payload_buffer);
    /**
     * @brief Format of packet:
     *
     * | g_imu.addr | ... | chk_sum | timestamp |     id     | gy_scale | start_timestamp | uart_buffer_len | chk_sum |
     */
    ADD_DATA(payload_buffer, len, &g_imu.addr, 1);

    ADD_DATA(payload_buffer, len, &p_reading->data, p_reading->n_bytes);

    uint8_t chksum = compute_chksum(p_reading->data, p_reading->n_bytes);
    ADD_DATA(payload_buffer, len, &chksum, 1);

    ADD_DATA(payload_buffer, len, &p_reading->time_us, sizeof(p_reading->time_us));

    ADD_DATA(payload_buffer, len, g_mcu.device_id, CONFIG_DEVICE_ID_LEN);

    const uint8_t scale = 0;
    ADD_DATA(payload_buffer, len, &scale, 1);

    ADD_DATA(payload_buffer, len, &p_reading->start_time_us, sizeof(p_reading->start_time_us));

    ADD_DATA(payload_buffer, len, &p_reading->uart_buffer_len, sizeof(p_reading->uart_buffer_len));

    chksum = compute_chksum(payload_buffer, GET_OFFSET(payload_buffer));
    ADD_DATA(payload_buffer, len, &chksum, 1);

    return GET_OFFSET(payload_buffer);
}

static void hi229_reset(hi229_t* p_gy) {
    hi229_send(p_gy, (uint8_t*)"AT+RST\r\n", (uint8_t*)"AT+OK");
}

COMMAND_FUNCTION(imu_cali_reset) {
    ESP_LOGI(TAG, "Executing command : IMU_RESET");
    hi229_reset(&g_imu);
    return ESP_OK;
}
COMMAND_FUNCTION(imu_cali_acc) {
    ESP_LOGI(TAG, "Executing command : IMU_CALI_MAG");
    hi229_reset(&g_imu);
    return ESP_OK;
}
COMMAND_FUNCTION(imu_cali_gyro) {
    ESP_LOGI(TAG, "Executing command : IMU_CALI_GYRO");
    hi229_reset(&g_imu);
    return ESP_OK;
}
COMMAND_FUNCTION(imu_cali_mag) {
    ESP_LOGI(TAG, "Executing command : IMU_CALI_MAG");
    hi229_reset(&g_imu);

    return ESP_OK;
}
COMMAND_FUNCTION(imu_enable) {
    ESP_LOGI(TAG, "Executing command : IMU_CALI_MAG");
    hi229_enable(&g_imu);
    set_sys_event(IMU_ENABLED);
    return ESP_OK;
}
COMMAND_FUNCTION(imu_disable) {
    ESP_LOGI(TAG, "Executing command : IMU_CALI_MAG");
    hi229_disable(&g_imu);
    clear_sys_event(IMU_ENABLED);
    return ESP_OK;
}
COMMAND_FUNCTION(imu_status) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_STATUS");
    int ret = gpio_get_level(g_imu.ctrl_pin);
    ESP_LOGI(TAG, "HI229 control pin %s\n\n", ret ? "HIGH" : "LOW");
    snprintf(tx_buffer, tx_len, "HI229 control pin %s\n\n", ret ? "HIGH" : "LOW");
    return ESP_OK;
}
COMMAND_FUNCTION(imu_imm) {
    return ESP_FAIL;
}

COMMAND_FUNCTION(imu_setup) {
    ESP_LOGI(TAG, "Executing command : IMU_GY_SETUP");
    uint8_t ret = hi229_setup(&g_imu);

    snprintf(tx_buffer, tx_len, "SETUP returned: 0x%x\n\n", ret);

    return ESP_OK;
}
COMMAND_FUNCTION(imu_scale) {
    snprintf(tx_buffer, tx_len, "IMU does not suport scale setting\n\n");
    return ESP_FAIL;
}

COMMAND_FUNCTION(imu_self_test) {
    ESP_LOGI(TAG, "Executing command : IMU_SELF_TEST");

    esp_err_t err = imu_self_test(&g_imu);

    if (err == ESP_OK) {
        snprintf(tx_buffer, tx_len, "Self-test OK: %d\n\n", g_mcu.blink_pin);
        return err;
    } else {
        snprintf(tx_buffer, tx_len, "Self-test FAIL: %d\n\n", g_mcu.blink_pin);
        return err;
    }

}