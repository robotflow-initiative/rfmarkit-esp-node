#include "functions.h"

#include "esp_err.h"
#include "esp_log.h"
#include "cJSON.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "globals.h"
#include "apps.h"

static const char* TAG = "func_command";



COMMAND_FUNCTION(restart) {
    ESP_LOGI(TAG, "Executing command : IMU_RESTART");
    esp_restart();
    return ESP_OK;
}

COMMAND_FUNCTION(ping){
    ESP_LOGI(TAG, "Executing command : IMU_PING");
    return ESP_OK;
}

COMMAND_FUNCTION(shutdown) {
    ESP_LOGI(TAG, "Executing command : IMU_SHUTDOWN");
    esp_enter_deep_sleep();
    return ESP_OK;
}

COMMAND_FUNCTION(update) {
    ESP_LOGI(TAG, "Executing command : IMU_UPDATE");
    xEventGroupSetBits(g_sys_event_group, UART_BLOCK_BIT);
    esp_err_t ret = esp_do_ota();
    return ret;
}

COMMAND_FUNCTION(start) {
    ESP_LOGI(TAG, "Executing command : IMU_START");
    xEventGroupClearBits(g_sys_event_group, UART_BLOCK_BIT);
    EventBits_t bits = xEventGroupWaitBits(g_sys_event_group, UART_ACTIVE_BIT, pdFALSE, pdFALSE, CONFIG_MAIN_LOOP_COUNT_PERIOD_MS / portTICK_PERIOD_MS);
    if (bits & UART_ACTIVE_BIT) {
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}

COMMAND_FUNCTION(stop) {
    ESP_LOGI(TAG, "Executing command : IMU_STOP");
    xEventGroupSetBits(g_sys_event_group, UART_BLOCK_BIT);
    return ESP_OK;
}


COMMAND_FUNCTION(id) {
    ESP_LOGI(TAG, "Executing command : IMU_ID");
    snprintf(tx_buffer, tx_len, "%s\n\n", g_device_id);
    return ESP_OK;
}

COMMAND_FUNCTION(ver) {
    ESP_LOGI(TAG, "Executing command : IMU_VER");
    snprintf(tx_buffer, tx_len, "%s\n\n", CONFIG_FIRMWARE_VERSION);
    return ESP_OK;
}

COMMAND_FUNCTION(blink_set) {
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

COMMAND_FUNCTION(blink_start) {
    ESP_LOGI(TAG, "Executing command : IMU_BLINK_START");

    app_blink_start();

    snprintf(tx_buffer, tx_len, "Blink started\n\n");
    return ESP_OK;
}

COMMAND_FUNCTION(blink_stop) {
    ESP_LOGI(TAG, "Executing command : IMU_BLINK_STOP");

    app_blink_stop();

    snprintf(tx_buffer, tx_len, "Blink stopped\n\n");
    return ESP_OK;
}

COMMAND_FUNCTION(blink_get) {
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

COMMAND_FUNCTION(blink_off) {
    ESP_LOGI(TAG, "Executing command : IMU_BLINK_OFF");

    app_blink_stop();

    gpio_set_level(g_blink_pin, !CONFIG_BLINK_LED_ENABLE_VALUE);

    snprintf(tx_buffer, tx_len, "Blink pin %d set to OFF\n\n", g_blink_pin);

    return ESP_OK;
}

COMMAND_FUNCTION(always_on) {
    ESP_LOGI(TAG, "Executing command : IMU_ALWAYS_ON");

    g_sleep_countup = INT32_MIN;

    return ESP_OK;
}

COMMAND_FUNCTION(wifi_set) { // TODO: Finish wifi set 
    ESP_LOGI(TAG, "Executing command : IMU_WIFI_SET");

    return ESP_FAIL;
}

COMMAND_FUNCTION(host_set) { // TODO: Finish host set 
    ESP_LOGI(TAG, "Executing command : IMU_HOST_SET");

    g_sleep_countup = INT32_MIN;

    return ESP_FAIL;
}