#include <string.h>
#include <esp_mesh.h>
#include <math.h>

#include "esp_http_server.h"
#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_vfs.h"
#include "cJSON.h"

#include "sys.h"
#include "rest_controller_handlers.h"
#include "blink.h"
#include "imu.h"
#include "battery.h"

static const char *TAG = "sys.rest.hdl    ";

#define CONFIG_PARAM_VALUE_MAX_LEN 128
#define CONFIG_WEBSOCKET_FRAM_MAX_LEN 64

/**
 * @brief Parse the key-value pair from the URI
 * uri encoded string is converted to plain text
 * @param uri
 * @param key
 * @param value
 * @return
**/
void url_decode_copyn(char *dest, const char *src, size_t size) {
    char *p = (char *) src;
    char *p_end = (char *) src + size;
    char code[3] = {0};
    unsigned char ch;

    while (*p) {
        if (p >= p_end) {
            break;
        }
        if (*p == '%') {
            memcpy(code, ++p, 2);
            code[2] = '\0';
            ch = (unsigned char) strtol(code, NULL, 16);
            *dest++ = (char) ch;
            p += 2;
        } else if (*p == '+') {
            *dest++ = ' ';
            p++;
        } else {
            *dest++ = *p++;
        }
    }

    *dest = '\0';
}

/**
 * @brief Parse the key-value pair from the URI
 * @param uri
 * @param key
 * @param value
 * @return
**/
static esp_err_t parse_url_kv_pair(const char *uri, const char *key, char *value) {
    if (uri == NULL || key == NULL || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    char *kv_start = strstr(uri, key);
    if (kv_start == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    char *kv_end = strchr(kv_start, '&');
    if (kv_end == NULL) {
        kv_end = (char *) uri + strlen(uri);
    }

    char *eq_pos = strchr(kv_start, '=');
    if (eq_pos == NULL || eq_pos > kv_end) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t value_len = kv_end - eq_pos - 1;
    if (value_len > CONFIG_PARAM_VALUE_MAX_LEN) {
        return ESP_ERR_INVALID_ARG;
    }
    // strncpy(value, eq_pos + 1, value_len);
    url_decode_copyn(value, eq_pos + 1, value_len);
    value[value_len] = '\0';

    return ESP_OK;
}

// uri:/v1/system/info
esp_err_t system_info_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    reset_power_save_timer();
    cJSON *root = cJSON_CreateObject();

    cJSON_AddStringToObject(root, "id", g_mcu.device_id);
    cJSON_AddStringToObject(root, "ver", CONFIG_FIRMWARE_VERSION);
    struct timeval tv_now = {0};
    gettimeofday(&tv_now, NULL);
    double now = tv_now.tv_sec + (double) tv_now.tv_usec / 1000000;
    cJSON_AddNumberToObject(root, "time", now);
    int64_t tsf_time = esp_mesh_get_tsf_time();
    // NOLINTNEXTLINE(cppcoreguidelines-narrowing-conversions)
    cJSON_AddNumberToObject(root, "tsf_time", tsf_time);
    int64_t battery_level = battery_read_level();
    cJSON_AddNumberToObject(root, "battery_level", (double) battery_level);
    cJSON_AddNumberToObject(root, "target_fps", g_mcu.target_fps);

    const char *response = cJSON_Print(root);
    httpd_resp_sendstr(req, response);
    free((void *) response);
    cJSON_Delete(root);
    return ESP_OK;
}

static void system_power_reboot_cb(__attribute__((unused)) TimerHandle_t xTimer) {
    esp_restart();
}

static void system_power_shutdown_cb(__attribute__((unused)) TimerHandle_t xTimer) {
    power_mgmt_on_enter_deep_sleep(false);
}

// uri:/v1/system/power
esp_err_t system_power_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    reset_power_save_timer();
    cJSON *root = cJSON_CreateObject();

    char target_state[CONFIG_PARAM_VALUE_MAX_LEN + 1] = {0};
    parse_url_kv_pair(req->uri, "target_state", target_state);
    cJSON_AddStringToObject(root, "target_state", target_state);

    TimerHandle_t system_power_timer = NULL;
    if (strcmp(target_state, "reboot") == 0) {
        system_power_timer = xTimerCreate("system_power_reboot_timer", pdMS_TO_TICKS(3 * 1000), pdFALSE, NULL, system_power_reboot_cb);
        cJSON_AddStringToObject(root, "status", "ok");
    } else if (strcmp(target_state, "shutdown") == 0) {
        system_power_timer = xTimerCreate("system_power_shutdown_timer", pdMS_TO_TICKS(3 * 1000), pdFALSE, NULL, system_power_shutdown_cb);
        cJSON_AddStringToObject(root, "status", "ok");
    } else {
        cJSON_AddStringToObject(root, "status", "invalid target_state value");
        httpd_resp_set_status(req, HTTPD_400);
    }

    const char *response = cJSON_Print(root);
    httpd_resp_sendstr(req, response);
    free((void *) response);
    cJSON_Delete(root);

    if (system_power_timer != NULL) {
        if (xTimerStart(system_power_timer, 0) == pdPASS) { // Starting the timer task
            ESP_LOGI(TAG, "system power timer started");
        }
    }

    return ESP_OK;
}

// uri:/v1/system/upgrade
esp_err_t system_upgrade_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    reset_power_save_timer();
    cJSON *root = cJSON_CreateObject();

    char ota_url[CONFIG_OTA_URL_LEN + 1] = {0};

    switch (req->method) {
        case HTTP_GET:
            cJSON_AddNumberToObject(root, "ota_err", g_mcu.state.ota_err);
            break;
        case HTTP_POST:
            ESP_LOGW(TAG, "%s", req->uri);
            parse_url_kv_pair(req->uri, "ota_url", ota_url);
            cJSON_AddStringToObject(root, "ota_url", ota_url);

            if (strlen(ota_url) > 0) {
                strcpy(g_mcu.ota_url, ota_url);
                set_sys_event(EV_SYS_OTA_TRIGGERED);
            } else {
                cJSON_AddStringToObject(root, "status", "empty ota_url");
                httpd_resp_set_status(req, HTTPD_500);
            }
            break;
        default:
            cJSON_AddStringToObject(root, "status", "invalid method");
            httpd_resp_set_status(req, HTTPD_400);
    }

    const char *response = cJSON_Print(root);
    httpd_resp_sendstr(req, response);
    free((void *) response);
    cJSON_Delete(root);
    return ESP_OK;
}

// uri:/v1/system/selftest
esp_err_t system_selftest_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    reset_power_save_timer();
    cJSON *root = cJSON_CreateObject();

    // TODO: Implement self-test
    cJSON_AddStringToObject(root, "status", "ok");

    const char *response = cJSON_Print(root);
    httpd_resp_sendstr(req, response);
    free((void *) response);
    cJSON_Delete(root);
    return ESP_OK;
}

// uri:/v1/system/power_mgmt
esp_err_t system_power_mgmt_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    reset_power_save_timer();
    cJSON *root = cJSON_CreateObject();

    cJSON_AddStringToObject(root, "mode", g_power_mgmt_ctx.mode == POWER_MODE_PERFORMANCE ? "performance" :
                                          g_power_mgmt_ctx.mode == POWER_MODE_NORMAL ? "normal" :
                                          g_power_mgmt_ctx.mode == POWER_MODE_LOW_ENERGY ? "low_energy" : "unknown");
    cJSON_AddBoolToObject(root, "no_sleep", g_power_mgmt_ctx.no_sleep);

    const char *response = cJSON_Print(root);
    httpd_resp_sendstr(req, response);
    free((void *) response);
    cJSON_Delete(root);
    return ESP_OK;
}

// uri:/v1/nvs
esp_err_t nvs_variable_list_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    reset_power_save_timer();
    cJSON *root = cJSON_CreateObject();
    cJSON *list = cJSON_AddArrayToObject(root, "names");

    // Add all the variable names to the list
    cJSON_AddItemToArray(list, cJSON_CreateString(CONFIG_NVS_WIFI_SSID_NAME));
    cJSON_AddItemToArray(list, cJSON_CreateString(CONFIG_NVS_WIFI_PSK_NAME));
    cJSON_AddItemToArray(list, cJSON_CreateString(CONFIG_NVS_DATA_HOST_NAME));
    cJSON_AddItemToArray(list, cJSON_CreateString(CONFIG_NVS_OTA_HOST_NAME));
    cJSON_AddItemToArray(list, cJSON_CreateString(CONFIG_NVS_NTP_HOST_NAME));
    cJSON_AddItemToArray(list, cJSON_CreateString(CONFIG_NVS_TEST_NAME));
    cJSON_AddItemToArray(list, cJSON_CreateString(CONFIG_NVS_IMU_BAUD_NAME));
    cJSON_AddItemToArray(list, cJSON_CreateString(CONFIG_NVS_SEQ_NAME));
    cJSON_AddItemToArray(list, cJSON_CreateString(CONFIG_NVS_TARGET_FPS_NAME));

    const char *response = cJSON_Print(root);
    httpd_resp_sendstr(req, response);
    free((void *) response);
    cJSON_Delete(root);
    return ESP_OK;
}

// uri:/v1/nvs/variable/*
esp_err_t nvs_variable_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    reset_power_save_timer();
    cJSON *root = cJSON_CreateObject();

    cJSON_AddStringToObject(root, "uri", req->uri);
    char *last_slash = strrchr(req->uri, '/');

    if (last_slash == NULL || strlen(last_slash) <= 1) {
        cJSON_AddStringToObject(root, "status", "empty name");
        httpd_resp_set_status(req, HTTPD_400);
    } else {
        char *name = last_slash + 1;
        char *name_end = strchr(name, '?');
        mcu_var_t *p_var = NULL;
        char value_buffer[CONFIG_VAR_STR_MAX_LEN + 1] = {0};
        esp_err_t err;

        switch (req->method) {
            case HTTP_GET:
                p_var = sys_find_var(name, strlen(name));
                if (p_var != NULL) {
                    sys_get_nvs_var(p_var, NULL, value_buffer);
                    cJSON_AddStringToObject(root, "value", value_buffer);
                    cJSON_AddStringToObject(root, "status", "ok");
                    break;
                } else {
                    cJSON_AddStringToObject(root, "status", "invalid name");
                    httpd_resp_set_status(req, HTTPD_400);
                    break;
                }
            case HTTP_POST:
                if (name_end == NULL || strlen(name_end) <= 1) {
                    cJSON_AddStringToObject(root, "status", "empty value");
                    httpd_resp_set_status(req, HTTPD_400);
                    break;
                }
                err = parse_url_kv_pair(req->uri, "value", value_buffer);
                if (err != ESP_OK) {
                    cJSON_AddStringToObject(root, "status", "invalid value");
                    httpd_resp_set_status(req, HTTPD_400);
                    break;
                } else {
                    p_var = sys_find_var(name, name_end - name);
                    if (p_var != NULL) {
                        err = sys_set_nvs_var(p_var, value_buffer);
                        if (err != ESP_OK) {
                            cJSON_AddStringToObject(root, "status", "cannot set value");
                            httpd_resp_set_status(req, HTTPD_500);
                        } else {
                            cJSON_AddStringToObject(root, "status", "ok");
                        }
                    } else {
                        cJSON_AddStringToObject(root, "status", "invalid name");
                        httpd_resp_set_status(req, HTTPD_400);
                    }
                    break;
                }
            default:
                cJSON_AddStringToObject(root, "status", "invalid method");
                httpd_resp_set_status(req, HTTPD_400);
        }
    }

    const char *response = cJSON_Print(root);
    httpd_resp_sendstr(req, response);
    free((void *) response);
    cJSON_Delete(root);
    return ESP_OK;
}

// uri:/v1/imu/calibrate
esp_err_t imu_calibrate_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    reset_power_save_timer();
    cJSON *root = cJSON_CreateObject();

    // TODO: Implement calibration
    cJSON_AddStringToObject(root, "status", "ok");

    const char *response = cJSON_Print(root);
    httpd_resp_sendstr(req, response);

    free((void *) response);
    cJSON_Delete(root);
    return ESP_OK;
}

// uri:/v1/imu/toggle
esp_err_t imu_toggle_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    reset_power_save_timer();
    cJSON *root = cJSON_CreateObject();

    char target_state[CONFIG_PARAM_VALUE_MAX_LEN + 1] = {0};
    parse_url_kv_pair(req->uri, "target_state", target_state);
    cJSON_AddStringToObject(root, "target_state", target_state);

    if (strcmp(target_state, "enable") == 0) {
        g_imu.toggle(g_imu.p_imu, true);
        cJSON_AddStringToObject(root, "status", "ok");
    } else if (strcmp(target_state, "disable") == 0) {
        g_imu.toggle(g_imu.p_imu, false);
        cJSON_AddStringToObject(root, "status", "ok");
    } else {
        cJSON_AddStringToObject(root, "status", "invalid target_state value");
        httpd_resp_set_status(req, HTTPD_400);
    }

    const char *response = cJSON_Print(root);
    httpd_resp_sendstr(req, response);
    free((void *) response);
    cJSON_Delete(root);
    return ESP_OK;
}

// uri:/v1/imu/status
esp_err_t imu_status_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    reset_power_save_timer();
    cJSON *root = cJSON_CreateObject();

    int ret = g_imu.enabled(g_imu.p_imu);
    cJSON_AddStringToObject(root, "power", ret ? "ON" : "OFF");
    cJSON_AddNumberToObject(root, "baud_rate", g_mcu.imu_baud);
    cJSON_AddStringToObject(root, "enabled", g_imu.p_imu->enabled ? "on" : "off");
    cJSON_AddNumberToObject(root, "imu_status", g_imu.p_imu->status);

    imu_dgram_t imu_data = {.seq=0};
    esp_err_t err = ESP_FAIL;
    switch (g_imu.p_imu->mux) {
        case IMU_MUX_DEBUG:
        case IMU_MUX_IDLE:
            g_imu.buffer_reset(g_imu.p_imu);
            err = g_imu.read(g_imu.p_imu, &imu_data, true);
            break;
        case IMU_MUX_STREAM:
            err = ring_buf_peek(&g_mcu.imu_ring_buf, &imu_data, -1, NULL);
            break;
    }
    if (err != ESP_OK) {
        cJSON_AddNullToObject(root, "imm");
    } else {
        cJSON *imm = cJSON_AddObjectToObject(root, "imm");
        cJSON *acc = cJSON_AddArrayToObject(imm, "acc");
        cJSON_AddItemToArray(acc, cJSON_CreateNumber(imu_data.imu.acc[0]));
        cJSON_AddItemToArray(acc, cJSON_CreateNumber(imu_data.imu.acc[1]));
        cJSON_AddItemToArray(acc, cJSON_CreateNumber(imu_data.imu.acc[2]));
        cJSON *gyr = cJSON_AddArrayToObject(imm, "gyr");
        cJSON_AddItemToArray(gyr, cJSON_CreateNumber(imu_data.imu.gyr[0]));
        cJSON_AddItemToArray(gyr, cJSON_CreateNumber(imu_data.imu.gyr[1]));
        cJSON_AddItemToArray(gyr, cJSON_CreateNumber(imu_data.imu.gyr[2]));
        cJSON *rpy = cJSON_AddArrayToObject(imm, "rpy");
        cJSON_AddItemToArray(rpy, cJSON_CreateNumber(imu_data.imu.eul[0]));
        cJSON_AddItemToArray(rpy, cJSON_CreateNumber(imu_data.imu.eul[1]));
        cJSON_AddItemToArray(rpy, cJSON_CreateNumber(imu_data.imu.eul[2]));
        cJSON *quat = cJSON_AddArrayToObject(imm, "quat");
        cJSON_AddItemToArray(quat, cJSON_CreateNumber(imu_data.imu.quat[0]));
        cJSON_AddItemToArray(quat, cJSON_CreateNumber(imu_data.imu.quat[1]));
        cJSON_AddItemToArray(quat, cJSON_CreateNumber(imu_data.imu.quat[2]));
        cJSON_AddItemToArray(quat, cJSON_CreateNumber(imu_data.imu.quat[3]));
        cJSON_AddNumberToObject(imm, "seq", imu_data.seq);
    }

    const char *response = cJSON_Print(root);
    httpd_resp_sendstr(req, response);
    free((void *) response);
    cJSON_Delete(root);
    return ESP_OK;
}

// uri:/v1/imu/debug/toggle
esp_err_t imu_debug_toggle_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    reset_power_save_timer();
    cJSON *root = cJSON_CreateObject();

    char mode[CONFIG_PARAM_VALUE_MAX_LEN + 1] = {0};
    parse_url_kv_pair(req->uri, ""
                                ""
                                ""
                                "", mode);
    cJSON_AddStringToObject(root, "target_state", mode);
    if (strcmp(mode, "enable") == 0) {
        g_imu.p_imu->mux = IMU_MUX_DEBUG;
        g_power_mgmt_ctx.no_sleep = true;
        cJSON_AddStringToObject(root, "status", "ok");
    } else if (strcmp(mode, "disable") == 0) {
        g_imu.p_imu->mux = IMU_MUX_IDLE;
        g_power_mgmt_ctx.no_sleep = false;
        cJSON_AddStringToObject(root, "status", "ok");
    } else {
        cJSON_AddStringToObject(root, "status", "invalid target_state value");
        httpd_resp_set_status(req, HTTPD_400);
    }

    const char *response = cJSON_Print(root);
    httpd_resp_sendstr(req, response);
    free((void *) response);
    cJSON_Delete(root);
    return ESP_OK;
}

/*
 * Structure holding server handle
 * and internal socket fd in order
 * to use out of request send
 */
struct async_resp_arg {
    httpd_handle_t hd;
    int fd;
};

SemaphoreHandle_t ws_async_task_mutex;
TaskHandle_t ws_async_send_task_handle;

static void ws_async_send_task(void *pvParameter) {
    ESP_LOGI(TAG, "ws_async_send_task started");
    struct async_resp_arg *resp_arg = (struct async_resp_arg *) pvParameter;
    httpd_handle_t hd = resp_arg->hd;
    int fd = resp_arg->fd;
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    os_delay_ms(500);

    /** create task mutex **/
    if (ws_async_task_mutex == NULL) {
        ws_async_task_mutex = xSemaphoreCreateMutex();
    }
    if (xSemaphoreTake(ws_async_task_mutex, portMAX_DELAY)) {
        uint8_t data[CONFIG_WEBSOCKET_FRAM_MAX_LEN];
        while (1) {
            if (g_imu.p_imu->mux != IMU_MUX_DEBUG) {
                ESP_LOGW(TAG, "IMU is not in debug mode");
                break;
            }
            size_t len = g_imu.read_bytes(g_imu.p_imu, data, CONFIG_WEBSOCKET_FRAM_MAX_LEN);
            ESP_LOGD(TAG, "len=%d", len);
            if (len <= 0) {
                continue;
            }
            ws_pkt.payload = data;
            ws_pkt.len = len;
            ws_pkt.type = HTTPD_WS_TYPE_BINARY;
            esp_err_t err = httpd_ws_send_frame_async(hd, fd, &ws_pkt);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "httpd_ws_send_frame_async got err=%d, hd=%p, fd=%d, ws_pkt=%p", err, &hd, fd, &ws_pkt);
                break;
            }
            os_delay_ms(100);
        }
        xSemaphoreGive(ws_async_task_mutex);
    }
    ESP_LOGI(TAG, "ws_async_send_task stopped");
    vTaskDelete(NULL);
}

static esp_err_t trigger_async_send(__attribute__((unused)) httpd_handle_t handle, httpd_req_t *req) {
    struct async_resp_arg resp_arg = {
        .hd = req->handle,
        .fd = httpd_req_to_sockfd(req)
    };
    xTaskCreate(ws_async_send_task, "ws_async_send_task", 3072, &resp_arg, 10, &ws_async_send_task_handle);
    return ESP_OK;
}

// uri:/v1/imu/debug/socket
esp_err_t imu_debug_socket_handler(httpd_req_t *req) {
    /** Handshake is done, so the new connection was opened **/
    if (req->method == HTTP_GET) {
        ESP_LOGD(TAG, "handshake done, the new connection was opened");
        /** Accept the connection if in debug mode **/
        if (g_imu.p_imu->mux == IMU_MUX_DEBUG) {
            trigger_async_send(req->handle, req);
            return ESP_OK;
        } else {
            ESP_LOGE(TAG, "IMU is not in debug mode");
            return ESP_FAIL;
        }
    }

    /** buf len is CONFIG_WEBSOCKET_FRAM_MAX_LEN+1 is for NULL termination as we are expecting a string **/
    uint8_t buf[CONFIG_WEBSOCKET_FRAM_MAX_LEN + 1];
    httpd_ws_frame_t ws_pkt = {
        .payload = buf,
        .type = HTTPD_WS_TYPE_TEXT
    };

    /** Set max_len = CONFIG_WEBSOCKET_FRAM_MAX_LEN to get the frame payload **/
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, CONFIG_WEBSOCKET_FRAM_MAX_LEN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
        return ret;
    } else {
        if (ws_pkt.len <= CONFIG_WEBSOCKET_FRAM_MAX_LEN) {
            if (ws_pkt.type != HTTPD_WS_TYPE_TEXT) {
                return ESP_FAIL;
            }
            ws_pkt.payload[ws_pkt.len] = 0;
            ESP_LOGI(TAG, "got packet(type: %d) with message: %s", ws_pkt.type, ws_pkt.payload);
            if (ws_pkt.len <= 0) {
                ESP_LOGD(TAG, "empty packet");
            } else {
                /** Proxy websocket writes to uart, add '\r\n' suffix **/
                char tx_buffer[CONFIG_WEBSOCKET_FRAM_MAX_LEN + 2];
                memcpy(tx_buffer, ws_pkt.payload, ws_pkt.len);
                tx_buffer[ws_pkt.len] = '\r';
                tx_buffer[ws_pkt.len + 1] = '\n';
                g_imu.write_bytes(g_imu.p_imu, (void *) tx_buffer, ws_pkt.len + 2);
            }
        } else {
            ESP_LOGW(TAG, "packet(len=%d) is too long, max length is %d", ws_pkt.len, CONFIG_WEBSOCKET_FRAM_MAX_LEN);
        }
    }
    return ret;
}

// uri:/v1/blink/configure
esp_err_t blink_configure_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    reset_power_save_timer();
    cJSON *root = cJSON_CreateObject();

    char tmp[CONFIG_PARAM_VALUE_MAX_LEN + 1];
    int seq;
    char *seq_end_ptr = NULL;
    char seq_str[4];
    bool success = true;
    switch (req->method) {
        case HTTP_GET:
            cJSON_AddNumberToObject(root, "seq", g_mcu.seq % 100U);
            cJSON_AddStringToObject(root, "mode", g_mcu.state.led_manual ? "manual" : "auto");
            break;
        case HTTP_POST:
            /** Configure LED Mode **/
            memset(tmp, 0, sizeof(tmp));
            parse_url_kv_pair(req->uri, "mode", tmp);
            cJSON_AddStringToObject(root, "mode", tmp);
            if (strlen(tmp) > 0) {
                if (strcmp(tmp, "auto") == 0) {
                    g_mcu.state.led_manual = false;
                    set_sys_event(EV_SYS_LED_STATUS_CHANGED);
                } else if (strcmp(tmp, "manual") == 0) {
                    g_mcu.state.led_manual = true;
                    set_sys_event(EV_SYS_LED_STATUS_CHANGED);
                } else if (strcmp(tmp, "off") == 0) {
                    g_mcu.state.led_manual = true;
                    blink_led_off();
                } else {
                    cJSON_AddStringToObject(root, "status", "invalid mode value");
                    httpd_resp_set_status(req, HTTPD_400);
                    success = false;
                    break;
                }
            }

            /** Configure LED Sequence **/
            memset(tmp, 0, sizeof(tmp));
            parse_url_kv_pair(req->uri, "seq", tmp);
            cJSON_AddStringToObject(root, "seq", tmp);
            seq = (int) strtol(tmp, &seq_end_ptr, 10);
            if (strlen(tmp) > 0) {
                if (seq_end_ptr >= tmp + strlen(tmp)) {
                    seq = seq % 0x100;
                    itoa(seq, seq_str, 10);
                    mcu_var_t *p_var = sys_find_var(CONFIG_NVS_SEQ_NAME, 3);
                    sys_set_nvs_var(p_var, (char *) seq_str);
                    g_mcu.seq = seq;
                } else {
                    cJSON_AddStringToObject(root, "status", "invalid seq value");
                    httpd_resp_set_status(req, HTTPD_400);
                    success = false;
                    break;
                }
            }
            break;
        default:
            cJSON_AddStringToObject(root, "status", "invalid method");
            httpd_resp_set_status(req, HTTPD_400);
            success = false;
    }
    if (success) {
        cJSON_AddStringToObject(root, "status", "ok");
    }

    const char *response = cJSON_Print(root);
    httpd_resp_sendstr(req, response);
    free((void *) response);
    cJSON_Delete(root);
    return ESP_OK;
}

// uri:/v1/blink/toggle
esp_err_t blink_toggle_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    reset_power_save_timer();
    cJSON *root = cJSON_CreateObject();

    char target_state[CONFIG_PARAM_VALUE_MAX_LEN + 1];
    int target_state_enum;
    esp_err_t err;

    switch (req->method) {
        case HTTP_GET:
            cJSON_AddNumberToObject(root, "state", g_mcu.state.led_status);
            break;
        case HTTP_POST:
            memset(target_state, 0, sizeof(target_state));
            parse_url_kv_pair(req->uri, "target_state", target_state);
            cJSON_AddStringToObject(root, "target_state", target_state);
            target_state_enum = (int) strtol(target_state, NULL, 10);
            if (target_state_enum > 0) {
                err = sys_set_led_status(target_state_enum);
                if (err == ESP_OK) {
                    cJSON_AddStringToObject(root, "status", "ok");
                } else {
                    cJSON_AddStringToObject(root, "status", "invalid target_state value");
                    httpd_resp_set_status(req, HTTPD_400);
                }
            } else {
                cJSON_AddStringToObject(root, "status", "invalid target_state value");
                httpd_resp_set_status(req, HTTPD_400);
            }
            break;
        default:
            cJSON_AddStringToObject(root, "status", "invalid method");
            httpd_resp_set_status(req, HTTPD_400);
    }

    const char *response = cJSON_Print(root);
    httpd_resp_sendstr(req, response);
    free((void *) response);
    cJSON_Delete(root);
    return ESP_OK;
}

// uri:/v1/operation/mode
esp_err_t operation_mode_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    reset_power_save_timer();
    cJSON *root = cJSON_CreateObject();

    char action_state[CONFIG_PARAM_VALUE_MAX_LEN + 1] = {0};

    switch (req->method) {
        case HTTP_GET:
            cJSON_AddBoolToObject(root, "active", g_mcu.state.active);
            //cJSON_AddNumberToObject(root, "imu_status", g_imu.status);
            cJSON_AddStringToObject(root, "imu_status", g_imu.p_imu->status == IMU_STATUS_FAIL ? "fail" :
                                                        sys_get_operation_mode() ? "running":
                                                        g_imu.p_imu->status == IMU_STATUS_READY ? "ready" : "unknown");
            break;
        case HTTP_POST:
            parse_url_kv_pair(req->uri, "action", action_state);
            cJSON_AddStringToObject(root, "action", action_state);
            cJSON_AddNumberToObject(root, "imu_status", g_imu.p_imu->status);
            if (strcmp(action_state, "start") == 0) {
                sys_set_operation_mode(true);
                cJSON_AddStringToObject(root, "status", "ok");
            } else if (strcmp(action_state, "stop") == 0) {
                sys_set_operation_mode(false);
                cJSON_AddStringToObject(root, "status", "ok");
            } else {
                cJSON_AddStringToObject(root, "status", "invalid action value");
                httpd_resp_set_status(req, HTTPD_400);
            }
            break;
        default:
            cJSON_AddStringToObject(root, "status", "invalid method");
            httpd_resp_set_status(req, HTTPD_400);
    }

    const char *response = cJSON_Print(root);
    httpd_resp_sendstr(req, response);
    free((void *) response);
    cJSON_Delete(root);
    return ESP_OK;
}
