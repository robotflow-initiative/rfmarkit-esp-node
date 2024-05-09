#include <string.h>

#include "esp_http_server.h"
#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_vfs.h"
#include "cJSON.h"
#include "sdkconfig.h"

#include "settings.h"
#include "rest_controller.h"
#include "rest_controller_handlers.h"

#define REST_CHECK(a, str, goto_tag, ...)                                              \
    do                                                                                 \
    {                                                                                  \
        if (!(a))                                                                      \
        {                                                                              \
            ESP_LOGE(TAG, "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__);      \
            goto goto_tag;                                                             \
        }                                                                              \
    } while (0)

/**
 * @struct rest_server_context
 * @brief Context for the RESTful server
 * @var base_path: base path for the RESTful server
 * @var buffer: buffer for requests
 */
typedef struct rest_server_context {
    char base_path[ESP_VFS_PATH_MAX + 1];
    char buffer[CONFIG_USER_CTX_BUFSIZE];
} rest_server_context_t;

static const char *TAG = "sys.controller";

/**
----------------------------------------------------------------------------------
NEW API                 PATH                    Type        Function
----------------------------------------------------------------------------------
system_info             /v1/system/info         [get]       ping,id,ver,time
system_power            /v1/system/power        [post]      restart, shutdown, #version_shutdown
system_upgrade          /v1/system/upgrade      [get|post]  update
system_selftest         /v1/system/selftest     [post]      self_test
system_power_mgmt       /v1/system/power_mgmt   [post]      always_on, cancel_always_on
nvs_variable            /v1/nvs/variable/<name> [get|post]  varset,varget
imu_calibrate           /v1/imu/calibrate       [post]      imu_cali_reset, imu_cali_acc, imu_cali_mag
imu_toggle              /v1/imu/toggle          [post]      imu_enable,imu_disable,
imu_status              /v1/imu/status          [get]       imu_status imu_imm
imu_debug_toggle        /v1/imu/debug/toggle    [post]      // toggle debug mode and disconnect uart_monitor
imu_debug_socket        /v1/imu/debug/socket    [ws]        imu_debug imu_setup
blink_configure         /v1/blink/configure     [get|post]  blink_set, blink_get, auto/manual
blink_toggle            /v1/blink/toggle        [get|post]  blink_start, blink_stop, blink_mute, also get led status
operation_mode          /v1/operation/mode      [get|post]  start stop
----------------------------------------------------------------------------------
**/

static httpd_uri_t controller_uri[] = {
        {.uri = "/v1/system/info", .method = HTTP_GET, .handler = system_info_handler,},
        {.uri = "/v1/system/power", .method = HTTP_POST, .handler = system_power_handler,},
        {.uri = "/v1/system/upgrade", .method = HTTP_GET, .handler = system_upgrade_handler,},
        {.uri = "/v1/system/upgrade", .method = HTTP_POST, .handler = system_upgrade_handler,},
        {.uri = "/v1/system/selftest", .method = HTTP_POST, .handler = system_selftest_handler,},
        {.uri = "/v1/system/power_mgmt", .method = HTTP_POST, .handler = system_power_mgmt_handler,},
        {.uri = "/v1/nvs/variable/*", .method = HTTP_GET, .handler = nvs_variable_handler,},
        {.uri = "/v1/nvs/variable/*", .method = HTTP_POST, .handler = nvs_variable_handler,},
        {.uri = "/v1/imu/calibrate", .method = HTTP_POST, .handler = imu_calibrate_handler,},
        {.uri = "/v1/imu/toggle", .method = HTTP_POST, .handler = imu_toggle_handler,},
        {.uri = "/v1/imu/status", .method = HTTP_GET, .handler = imu_status_handler,},
        {.uri = "/v1/imu/debug/toggle", .method = HTTP_POST, .handler = imu_debug_toggle_handler,},
        {.uri = "/v1/imu/debug/socket", .method =HTTP_GET, .handler = imu_debug_socket_handler, .is_websocket = true},
        {.uri = "/v1/blink/configure", .method = HTTP_GET, .handler = blink_configure_handler,},
        {.uri = "/v1/blink/configure", .method = HTTP_POST, .handler = blink_configure_handler,},
        {.uri = "/v1/blink/toggle", .method = HTTP_GET, .handler = blink_toggle_handler,},
        {.uri = "/v1/blink/toggle", .method = HTTP_POST, .handler = blink_toggle_handler,},
        {.uri = "/v1/operation/mode", .method = HTTP_GET, .handler = operation_mode_handler,},
        {.uri = "/v1/operation/mode", .method = HTTP_POST, .handler = operation_mode_handler,},
};

/**
 * @brief Default error handler
 * @param req
 * @param error
 * @return
**/
esp_err_t default_err_handler(httpd_req_t *req, httpd_err_code_t error) {
    httpd_resp_set_type(req, "application/json");
    cJSON *root = cJSON_CreateObject();

    cJSON_AddStringToObject(root, "responder", "default_not_found_err_handler");
    cJSON_AddStringToObject(root, "path", req->uri);
    cJSON_AddNumberToObject(root, "error", error);

    const char *response = cJSON_Print(root);
    httpd_resp_sendstr(req, response);

    free((void *) response);
    cJSON_Delete(root);
    return ESP_OK;
}

/**
 * Registering uri handlers
 * @param server
 * @param rest_context
 * @return error code
**/
esp_err_t rest_server_register(httpd_handle_t server, rest_server_context_t *rest_context) {
    /** Registering uri handlers*/
    for (int idx = 0; idx < sizeof(controller_uri) / sizeof(httpd_uri_t); idx++) {
        httpd_uri_t *uri_ptr = &controller_uri[idx];
        uri_ptr->user_ctx = rest_context;
        httpd_register_uri_handler(server, uri_ptr);
    }
    httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, default_err_handler);
    return ESP_OK;
}

/**
 * Start the RESTful controller at a base path
 * @param base_path
 * @return httpd_handle_t
**/
httpd_handle_t rest_controller_start(const char *base_path) {
    /** Check the base path **/
    REST_CHECK(base_path, "wrong base path", err);
    rest_server_context_t *rest_context = calloc(1, sizeof(rest_server_context_t));
    REST_CHECK(rest_context, "no memory for rest context", err);
    strlcpy(rest_context->base_path, base_path, sizeof(rest_context->base_path));

    /** Configuration for the httpd server **/
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.server_port = CONFIG_CONTROLLER_LISTEN_PORT;
    config.max_uri_handlers = sizeof(controller_uri) / sizeof(httpd_uri_t);

    ESP_LOGI(TAG, "starting RESTful controller");
    REST_CHECK(httpd_start(&server, &config) == ESP_OK, "start controller failed", err_start);

    /** URI handlers **/
    rest_server_register(server, rest_context);

    return server;
err_start:
    free(rest_context);
err:
    return NULL;
}

/**
 * Stop the RESTful controller
**/
esp_err_t rest_controller_stop(httpd_handle_t server) {
    /** Stop the httpd server **/
    httpd_stop(server);
    return ESP_OK;
}