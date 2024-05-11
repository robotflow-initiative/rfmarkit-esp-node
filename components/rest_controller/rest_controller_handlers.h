#ifndef REST_CONTROLLER_HANDLERS_
#define REST_CONTROLLER_HANDLERS_

#include "esp_err.h"
#include "esp_http_server.h"

// clang-format off

esp_err_t system_info_handler(httpd_req_t *req);

esp_err_t system_power_handler(httpd_req_t *req);

esp_err_t system_upgrade_handler(httpd_req_t *req);

esp_err_t system_selftest_handler(httpd_req_t *req);

esp_err_t system_power_mgmt_handler(httpd_req_t *req);

esp_err_t nvs_variable_handler(httpd_req_t *req);

esp_err_t imu_calibrate_handler(httpd_req_t *req);

esp_err_t imu_toggle_handler(httpd_req_t *req);

esp_err_t imu_status_handler(httpd_req_t *req);

esp_err_t imu_debug_toggle_handler(httpd_req_t *req);

esp_err_t imu_debug_socket_handler(httpd_req_t *req);

esp_err_t blink_configure_handler(httpd_req_t *req);

esp_err_t blink_toggle_handler(httpd_req_t *req);

esp_err_t operation_mode_handler(httpd_req_t *req);

// clang-format on

#endif