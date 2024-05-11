#ifndef REST_CONTROLLER_
#define REST_CONTROLLER_

#include "esp_err.h"
#include "esp_http_server.h"

#define CONFIG_USER_CTX_BUFSIZE 64

httpd_handle_t rest_controller_start(const char *base_path);

esp_err_t rest_controller_stop(httpd_handle_t server);

#endif 