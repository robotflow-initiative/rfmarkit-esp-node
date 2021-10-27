#include "funcs.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "settings.h"

static const char * TAG = "func_ota";

esp_err_t esp_do_ota() {
    esp_http_client_config_t config = {
            .url = CONFIG_OTA_APIHOST,
            .max_authorization_retries = CONFIG_OTA_MAXIMUM_RETRY,
            .auth_type = HTTP_AUTH_TYPE_NONE,
        };
        esp_err_t ret = esp_https_ota(&config);
        if (ret == ESP_OK) {
            esp_restart();
        } else {
            ESP_LOGI(TAG, "OTA update error, return code: %d", (int)ret);
            return ESP_FAIL;
        }
        return ESP_OK;
}