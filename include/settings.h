#ifndef SETTINGS_H_
#define SETTINGS_H_

/** System settings **/
#define CONFIG_TARGET_FPS                   100
#define CONFIG_EN_MULTI_CORE                1
#define CONFIG_EN_PROFILING                 0
#define CONFIG_EN_FPS_PROFILING             1
#define CONFIG_EN_READ_FPS_LIM              1

/** Hardware related settings **/
#define CONFIG_DATA_BUF_LEN                 128
#define CONFIG_DEVICE_ID_LEN                12

/** BLE related settings **/
#define CONFIG_BLE_LOCAL_NAME_LEN           32
#define CONFIG_BLE_LOCAL_NAME_PREFIX        "markit_"

/** OTA related settings **/
#define CONFIG_OTA_URL_LEN                  128

/** Wi-Fi&Network settings**/
#define CONFIG_NORMAL_TX_POWER              (34)
#define CONFIG_MAX_TX_POWER                 (68)
#define CONFIG_WIFI_SSID                    "markit"
#define CONFIG_WIFI_PSK                     "markit"
#define CONFIG_DATA_HOST_IP_ADDR            "255.255.255.255"   // broadcast address by default
#define CONFIG_DATA_HOST_PORT               18891
#define CONFIG_DISCOVERY_PORT               18889
#define CONFIG_CONTROLLER_LISTEN_PORT       18888
#define CONFIG_CONTROLLER_BASE_PATH         "/"
#define CONFIG_WIFI_MAX_RETRY               5

/** NVS settings **/
#define CONFIG_NVS_WIFI_SSID_NAME           "WIFI_SSID"
#define CONFIG_NVS_WIFI_PSK_NAME            "WIFI_PSK"
#define CONFIG_NVS_DATA_HOST_NAME           "DATA_HOST"
#define CONFIG_NVS_OTA_HOST_NAME            "OTA_HOST"
#define CONFIG_NVS_NTP_HOST_NAME            "NTP_HOST"
#define CONFIG_NVS_TEST_NAME                "TEST"
#define CONFIG_NVS_IMU_BAUD_NAME            "IMU_BAUD"
#define CONFIG_NVS_SEQ_NAME                 "SEQ"
#define CONFIG_NVS_TARGET_FPS_NAME          "TARGET_FPS"

/** Retry limits **/
#define CONFIG_ESP_MAXIMUM_RETRY            20

/** NTP settings **/
#define CONFIG_NTP_HOST_IP_ADDR             ""                  // left blank to use GW as ntp server by default
#define CONFIG_LOCAL_TZ                     "CTS-8"
#define CONFIG_NTP_HOST_IP_ADDR_BACKUP      "202.120.2.101"     // ntp.sjtu.edu.cn
#define CONFIG_NTP_PRIMARY_MAX_RETRY        2
#define CONFIG_NTP_TIMEOUT_S                5
#define CONFIG_NTP_UPDATE_INTERVAL_S        1200

/** Discovery Related **/
#define CONFIG_DISCOVERY_INTERVAL_S         15
#define CONFIG_DISCOVERY_REPLY_TIMEOUT_S    5

/** Power Management Related **/
#define CONFIG_POWER_MGMT_DURATION_S        20
#define CONFIG_POWER_SAVE_TIMEOUT_S         300
#define CONFIG_POWER_SAVE_CYCLE_MAX_COUNT   300
#define CONFIG_POWER_SAVE_MOTION_LIMIT      0.04
#define CONFIG_POWER_WAKE_UP_DURATION_S     30

/** Button Related **/
#define CONFIG_LONG_PRESS_DURATION          (1000 / portTICK_PERIOD_MS)
#define CONFIG_DOUBLE_CLICK_DURATION        (500 / portTICK_PERIOD_MS)

/** On Board Button **/
#include "modelspec.h"


#endif