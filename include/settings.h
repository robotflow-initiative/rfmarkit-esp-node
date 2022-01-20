#ifndef _SETTINGS_H
#define _SETTINGS_H

#define CONFIG_FIRMWARE_VERSION "2.6.0"

#include "driver/gpio.h"

/** Debug options **/
#define CONFIG_USE_PSEUDO_VALUE 0
#define CONFIG_EN_DEBUG_OVER_TCP 0
#define CONFIG_EN_PARSER_DEBUG 0
#define CONFIG_EN_IMU_DEBUG 0

/** System settings **/
#define CONFIG_MAIN_LOOP_COUNT_PERIOD_MS 10000
#define CONFIG_MAIN_LOOP_MAX_COUNT_NUM 18
#define CONFIG_MAX_TX_POWER (68)
#define CONFIG_MULTI_CORE 0
#define CONFIG_SERIAL_QUEUE_LEN 128
#define CONFIG_DEVICE_ID_LEN 12
#define CONFIG_EN_IMU 1

#define IMU_TYPE_GY95 95
#define IMU_TYPE_HI229 229
#define CONFIG_IMU_TYPE IMU_TYPE_HI229

/** Wi-Fi environment settings**/
#define ENV 4
#if ENV == 0
#define CONFIG_ESP_WIFI_SSID "yz_ri"
#define CONFIG_ESP_WIFI_PASSWORD "yzri@1220"
#define CONFIG_HOST_IP_ADDR "10.52.21.125"
#define CONFIG_HOST_PORT 18889
#define CONFIG_LOCAL_PORT 18888
#elif ENV == 1
#define CONFIG_ESP_WIFI_SSID "SPEIT-105_IoT"
#define CONFIG_ESP_WIFI_PASSWORD "IoTIoTIoT"
#define CONFIG_HOST_IP_ADDR "192.168.1.91"
#define CONFIG_HOST_PORT 18888
#define CONFIG_LOCAL_PORT 18888
#elif ENV == 2
#define CONFIG_ESP_WIFI_SSID "Yutong-LI-Phone4"
#define CONFIG_ESP_WIFI_PASSWORD "88888888"
#define CONFIG_HOST_IP_ADDR "172.20.10.10"
#define CONFIG_HOST_PORT 18888
#define CONFIG_LOCAL_PORT 18888
#elif ENV == 3
#define CONFIG_ESP_WIFI_SSID "yz_sensor"
#define CONFIG_ESP_WIFI_PASSWORD "yzri@1220"
#define CONFIG_HOST_IP_ADDR "10.52.21.125"
#define CONFIG_HOST_PORT 18888
#define CONFIG_LOCAL_PORT 18888
#elif ENV == 4
#define CONFIG_ESP_WIFI_SSID "yz_sensor"
#define CONFIG_ESP_WIFI_PASSWORD "yzri@1220"
#define CONFIG_HOST_IP_ADDR "10.53.21.102"
#define CONFIG_HOST_PORT 18888
#define CONFIG_LOCAL_PORT 18888
// TODO: Make wifi configurable
// TODO: Remove redandent settings
#endif

/** OTA debugging is not different in debug mode and non debug mode **/
#define CONFIG_OTA_PORT "5138"
#define CONFIG_OTA_APIHOST "http://"CONFIG_HOST_IP_ADDR":"CONFIG_OTA_PORT"/firmware.bin" // TODO: Upgrading via HTTP is not safe
#define CONFIG_OTA_MAXIMUM_RETRY 1

/** Retry limits **/
#define CONFIG_ESP_MAXIMUM_RETRY 10
#define CONFIG_ESP_TCP_MAXIMUM_RETRY 5
#define CONFIG_ESP_LIGHT_SLEEP_MAXIMUM_RETRY 10
#define CONFIG_DISCONNECT_SLEEP_NUS 30000000

/** Send parsed json **/
#define CONFIG_SEND_PARSED 0
#undef CONFIG_PAYLOAD_BUFFER_LEN
#if CONFIG_SEND_PARSED
#define CONFIG_PAYLOAD_BUFFER_LEN 512
#else
#define CONFIG_PAYLOAD_BUFFER_LEN 40
#endif


/** NTP settings **/
#define CONFIG_LOCAL_TZ "CTS-8"
#define CONFIG_NTP_SERVER_ADDR CONFIG_HOST_IP_ADDR
#define CONFIG_NTP_MAX_RETRY 2
#define CONFIG_NTP_TIMEOUT_S 20
#define CONFIG_NTP_UPDATE_INTERVAL_MS 1200000

/** On Board Button **/
#include "pins.h"


#endif