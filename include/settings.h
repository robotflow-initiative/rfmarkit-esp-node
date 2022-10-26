#ifndef SETTINGS_H_
#define SETTINGS_H_

#include "../VERSION"

/** Debug options **/
#define CONFIG_EN_DEBUG_OVER_TCP 0

/** System settings **/
#define CONFIG_MAIN_LOOP_DUTY_PERIOD_MS 1000 // Duty cycle of main loop, the main loop checks system events
#define CONFIG_MAIN_LOOP_MAX_LOOP_NUM 360  // The system goes to sleep after CONFIG_MAIN_LOOP_MAX_LOOP_NUM cycles

/** Hardware related settings **/
#define CONFIG_MULTI_CORE 1
#define CONFIG_SERIAL_QUEUE_LEN 128
#define CONFIG_DEVICE_ID_LEN 12

/** Peripheral related settings **/
#define IMU_TYPE_GY95 95
#define IMU_TYPE_HI229 229
#define CONFIG_IMU_TYPE IMU_TYPE_HI229

/** Wi-Fi environment settings**/
#define CONFIG_MAX_TX_POWER (68)
#define ENV 4
#if ENV == 0
#define CONFIG_WIFI_SSID "yz_ri"
#define CONFIG_WIFI_PASSWORD "yzri@1220"
#define CONFIG_HOST_IP_ADDR "10.52.21.125"
#define CONFIG_HOST_PORT 18889
#define CONFIG_LOCAL_PORT 18888
#elif ENV == 1
#define CONFIG_WIFI_SSID "SPEIT-105_IoT"
#define CONFIG_WIFI_PASSWORD "IoTIoTIoT"
#define CONFIG_HOST_IP_ADDR "192.168.1.91"
#define CONFIG_HOST_PORT 18888
#define CONFIG_LOCAL_PORT 18888
#elif ENV == 2
#define CONFIG_WIFI_SSID "Yutong-LI-Phone4"
#define CONFIG_WIFI_PASSWORD "88888888"
#define CONFIG_HOST_IP_ADDR "172.20.10.10"
#define CONFIG_HOST_PORT 18888
#define CONFIG_LOCAL_PORT 18888
#elif ENV == 3
#define CONFIG_WIFI_SSID "yz_sensor"
#define CONFIG_WIFI_PASSWORD "yzri@1220"
#define CONFIG_HOST_IP_ADDR "10.52.21.125"
#define CONFIG_HOST_PORT 18888
#define CONFIG_LOCAL_PORT 18888
#elif ENV == 4
#define CONFIG_WIFI_SSID "yz_sensor"
#define CONFIG_WIFI_PSK "yzri@1220"
#define CONFIG_DATA_HOST_IP_ADDR "10.53.21.102"
#define CONFIG_DATA_HOST_PORT 18888
#define CONFIG_LOCAL_PORT 18888

#endif

/** OTA debugging is not different in debug mode and non debug mode **/
#define CONFIG_OTA_PORT 5138
#define CONFIG_OTA_HOST_IP_ADDR "10.53.21.164"

/** Retry limits **/
#define CONFIG_ESP_MAXIMUM_RETRY 10
#define CONFIG_PAYLOAD_BUFFER_LEN 140

/** NTP settings **/
#define CONFIG_LOCAL_TZ "CTS-8"
#define CONFIG_NTP_HOST_IP_ADDR "10.53.21.164"
#define CONFIG_NTP_HOST_IP_ADDR_BACKUP "202.120.2.101"

#define CONFIG_NTP_MAX_RETRY 2
#define CONFIG_NTP_TIMEOUT_S 20
#define CONFIG_NTP_UPDATE_INTERVAL_MS 1200000

#define CONFIG_CTRL_RX_LEN 64
#define CONFIG_CTRL_TX_LEN 512

#define CONFIG_BLINK_USE_PWM 1

/** On Board Button **/
#include "modelspec.h"


#endif