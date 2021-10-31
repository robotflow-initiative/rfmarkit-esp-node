#ifndef _SETTINGS_H
#define _SETTINGS_H

#include "driver/gpio.h"

#define USE_PSEUDO_VALUE 0
#define CONFIG_EN_DEBUG_OVER_TCP 0

#define CONFIG_MAX_TX_POWER (8)
#define CONFIG_MULTI_CORE 0
#define CONFIG_MSG_QUEUE_LEN 256

/** LAB and Non LAB environment has different network **/
#define ENV 3
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
#define CONFIG_SEND_PARSED 1
#define CONFIG_PAYLOAD_BUFFER_LEN 512

/** GY95 related settings **/
#define GY95_MSG_LEN 40
#define GY95_CTRL_PIN GPIO_NUM_5
#define GY95_CTRL_PIN_MASK (1ULL << GY95_CTRL_PIN)
#define GY95_ADDR 0xa4
#define GY95_READ_OP 0x03
#define GY95_REG_THRESH 0x2c
#define GY95_DEFAULT_START_REG 0x14
#define GY95_PORT (1) // UART_NUM_X
#define GY95_RX GPIO_NUM_6
#define GY95_TX GPIO_NUM_7
#define GY95_RTS GPIO_NUM_4
#define GY95_CTS GPIO_NUM_8
#define GY95_N_TICK 10

/** NTP settings **/
#define CONFIG_NTP_SERVER_ADDR CONFIG_HOST_IP_ADDR
#define CONFIG_NTP_MAX_RETRY 2
#define CONFIG_NTP_UPDATE_INTERVAL_MS 1200000

#endif