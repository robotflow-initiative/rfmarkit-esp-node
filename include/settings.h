#ifndef _SETTINGS_H
#define _SETTINGS_H

#include "driver/gpio.h"

#define DEBUG 0

#define CONFIG_MAX_TX_POWER (8)
#define CONFIG_MULTI_CORE 0
#define CONFIG_MSG_QUEUE_LEN 512

/** LAB and Non LAB environment has different network **/
#define LAB 1
#if LAB
#define CONFIG_ESP_WIFI_SSID "yz_ri"
#define CONFIG_ESP_WIFI_PASSWORD "yzri@1220"
#define CONFIG_HOST_IP_ADDR "10.52.21.125"
#define CONFIG_HOST_PORT 18888
#define CONFIG_LOCAL_PORT 18888
#else
#define CONFIG_ESP_WIFI_SSID "SPEIT-105_IoT"
#define CONFIG_ESP_WIFI_PASSWORD "IoTIoTIoT"
#define CONFIG_HOST_IP_ADDR "192.168.1.91"
#define CONFIG_HOST_PORT 18888
#define CONFIG_LOCAL_PORT 18888
#endif

/** OTA debugging is not different in debug mode and non debug mode **/
#if DEBUG
#define CONFIG_OTA_PORT "5139"
#else
#define CONFIG_OTA_PORT "5138"
#endif

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
#define GY95_RX (6)
#define GY95_TX (7)
#define GY95_RTS (4)
#define GY95_CTS (5)
#define GY95_N_TICK 10

/** NTP settings **/
#define CONFIG_NTP_SERVER_ADDR "ntp.ntsc.ac.cn"
#define CONFIG_NTP_MAX_RETRY 2

#endif