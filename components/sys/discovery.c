//
// Created by liyutong on 2024/4/30.
//
#include <stdlib.h>

#include "esp_log.h"

#include <lwip/inet.h>
#include <lwip/sockets.h>

#include "apps.h"
#include "settings.h"
#include "udp.h"
#include "sys.h"

#define CONFIG_DEVICE_TYPE_INTERFACE                    0
#define CONFIG_DEVICE_TYPE_MARKER                       1
#define CONFIG_DEVICE_TYPE_PC                           2
#define CONFIG_MAGIC_MARKER_ADDR                        229
#define CONFIG_MAGIC_NUMBER_LENGTH                      5
#define CONFIG_MAGIC_DEVICE_TYPE_INDEX                  5
#define CONFIG_MAGIC_DEVICE_TYPE_LENGTH                 1
#define CONFIG_MAGIC_REPLY_FLAG_INDEX                   6
#define CONFIG_MAGIC_REPLY_FLAG_LENGTH                  1

#define CONFIG_DISCOVERY_BROADCAST_ADDR                 "255.255.255.255"
#define CONFIG_DISCOVERY_BROADCAST_REPLY_CONTROL_OFFSET 6
#define CONFIG_DISCOVERY_BROADCAST_DEVICE_ID_OFFSET     7

typedef struct {
    char ntp_host[32];
    char ota_host[32];
} MarkerDiscoveryReplyPacket;

static char discovery_msg_buffer[] = "\xe5\xe5\xe5\xe5\xe5\x01\x00""000000000000"; // magic header
static SemaphoreHandle_t sync_mutex = NULL;
static udp_socket_t client = {0};

static const char *TAG = "sys.discovery";

/**
 * @brief Prepare discovery message buffer ONCE
 * @return
**/
static esp_err_t sys_prepare_discovery_msg_buffer_once() {
    memcpy(discovery_msg_buffer + CONFIG_DISCOVERY_BROADCAST_DEVICE_ID_OFFSET, g_mcu.device_id, sizeof(g_mcu.device_id));
    return ESP_OK;
}

/**
 * @brief Set discovery reply control flag to a prepared message
 * @param reply
 * @return
**/
static esp_err_t sys_set_discovery_msg_reply_control(bool reply) {
    discovery_msg_buffer[CONFIG_DISCOVERY_BROADCAST_REPLY_CONTROL_OFFSET] = reply ? '\x01' : '\x00';
    return ESP_OK;
}

/**
 * Decode discovery reply control packet
 * @param buf
 * @param buf_len
 * @out pkt
**/
static esp_err_t sys_decode_discovery_reply_control(const uint8_t *buf, size_t buf_len, MarkerDiscoveryReplyPacket *pkt) {
    if (buf_len <
        CONFIG_MAGIC_NUMBER_LENGTH + CONFIG_MAGIC_DEVICE_TYPE_LENGTH + CONFIG_MAGIC_REPLY_FLAG_LENGTH + sizeof(MarkerDiscoveryReplyPacket)) {
        return ESP_FAIL;
    }

    /** detect first five bytes **/
    for (int i = 0; i < CONFIG_MAGIC_NUMBER_LENGTH; i++) {
        if (buf[i] != CONFIG_MAGIC_MARKER_ADDR) {
            ESP_LOGW(TAG, "magic number is not matched");
            return ESP_FAIL;
        }
    }
    /** this byte must match with source **/
    uint8_t source = buf[CONFIG_MAGIC_DEVICE_TYPE_INDEX];
    if (source != CONFIG_DEVICE_TYPE_INTERFACE) {
        ESP_LOGW(TAG, "source device type(%d) is not correct", source);
        return ESP_FAIL;
    }
    // bool reply = buf[CONFIG_MAGIC_DEVICE_TYPE_INDEX + 1];

    /** unpack the payload **/
    const uint8_t *packetData = buf + CONFIG_MAGIC_REPLY_FLAG_INDEX + 1;
    memcpy(pkt, packetData, sizeof(MarkerDiscoveryReplyPacket));

    return ESP_OK;
}

/**
 * @brief Discovery handler, this function is called when EV_TASK_DISCOVERY is set
 * @param handler_args
 * @param base
 * @param id
 * @param event_data
**/
void sys_discovery_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data) {
    if (!sync_mutex) {
        /** Create mutex **/
        sync_mutex = xSemaphoreCreateMutex();
        sys_prepare_discovery_msg_buffer_once();
    }
    if (xSemaphoreTake(sync_mutex, portMAX_DELAY) == pdTRUE) {
        ESP_LOGD(TAG, "sys_discovery event is handled");
        esp_err_t err;
        if (!client.initialized) {
            udp_socket_init(&client, CONFIG_DISCOVERY_PORT, CONFIG_DISCOVERY_BROADCAST_ADDR, CONFIG_DISCOVERY_PORT);
            udp_socket_set_timeout(&client, CONFIG_DISCOVERY_REPLY_TIMEOUT_S);
        }

        /** Update last try timestamp record **/
        get_time_usec(g_mcu.state.last_discovery_unix_usec);

        if (g_mcu.state.discovery_completed) {
            /** discovery is completed, only send keep-alive packets **/
            sys_set_discovery_msg_reply_control(false);
            err = udp_socket_send(&client, (uint8_t *) discovery_msg_buffer, sizeof(discovery_msg_buffer) - 1);
            ESP_LOGD(TAG, "sending keep-alive packets tp %s:%d", inet_ntoa(client.dest_addr.sin_addr), ntohs(client.dest_addr.sin_port));
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "error occurred during sending (keep-alive)");
            }
        } else {
            /** discovery is not done, broadcasting and receiving **/
            sys_set_discovery_msg_reply_control(true);
            err = udp_socket_send(&client, (uint8_t *) discovery_msg_buffer, sizeof(discovery_msg_buffer) - 1);

            if (err != ESP_OK) {
                ESP_LOGE(TAG, "error occurred during sending (register): errno %d", errno);
                goto exit;
            }

            static uint8_t discovery_reply_buffer[128];
            int64_t time_start = esp_timer_get_time();
            bool reply_received = false;

            /** wait for reply **/
            while ((esp_timer_get_time() - time_start) < CONFIG_DISCOVERY_REPLY_TIMEOUT_S * 1000000L) {
                size_t len;
                err = udp_socket_recv(&client, discovery_reply_buffer, sizeof(discovery_reply_buffer) - 1, &len);
                if (err != ESP_OK) {
                    continue;   // ignore
                } else {
                    /** Parse Replay **/
                    MarkerDiscoveryReplyPacket pkt;
                    err = sys_decode_discovery_reply_control(discovery_reply_buffer, len, &pkt);
                    if (err == ESP_OK) {
                        ESP_LOGI(TAG, "reply received from %s, ntp_host=%s, ota_host=%s", inet_ntoa(client.reply_addr.sin_addr), pkt.ntp_host, pkt.ota_host);
                        reply_received = true;
                        /** Update data_host_ip **/
                        memcpy(g_mcu.data_host_ip_addr, inet_ntoa(client.reply_addr.sin_addr), strlen(inet_ntoa(client.reply_addr.sin_addr)));

                        /** Update NTP and OTA host **/
                        if (strlen(pkt.ntp_host) > 0) {
                            memcpy(g_mcu.ntp_host_ip_addr, &pkt.ntp_host, strlen(pkt.ntp_host));
                        }
                        if (strlen(pkt.ota_host) > 0) {
                            memcpy(g_mcu.ntp_host_ip_addr, &pkt.ntp_host, strlen(pkt.ntp_host));
                        }
                        ESP_LOGI(TAG, "discovery completed, detected interface address: %s", g_mcu.data_host_ip_addr);
                        g_mcu.state.discovery_completed = true;
                        set_sys_event(EV_SYS_DISCOVERY_COMPLETED);

                        /** Update interface address **/
                        udp_socket_set_destination(&client, inet_ntoa(client.reply_addr.sin_addr), CONFIG_DISCOVERY_PORT);
                        break;
                    }
                }
            }
            if (!reply_received) {
                ESP_LOGW(TAG, "discovery failed, no reply received");
            }
        }

exit:
        xSemaphoreGive(sync_mutex);
        clear_task_event(EV_TASK_DISCOVERY);
        return;
    }
}