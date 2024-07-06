//
// Created by liyutong on 2024/5/9.
//
#include "esp_log.h"

#include "udp.h"

static const char *TAG = "udp";

/**
 * @brief  Initialize a UDP socket object
 * @param object
 * @param source_port
 * @param dest_addr
 * @param dest_port
 * @return
**/
esp_err_t udp_socket_init(udp_socket_t *object, uint16_t source_port, const char *dest_addr, uint16_t dest_port) {
    if (object == NULL) {
        ESP_LOGE(TAG, "object is NULL");
        return ESP_FAIL;
    }
    if (dest_port == 0 || dest_addr == NULL) {
        ESP_LOGE(TAG, "invalid destination address/port");
        return ESP_FAIL;
    }
    if (object->initialized) {
        ESP_LOGE(TAG, "object already initialized");
        return ESP_FAIL;
    }

    object->sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (object->sock < 0) {
        ESP_LOGE(TAG, "unable to create socket: errno %d", errno);
        return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "socket(%d) created, sending to %s:%d", object->sock, dest_addr, dest_port);
        object->initialized = true;
    }

    object->source_addr.sin_family = AF_INET;
    object->source_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (source_port != 0) {
        esp_err_t err = udp_socket_bind_local_port(object, source_port);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "socket bind failed: errno %d", errno);
            goto handler_err;
        }
    }

    udp_socket_set_destination(object, dest_addr, dest_port);

    if (strcmp(dest_addr, "255.255.255.255") == 0) {
        esp_err_t err = udp_socket_set_broadcast(object, true);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "set broadcast failed");
        }
    }
    return ESP_OK;

    handler_err:
    object->initialized = false;
    close(object->sock);
    object->sock = -1;
    return ESP_FAIL;
}

/**
 * @brief  Free a UDP socket object
 * @param object
 * @return
**/
esp_err_t udp_socket_free(udp_socket_t* object) {
    if (object == NULL) {
        ESP_LOGE(TAG, "object is NULL");
        return ESP_FAIL;
    }
    if (!object->initialized) {
        ESP_LOGE(TAG, "object not initialized");
        return ESP_FAIL;
    }

    close(object->sock);
    object->initialized = false;
    object->sock = -1;
    return ESP_OK;
}

/**
 * @brief  Bind a local port to a UDP socket object
 * @param object
 * @param source_port
 * @return
**/
esp_err_t udp_socket_bind_local_port(udp_socket_t *object, uint16_t source_port) {
    if (object == NULL) {
        ESP_LOGE(TAG, "object is NULL");
        return ESP_FAIL;
    }
    if (!object->initialized) {
        ESP_LOGE(TAG, "object not initialized");
        return ESP_FAIL;
    }

    object->source_port = source_port;
    object->source_addr.sin_port = htons(source_port);
    esp_err_t err = bind(object->sock, (struct sockaddr *) &object->source_addr, sizeof(struct sockaddr));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "socket bind failed: errno %d", errno);
        return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "bind local port to %d", source_port);
        return ESP_OK;
    }

}

/**
 * @brief  Set the destination address and port of a UDP socket object
 * @param object
 * @param ip_addr
 * @param port
 * @return
**/
esp_err_t udp_socket_set_destination(udp_socket_t *object, const char *ip_addr, uint16_t port) {
    if (object == NULL) {
        ESP_LOGE(TAG, "object is NULL");
        return ESP_FAIL;
    }
    if (!object->initialized) {
        ESP_LOGE(TAG, "object not initialized");
        return ESP_FAIL;
    }

    object->dest_addr.sin_family = AF_INET;
    object->dest_addr.sin_addr.s_addr = inet_addr(ip_addr);
    object->dest_port = port;
    object->dest_addr.sin_port = htons(port);

    return ESP_OK;

}

/**
 * @brief  Set the broadcast flag of a UDP socket object
 * @param object
 * @param broadcast
 * @return
**/
esp_err_t udp_socket_set_broadcast(udp_socket_t *object, bool broadcast) {
    if (object == NULL) {
        ESP_LOGE(TAG, "object is NULL");
        return ESP_FAIL;
    }
    if (!object->initialized) {
        ESP_LOGE(TAG, "object not initialized");
        return ESP_FAIL;
    }

    int flag = broadcast ? 1 : 0;
    int ret = setsockopt(object->sock, SOL_SOCKET, SO_BROADCAST, &flag, sizeof(flag));

    if (ret < 0) {
        ESP_LOGE(TAG, "setsockopt failed: errno %d", errno);
        return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "set broadcast to %d", flag);
        return ESP_OK;
    }
}

/**
 * @brief  Set the timeout of a UDP socket object
 * @param object
 * @param timeout_s
 * @return
**/
esp_err_t udp_socket_set_timeout(udp_socket_t *object, uint32_t timeout_s) {
    if (object == NULL) {
        ESP_LOGE(TAG, "object is NULL");
        return ESP_FAIL;
    }
    if (!object->initialized) {
        ESP_LOGE(TAG, "object not initialized");
        return ESP_FAIL;
    }

    struct timeval timeout = {(long)timeout_s, 0};
    int ret = setsockopt(object->sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    if (ret < 0) {
        ESP_LOGE(TAG, "setsockopt failed: errno %d", errno);
        return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "set timeout to %d seconds", timeout_s);
        return ESP_OK;
    }
}

/**
 * @brief  Send data to a UDP socket object
 * @param object
 * @param data_ptr
 * @param size
 * @return
**/
esp_err_t udp_socket_send(udp_socket_t *object, uint8_t *data_ptr, size_t size) {
    if (object == NULL) {
        ESP_LOGE(TAG, "object is NULL");
        return ESP_FAIL;
    }
    if (!object->initialized) {
        ESP_LOGE(TAG, "object not initialized");
        return ESP_FAIL;
    }

    ssize_t ret = sendto(object->sock, data_ptr, size, 0, (struct sockaddr *) &object->dest_addr, sizeof(struct sockaddr));
    if (ret < 0) {
        ESP_LOGE(TAG, "sendto failed: errno %d", errno);
        return ESP_ERR_NO_MEM;
    } else {
        ESP_LOGD(TAG, "sent %d bytes", ret);
        return ESP_OK;
    }
}

/**
 * @brief  Receive data from a UDP socket object
 * @param object
 * @param data_ptr
 * @param size
 * @param len
 * @return
**/
esp_err_t udp_socket_recv(udp_socket_t* object, uint8_t * data_ptr, size_t size, size_t * len) {
    if (object == NULL) {
        ESP_LOGE(TAG, "object is NULL");
        return ESP_FAIL;
    }
    if (!object->initialized) {
        ESP_LOGE(TAG, "object not initialized");
        return ESP_FAIL;
    }

    socklen_t socklen = sizeof(object->reply_addr);
    ssize_t recv_len = recvfrom(object->sock, data_ptr, size, 0, (struct sockaddr *) &object->reply_addr, &socklen);
    *len = recv_len;
    if (recv_len < 0) {
        ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
        return ESP_FAIL;
    } else {
        ESP_LOGD(TAG, "received %d bytes", recv_len);
        return ESP_OK;
    }

}