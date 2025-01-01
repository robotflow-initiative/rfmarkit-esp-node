//
// Created by liyutong on 2024/5/9.
//
#ifndef _UDP_H
#define _UDP_H

#include "esp_system.h"
#include "esp_err.h"

#include <sys/cdefs.h>
#include <lwip/inet.h>
#include <lwip/sockets.h>

typedef struct {
    bool initialized;
    uint16_t source_port;
    uint16_t dest_port;
    struct sockaddr_in source_addr;
    struct sockaddr_in dest_addr;
    struct sockaddr_in reply_addr;
    int sock;
} udp_socket_t;

#define delay_ms(x) vTaskDelay((x) / portTICK_PERIOD_MS)

esp_err_t udp_socket_init(udp_socket_t* object, uint16_t source_port, const char* dest_addr, uint16_t dest_port);

esp_err_t udp_socket_free(udp_socket_t* object);

esp_err_t udp_socket_bind_local_port(udp_socket_t* object, uint16_t source_port);

esp_err_t udp_socket_set_destination(udp_socket_t* object, const char* ip_addr, uint16_t port);

esp_err_t udp_socket_set_broadcast(udp_socket_t* object, bool broadcast);

esp_err_t udp_socket_set_timeout(udp_socket_t* object, uint32_t timeout_s);

esp_err_t udp_socket_send(udp_socket_t* object, uint8_t * data_ptr, size_t size);

esp_err_t udp_socket_recv(udp_socket_t* object, uint8_t * data_ptr, size_t size, size_t * len);

#endif //#_UDP_H

