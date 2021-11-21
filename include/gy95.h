#ifndef _GY95_H
#define _GY95_H

#include "esp_system.h"
#include "freertos/semphr.h"
#include "settings.h"

typedef enum gy95_status_t {
    GY95_OK,
    GY95_RECV_COMPLETE,
    GY95_FAIL,
    GY95_READY,
} gy95_status_t;

typedef struct gy95_t {
    int port;

    int ctrl_pin;
    int rx_pin;
    int tx_pin;
    int rts_pin;
    int cts_pin;

    uint8_t addr;
    int cursor;
    int start_reg;
    int length;
    gy95_status_t status;

    uint8_t acc_scale;
    uint8_t gyro_scale;
    uint8_t mag_scale;
    uint8_t scale;

    SemaphoreHandle_t mux;

    uint8_t buf[GY95_PAYLOAD_LEN];

} gy95_t;

void gy95_msp_init(gy95_t* p_gy);

void gy95_init(gy95_t* p_gy,
               int port,
               int ctrl_pin,
               int rx_pin,
               int tx_pin,
               int rts_pin,
               int cts_pin,
               int addr
);

esp_err_t gy95_send(gy95_t* p_gy, uint8_t ctrl_msg[4], uint8_t* echo);

esp_err_t gy95_setup(gy95_t* p_gy);

esp_err_t gy95_cali_acc(gy95_t* p_gy);

void gy95_cali_mag(gy95_t* p_gy);

esp_err_t gy95_cali_reset(gy95_t* p_gy);

void gy95_clean(gy95_t* p_gy);

bool gy95_chksum(gy95_t* p_gy);

void gy95_read(gy95_t* p_gy);

void gy95_safe_read(gy95_t* p_gy);

size_t gy95_get_buffer_len(gy95_t* p_gy);

void gy95_enable(gy95_t* p_gy);

void gy95_disable(gy95_t* p_gy);

#endif