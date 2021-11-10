#ifndef _GY95_H
#define _GY95_H

#include "esp_system.h"
#include "settings.h"

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
    bool flag;
    
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

void gy95_enable(gy95_t* p_gy);

void gy95_disable(gy95_t* p_gy);

#endif