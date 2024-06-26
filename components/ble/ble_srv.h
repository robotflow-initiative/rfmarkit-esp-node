//
// Created by liyutong on 2024/4/28.
//

#ifndef BLE_SRV_
#define BLE_SRV_

/* BLE */
#include "nimble/ble.h"
#include "modlog/modlog.h"

#define GATT_WIFI_CONFIGURATION_UUID            0x1829
#define GATT_WIFI_WRITE_UUID                    0x2B1F
#define GATT_WIFI_READ_UUID                     0x2B20

struct ble_hs_cfg;
struct ble_gatt_register_ctxt;

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);

int gatt_svr_init(void);

void sys_start_ble_srv(void);

void sys_stop_ble_srv(void);

#endif
