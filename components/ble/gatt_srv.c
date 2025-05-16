//
// Created by liyutong on 2024/4/28.
//
/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <assert.h>
#include <esp_netif.h>
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "ble_srv.h"
#include "sys.h"
#include "imu.h"

static const char *TAG = "ble.gatt_srv    ";

static int gatt_svr_chr_wifi_info(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg
) {
    uint16_t uuid;

    uuid = ble_uuid_u16(ctxt->chr->uuid);
    ESP_LOGI(TAG, "service accessed with uuid: %x", uuid);
    if (uuid == GATT_WIFI_WRITE_UUID) {
        ESP_LOGI(TAG, "write received, data: %.*s", ctxt->om->om_len, ctxt->om->om_data);
        char ssid[CONFIG_VAR_STR_MAX_LEN];
        char psk[CONFIG_VAR_STR_MAX_LEN];

        /** Decode the data "<SSID> <PASSWORD>" **/
        ctxt->om->om_data[ctxt->om->om_len] = 0;
        char *token = strtok((char *) ctxt->om->om_data, " ");
        if (token != NULL) {
            strcpy(ssid, token);
            token = strtok(NULL, "\n");
            if (token != NULL) {
                strcpy(psk, token);
            } else {
                ESP_LOGE(TAG, "no psk provided.");
                return BLE_ATT_ERR_UNLIKELY;
            }
        } else {
            ESP_LOGE(TAG, "no SSID provided.");
            return BLE_ATT_ERR_UNLIKELY;
        }

        /** Apply the decoded information **/
        memset(g_mcu.wifi_ssid, 0, sizeof(g_mcu.wifi_ssid));
        memset(g_mcu.wifi_psk, 0, sizeof(g_mcu.wifi_psk));
        strcpy(g_mcu.wifi_ssid, ssid);
        strcpy(g_mcu.wifi_psk, psk);

        /** Save config **/
        mcu_var_t *p_var;
        p_var = sys_find_var(CONFIG_NVS_WIFI_SSID_NAME, strlen(CONFIG_NVS_WIFI_SSID_NAME));
        sys_set_nvs_var(p_var, ssid);
        p_var = sys_find_var(CONFIG_NVS_WIFI_PSK_NAME, strlen(CONFIG_NVS_WIFI_PSK_NAME));
        sys_set_nvs_var(p_var, psk);

        set_sys_event(EV_SYS_WIFI_CONFIG_UPDATED);
        return ESP_OK;
    } else if (uuid == GATT_WIFI_READ_UUID) {
        bool wifi_connected = g_mcu.state.wifi_connected;
        int rc;
        if (wifi_connected) {
            /** Return the device IP **/
            char ip_str[16];
            sprintf(ip_str, IPSTR, IP2STR(&(g_mcu.ip_info.ip)));
            rc = os_mbuf_append(ctxt->om, ip_str, strlen(ip_str));
        } else {
            /** 0.0.0.0 for unconnected **/
            rc = os_mbuf_append(ctxt->om, "0.0.0.0", 7);
        }
        return rc == 0 ? 0 : BLE_ATT_ERR_UNLIKELY;
    } else {
        return BLE_ATT_ERR_UNLIKELY;
    }
}

typedef struct {
    float acc[3];           /* acceleration                     */
    float gyr[3];           /* angular velocity                 */
    float quat[4];          /* attitude: quaternion  [w,x,y,z]  */
    uint32_t imu_ts_ms;     /* device timestamp in milliseconds */
} imu_ble_dgram_t;

static int gatt_svr_chr_imu(
        uint16_t conn_handle,
        uint16_t attr_handle,
        struct ble_gatt_access_ctxt *ctxt,
        void *arg
) {
    int rc;
    imu_dgram_t imu_data;
    rc = xQueueReceive(g_mcu.imu_queue, &imu_data, 0);
    if (rc != pdTRUE) {
        ESP_LOGD(TAG, "imu data not available");
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    /** Copy the data **/
    imu_ble_dgram_t imu_ble_data;
    memcpy(imu_ble_data.acc, imu_data.imu.acc, sizeof(imu_ble_data.acc));
    memcpy(imu_ble_data.gyr, imu_data.imu.gyr, sizeof(imu_ble_data.gyr));
    memcpy(imu_ble_data.quat, imu_data.imu.quat, sizeof(imu_ble_data.quat));
    imu_ble_data.imu_ts_ms = imu_data.imu.imu_ts_ms;

    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            // read op indicates that the client is reading imu
            if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
                ESP_LOGD(TAG, "characteristic read; conn_handle=%d attr_handle=%d\n",
                         conn_handle, attr_handle);
            } else {
                ESP_LOGD(TAG, "characteristic read by NimBLE stack; attr_handle=%d\n",
                         attr_handle);
            }
            if (attr_handle == g_mcu.arhs_val_handle) {
                rc = os_mbuf_append(ctxt->om,
                                    &imu_ble_data,
                                    sizeof(imu_ble_data));
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            goto unknown;
    }
    unknown:
    /**
     * unknown characteristic/descriptor;
     * The NimBLE host should not have called this function;
    **/
    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /** Service: Device Information **/
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(GATT_WIFI_CONFIGURATION_UUID),
        .characteristics = (struct ble_gatt_chr_def[])
            {{
                 /* Characteristic: * Manufacturer name */
                 .uuid = BLE_UUID16_DECLARE(GATT_WIFI_WRITE_UUID),
                 .access_cb = gatt_svr_chr_wifi_info,
                 .flags = BLE_GATT_CHR_F_WRITE,
             },
             {
                 /* Characteristic: * Manufacturer name */
                 .uuid = BLE_UUID16_DECLARE(GATT_WIFI_READ_UUID),
                 .access_cb = gatt_svr_chr_wifi_info,
                 .flags = BLE_GATT_CHR_F_READ,
             },
             {
                .uuid = BLE_UUID16_DECLARE(GATT_IMU_NOTIFY_UUID),
                .access_cb = gatt_svr_chr_imu,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &g_mcu.arhs_val_handle,
             },
             {
                 0, /** No more characteristics in this service **/
             },
            }
    },

    {
        0, /** No more services **/
    },
};


void gatt_svr_subscribe_cb(struct ble_gap_event *event) {
    /** check attribute handle **/
    if (event->subscribe.attr_handle == g_mcu.arhs_val_handle) {
        if (event->subscribe.cur_notify) {
            ESP_LOGI(TAG, "subscribe to arhs");
            /** update arhs data subscription status **/
            g_mcu.arhs_conn_handle = event->subscribe.conn_handle;
            g_mcu.arhs_subscribed = true;
        } else {
            ESP_LOGI(TAG, "unsubscribe from arhs");
            /** update arhs data subscription status **/
            g_mcu.arhs_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            g_mcu.arhs_subscribed = false;
        }
    }
}

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg) {
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
        case BLE_GATT_REGISTER_OP_SVC:
            ESP_LOGD(TAG, "registered service %s with handle=%d\n",
                     ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                     ctxt->svc.handle);
            break;

        case BLE_GATT_REGISTER_OP_CHR:
            ESP_LOGD(TAG, "registering characteristic %s with "
                          "def_handle=%d val_handle=%d\n",
                     ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                     ctxt->chr.def_handle,
                     ctxt->chr.val_handle);
            break;

        case BLE_GATT_REGISTER_OP_DSC:
            ESP_LOGD(TAG, "registering descriptor %s with handle=%d\n",
                     ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                     ctxt->dsc.handle);
            break;

        default:
            assert(0);
            break;
    }
}

esp_err_t gatt_svr_init(void) {
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return ESP_FAIL;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return ESP_FAIL;
    }

    return ESP_OK;
}