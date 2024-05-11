//
// Created by liyutong on 2024/4/28.
//
#include "esp_nimble_hci.h"
#include "esp_log.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"

#include "ble_srv.h"
#include "sys.h"

static int blehr_gap_event(struct ble_gap_event *event, void *arg);

static uint8_t blehr_addr_type;
static uint16_t conn_handle;

static const char *TAG = "ble.srv";


static void blehr_advertise(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    /**
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info)
     *     o Advertising tx power
     *     o Device name
     **/
    memset(&fields, 0, sizeof(fields));

    /**
     * Advertise two flags:
     *      o Discoverability in forthcoming advertisement (general)
     *      o BLE-only (BR/EDR unsupported)
     **/
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /**
     * Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     **/
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    fields.name = (uint8_t *) g_mcu.ble_local_name;
    fields.name_len = strlen(g_mcu.ble_local_name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "error setting advertisement data; rc=%d", rc);
        return;
    }

    /** Begin advertising **/
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(blehr_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, blehr_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "error enabling advertisement; rc=%d", rc);
        return;
    }
}

static int blehr_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            /** A new connection was established or a connection attempt failed **/
            ESP_LOGI(TAG,
                     "connection %s; status=%d",
                     event->connect.status == 0 ? "established" : "failed",
                     event->connect.status
            );

            if (event->connect.status != 0) {
                /** Connection failed; resume advertising **/
                blehr_advertise();
            }
            conn_handle = event->connect.conn_handle;
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "disconnect; reason=%d", event->disconnect.reason);

            /** Connection terminated; resume advertising **/
            blehr_advertise();
            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG, "adv complete\n");
            blehr_advertise();
            break;

        case BLE_GAP_EVENT_SUBSCRIBE:
            ESP_LOGI(TAG, "conn_handle from subscribe=%d", conn_handle);
            break;

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "mtu update event; conn_handle=%d mtu=%d", event->mtu.conn_handle, event->mtu.value);
            break;

    }

    return 0;
}

void blehr_on_sync(void) {
    int rc;

    rc = ble_hs_id_infer_auto(0, &blehr_addr_type);
    assert(rc == 0);

    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(blehr_addr_type, addr_val, NULL);

    ESP_LOGI(
        TAG,
        "device address: %02x:%02x:%02x:%02x:%02x:%02x",
        addr_val[5], addr_val[4], addr_val[3], addr_val[2], addr_val[1], addr_val[0]
    );

    /* Begin advertising */
    blehr_advertise();
}

static void blehr_on_reset(int reason) {
    ESP_LOGE(TAG, "resetting state; reason=%d", reason);
}

static void blehr_host_task(void *param) {
    ESP_LOGI(TAG, "BLE host task started");
    /** This function will return only when nimble_port_stop() is executed **/
    nimble_port_run();

    nimble_port_freertos_deinit();
}


void sys_start_ble_srv() {
    ESP_ERROR_CHECK(esp_nimble_hci_and_controller_init());

    nimble_port_init();
    ble_hs_cfg.sync_cb = blehr_on_sync;
    ble_hs_cfg.reset_cb = blehr_on_reset;

    /** Set the default device name **/
    ESP_ERROR_CHECK(gatt_svr_init());

    /** Set the default device name **/
    ble_svc_gap_device_name_set(g_mcu.ble_local_name);

    /** Start the task **/
    nimble_port_freertos_init(blehr_host_task);
}

void sys_stop_ble_srv() {
    int ret = nimble_port_stop();
    if (ret == 0) {
        ESP_LOGI(TAG, "BLE host stopped");
        ret = esp_nimble_hci_and_controller_deinit();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "esp_nimble_hci_and_controller_deinit() failed with error: %d", ret);
        }
    }
}