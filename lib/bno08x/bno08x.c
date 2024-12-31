#include <string.h>
#include <esp_mesh.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_log.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "nvs_flash.h"

#include "imu.h"
#include "bno08x.h"
#include "bno08x_driver.h"
#include "spatial.h"

#define bno08x_delay_ms(x) vTaskDelay((x) / portTICK_PERIOD_MS);

typedef struct {
    imu_t base;
    BNO08x driver;
} bno08x_t;

imu_interface_t g_imu = {0};

static bno08x_t s_bno08x = {0};

static const char *TAG = "imu[bno08x]";


static esp_err_t bno08x_config(imu_t *p_imu) {
    bno08x_t *p_bno08x = (bno08x_t *) p_imu;
    BNO08x *p_driver = &p_bno08x->driver;

    bool ret = BNO08x_initialize(p_driver);
    if (!ret) {
        ESP_LOGE(TAG, "failed to initialize BNO08x");
        p_bno08x->base.status = IMU_STATUS_FAIL;
        return ESP_FAIL;
    }

#if CONFIG_USE_LINEAR_ACCELERATION
    BNO08x_enable_linear_accelerometer(p_driver, 1000 / p_imu->target_fps); // 100Hz
#else
    BNO08x_enable_accelerometer(p_driver, 1000 / p_imu->target_fps); // 100Hz
#endif
    BNO08x_enable_rotation_vector(p_driver, 1000 / p_imu->target_fps); // 100Hz
    BNO08x_enable_step_counter(p_driver, CONFIG_BNO08X_SLOW_INTERVAL_MS); // 2Hz
    BNO08x_enable_stability_classifier(p_driver, CONFIG_BNO08X_SLOW_INTERVAL_MS); // 2Hz

    p_bno08x->base.enabled = true;

    return ESP_OK;
}

/**
 * @brief
 * @param p_imu
 * @param p_config
 * @return
**/
static esp_err_t bno08x_init(imu_t *p_imu, __attribute__((unused)) imu_config_t *p_config) {
    bno08x_t *p_bno08x = (bno08x_t *) p_imu;
    BNO08x *p_driver = &p_bno08x->driver;

    p_imu->addr = CONFIG_BNO08X_ADDR;
    p_imu->enabled = false;

    p_imu->mutex = xSemaphoreCreateMutex();
    if (p_imu->mutex == NULL) {
        ESP_LOGE(TAG, "failed to create mutex");
        p_imu->status = IMU_STATUS_FAIL;
    } else {
        p_imu->status = IMU_STATUS_READY;
    }

    BNO08x_config_t config = {
        .spi_peripheral = CONFIG_BNO08X_SPI_HOST,
        .io_mosi = CONFIG_BNO08X_MOSI_PIN,
        .io_miso = CONFIG_BNO08X_MISO_PIN,
        .io_sclk = CONFIG_BNO08X_SCLK_PIN,
        .io_cs = CONFIG_BNO08X_CS_PIN,
        .io_int = CONFIG_BNO08X_INT_PIN,
        .io_rst = CONFIG_BNO08X_RST_PIN,
        .io_wake = CONFIG_BNO08X_WAKE_PIN,
        .sclk_speed = CONFIG_BNO08X_SPI_SPEED,

    };
    BNO08x_init(p_driver, &config);
    p_bno08x->base.initialized = true;

    return bno08x_config(p_imu);
}

/**
 * @brief
 * @param p_imu
 * @param out
 * @param crc_check
 * @return
**/
esp_err_t bno08x_read(imu_t *p_imu, imu_dgram_t *out, __attribute__((unused)) bool crc_check) {
    BNO08x *p_driver = &((bno08x_t *) p_imu)->driver;

    float rad_acc;
    uint8_t acc;

    int64_t now = esp_timer_get_time();
    if (BNO08x_data_available(p_driver)) {
        BNO08x_get_quat(p_driver, &out->imu.quat[1], &out->imu.quat[2], &out->imu.quat[3], &out->imu.quat[0], &rad_acc, &acc);
        // spatial_quaternion_to_euler_deg((Quaternion *) &(out->imu.quat), (Euler *) &out->imu.eul);

#if CONFIG_USE_LINEAR_ACCELERATION
        BNO08x_get_linear_accel(p_driver, &out->imu.acc[0], &out->imu.acc[1], &out->imu.acc[2], &acc);
#else
        BNO08x_get_accel(p_driver, &out->imu.acc[0], &out->imu.acc[1], &out->imu.acc[2], &acc);
#endif

        // tsf timestamp
        out->tsf_ts_us = esp_mesh_get_tsf_time();

        out->buffer_delay_us = (int32_t)(esp_timer_get_time() - now);

        // unix timestamp
        struct timeval tv_now = { 0, 0 };
        gettimeofday(&tv_now, NULL);
        out->dev_ts_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
        return ESP_OK;
    }

    return ESP_FAIL;
}

/**
 * @brief
 * @param p_imu
 * @param enable
 * @return
**/
esp_err_t bno08x_toggle(imu_t *p_imu, bool enable) {
    BNO08x *p_driver = &((bno08x_t *) p_imu)->driver;

    if (enable == p_imu->enabled) {
        ESP_LOGW(TAG, "imu already %s", enable ? "enabled" : "disabled");
        return ESP_OK;
    }

    bool success = false;
    if (enable) {
        BNO08x_hard_reset(p_driver);
        success = bno08x_config(p_imu);
        p_imu->enabled = success ? true : p_imu->enabled;
    } else {
        success = BNO08x_mode_sleep(p_driver);
        p_imu->enabled = success ? false : p_imu->enabled;
    }

    return success ? ESP_OK : ESP_FAIL;
}

/**
 * @brief
 * @param p_imu
 * @return
**/
int bno08x_enabled(imu_t *p_imu) {
    return p_imu->enabled ? 1 : 0;
}

/**
 * @brief
 * @param p_imu
 * @return
**/
esp_err_t bno08x_self_test(imu_t *p_imu) {
    // Not implemented
    return ESP_OK;
}

/**
 * @brief
 * @param p_imu
**/
void bno08x_soft_reset(imu_t *p_imu) {
    BNO08x *p_driver = &((bno08x_t *) p_imu)->driver;
    BNO08x_soft_reset(p_driver);
}

/**
 * @brief
 * @param p_imu
**/
void bno08x_hard_reset(imu_t *p_imu) {
    BNO08x *p_driver = &((bno08x_t *) p_imu)->driver;
    BNO08x_hard_reset(p_driver);
}

/**
 * @brief
 * @param p_imu
**/
void bno08x_buffer_reset(imu_t *p_imu) {
    // Not implemented
}

/**
 * @brief
 * @param p_imu
 * @return
**/
int64_t bno08x_get_delay_us(imu_t *p_imu) {
    // Not implemented
    return 0;
}

/**
 * @brief
 * @param p_imu
 * @param out
 * @param len
 * @return
**/
size_t bno08x_read_bytes(imu_t *p_imu, uint8_t *out, size_t len) {
    BNO08x *p_driver = &((bno08x_t *) p_imu)->driver;

    float rad_acc;
    uint8_t acc;
    imu_dgram_t out_dgram;

    if (BNO08x_data_available(p_driver)) {
        BNO08x_get_quat(p_driver, &out_dgram.imu.quat[1], &out_dgram.imu.quat[2], &out_dgram.imu.quat[3], &out_dgram.imu.quat[0], &rad_acc, &acc);
        spatial_quaternion_to_euler_deg((Quaternion *) &(out_dgram.imu.quat), (Euler *) &out_dgram.imu.eul);
#if CONFIG_USE_LINEAR_ACCELERATION
        BNO08x_get_linear_accel(p_driver, &out_dgram.imu.acc[0], &out_dgram.imu.acc[1], &out_dgram.imu.acc[2], &acc);
#else
        BNO08x_get_accel(p_driver, &out_dgram.imu.acc[0], &out_dgram.imu.acc[1], &out_dgram.imu.acc[2], &acc);
#endif
    }

    snprintf(
        (char *) out, len,
        "[%+.2f %+.2f %+.2f %+.2f], [%+.2f %+.2f %+.2f]",
        out_dgram.imu.quat[0], out_dgram.imu.quat[1], out_dgram.imu.quat[2], out_dgram.imu.quat[3],
        out_dgram.imu.eul[0], out_dgram.imu.eul[1], out_dgram.imu.eul[2]
    );
    return strlen((char *) out);
}

/**
 * @brief
 * @param p_imu
 * @param in
 * @param len
 * @return
**/
esp_err_t bno08x_write_bytes(imu_t *p_imu, void *in, size_t len) {
    // Not implemented
    return ESP_OK;
}

/**
 *
 * @param p_interface
 * @param p_config
**/
void imu_interface_init(imu_interface_t *p_interface, imu_config_t *p_config) {
    p_interface->p_imu = (imu_t *) &s_bno08x;
    p_interface->p_imu->target_fps = p_config->target_fps;
    if (p_interface->p_imu->initialized) {
        ESP_LOGW(TAG, "IMU already initialized");
        return;
    } else {
        bno08x_init(p_interface->p_imu, NULL);
    }

    p_interface->init = bno08x_init;
    p_interface->read = bno08x_read;
    p_interface->toggle = bno08x_toggle;
    p_interface->enabled = bno08x_enabled;
    p_interface->self_test = bno08x_self_test;
    p_interface->soft_reset = bno08x_soft_reset;
    p_interface->hard_reset = bno08x_hard_reset;
    p_interface->buffer_reset = bno08x_buffer_reset;
    p_interface->get_delay_us = bno08x_get_delay_us;
    p_interface->read_bytes = bno08x_read_bytes;
    p_interface->write_bytes = bno08x_write_bytes;
}