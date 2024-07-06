#ifndef _BNO08X_H
#define _BNO08X_H

#include "driver/gpio.h"

#include "imu.h"
#include "modelspec.h"


#define CONFIG_BNO08X_ADDR              0x50
#define CONFIG_BNO08X_TRY_TIMES         5
#define CONFIG_BNO08X_INTERVAL_MS       10
#define CONFIG_BNO08X_SLOW_INTERVAL_MS  500

/** Modify this section to adapt different board **/
#define CONFIG_BNO08X_MOSI_PIN          CONFIG_IMU_MOSI_PIN
#define CONFIG_BNO08X_MISO_PIN          CONFIG_IMU_MISO_PIN
#define CONFIG_BNO08X_SCLK_PIN          CONFIG_IMU_SCLK_PIN
#define CONFIG_BNO08X_CS_PIN            CONFIG_IMU_CS_PIN
#define CONFIG_BNO08X_INT_PIN           CONFIG_IMU_INT_PIN
#define CONFIG_BNO08X_RST_PIN           CONFIG_IMU_RST_PIN
#define CONFIG_BNO08X_WAKE_PIN          CONFIG_IMU_WAKE_PIN

// Depending on the board (esp32s, esp32s3), the port is different
#if defined(CONFIG_IDF_TARGET_ESP32S)
#define CONFIG_BNO08X_SPI_HOST          VSPI_HOST
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#define CONFIG_BNO08X_SPI_HOST          SPI2_HOST
#endif

#define CONFIG_BNO08X_SPI_SPEED         3000000

typedef struct {
    imu_config_t base;

    /** pin configuration **/
    gpio_num_t io_mosi;
    gpio_num_t io_miso;
    gpio_num_t io_sclk;
    gpio_num_t io_cs;
    gpio_num_t io_int;
    gpio_num_t io_rst;
    gpio_num_t io_wake;
} bno08x_config_t;

/** Exposed API **/
#define imu_interface_init_external(imu)  \
    imu_interface_init((imu), NULL)


#endif