#ifndef IMU_H_
#define IMU_H_

#include "settings.h"

typedef struct {
    uint32_t id;            /* user defined ID                  */
    float acc[3];           /* acceleration                     */
    float gyr[3];           /* angular velocity                 */
    float mag[3];           /* magnetic field                   */
    float eul[3];           /* attitude: eular angle [r,p,y]    */
    float quat[4];          /* attitude: quaternion  [w,x,y,z]  */
    float pressure;         /* air pressure                         */
    uint32_t timestamp;
} imu_data_t;

typedef struct {
    imu_data_t imu;
    int64_t time_us;
    int64_t tsf_time_us;
    uint32_t seq;
    int32_t buffer_delay_us;
} imu_dgram_t;

typedef enum {
    IMU_STATUS_UNKNOWN = -1,
    IMU_STATUS_FAIL,
    IMU_STATUS_READY,
} imu_status_t;

typedef enum {
    IMU_MUX_IDLE,
    IMU_MUX_STREAM,
    IMU_MUX_DEBUG
} imu_mux_t;

/**
 * @brief IMU configuration
 */
typedef struct {
} imu_config_t;

/**
 * @brief IMU object
 */
typedef struct {
    bool initialized;
    uint8_t addr;
    imu_status_t status;
    bool enabled;
    imu_mux_t mux;
    SemaphoreHandle_t mutex;
} imu_t;

/**
 * @brief IMU interface
 */
typedef struct {
    imu_t *p_imu;

    esp_err_t (*init)(imu_t *p_imu, imu_config_t *p_config);

    esp_err_t (*read)(imu_t *p_imu, imu_dgram_t *out, bool crc_check);

    esp_err_t (*read_latest)(imu_t *p_imu, imu_dgram_t *out, bool crc_check);

    esp_err_t (*toggle)(imu_t *p_imu, bool enable);

    int (*is_powered_on)(imu_t *p_imu);

    esp_err_t (*self_test)(imu_t *p_imu);

    void (*chip_soft_reset)(imu_t *p_imu);

    void (*chip_hard_reset)(imu_t *p_imu);

    void (*buffer_reset)(imu_t *p_imu);

    int64_t (*get_buffer_delay)(imu_t *p_imu);

    size_t (*read_bytes)(imu_t *p_imu, uint8_t *out, size_t len);

    esp_err_t (*write_bytes)(imu_t *p_imu, void *in, size_t len);
} imu_interface_t;

extern imu_interface_t g_imu;

void imu_interface_init(imu_interface_t *p_interface, __attribute__((unused)) imu_config_t *p_config);


/** IMU model related **/
#if CONFIG_IMU_SENSOR_BNO08X

#include "bno08x.h"

#elif CONFIG_IMU_SENSOR_HI229

#include "hi229.h"

#elif CONFIG_IMU_SENSOR_GY95

#include "gy95.h"

#endif

// TODO: fix gy95 broken interface, make it align with hi229
#endif