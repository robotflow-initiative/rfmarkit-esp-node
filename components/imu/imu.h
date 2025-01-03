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
    float pressure;         /* air pressure                     */
    uint32_t imu_ts_ms;     /* device timestamp in milliseconds */
} imu_data_t;

typedef struct {
    uint64_t step;          /* step counter                 */
    uint8_t stability;      /* stability indicator          */
    uint8_t activity;       /* activity indicator           */
    uint32_t timestamp;
} imu_data_extra_t; // TODO: implement extra information mechanism

typedef struct {
    imu_data_t imu;
    int64_t dev_ts_us;
    int64_t tsf_ts_us;
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
    int32_t target_fps;
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
    int32_t target_fps;
    uint64_t seq;   // sequence number
} imu_t;

typedef void (*imu_cb_fxn_t)(void *);  // void * is the imu_data_t

/**
 * @brief IMU interface
 */
typedef struct {
    imu_t *p_imu;

    esp_err_t (*init)(imu_t *p_imu, imu_config_t *p_config);

    esp_err_t (*read)(imu_t *p_imu, imu_dgram_t *out, bool crc_check);

    esp_err_t (*toggle)(imu_t *p_imu, bool enable);

    int (*enabled)(imu_t *p_imu);

    esp_err_t (*self_test)(imu_t *p_imu);

    void (*soft_reset)(imu_t *p_imu);

    void (*hard_reset)(imu_t *p_imu);

    void (*buffer_reset)(imu_t *p_imu);

    int64_t (*get_delay_us)(imu_t *p_imu);

    size_t (*read_bytes)(imu_t *p_imu, uint8_t *out, size_t len);

    esp_err_t (*write_bytes)(imu_t *p_imu, void *in, size_t len);
} imu_interface_t;

extern imu_interface_t g_imu;

void imu_interface_init(imu_interface_t *p_interface, imu_config_t *p_config);


/** IMU model related **/
#if CONFIG_IMU_SENSOR_BNO08X

#include "bno08x.h"

#elif CONFIG_IMU_SENSOR_HI229

#include "hi229.h"

#endif

// TODO: fix hi229 broken interface, make it align with bno08x
#endif