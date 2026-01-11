#pragma once 

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== DRIVER ERROR CODES ================= */
typedef enum {
    IMU_OK = 0,
    IMU_ERR_INVALID_ARG,
    IMU_ERR_NOT_INITIALIZED,
    IMU_ERR_HW_NOT_FOUND,
    IMU_ERR_I2C,
    IMU_ERR_INTERNAL
} imu_result_t;

/* ===================== IMU SAMPLE STRUCT ============== */

/* This will represent one IMU sample. FOr now these stubs will be filled by simulated values
   once the sensor arrives the real driver will fill these by reading register */
typedef struct {
    int64_t time_us; /* timestamp in microseconds since boot */
    float acc_x; /* acceleration in x direction */
    float acc_y; /* in y direction */
    float acc_z; 

    float gyro_x; /* gyro in x direction (deg/s) */
    float gyro_y; 
    float gyro_z;
} imu_sample_t; 

/* ==================== CONFIG STRUCT ============ */
typedef struct {
    uint8_t i2c_addr;
    uint32_t i2c_clock_hz; 
} imu_config_t; 

/* =================== API ================== */
imu_result_t imu_init(const imu_config_t* config); 
imu_result_t imu_read(imu_sample_t* out); 
const char* imu_error_to_string(imu_result_t r); 

#ifdef __cplusplus
}
#endif