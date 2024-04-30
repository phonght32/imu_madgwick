#ifndef _IMU_MADGWICK_H_
#define _IMU_MADGWICK_H_

#ifdef __cplusplus 
extern "C" {
#endif

#include "stdint.h"
#include "math.h"

#include "err_code.h"

typedef struct imu_madgwick *imu_madgwick_handle_t;

/**
 * @brief   Configuration structure.
 */
typedef struct {
    float beta;
    float sample_freq;
} imu_madgwick_cfg_t;

/*
 * @brief   Configure Madgwick AHRS parameters.
 *
 * @param   config Struct pointer.
 *
 * @return
 *      - Madgwick handle structure: Success.
 *      - 0:                         Fail.
 */
imu_madgwick_handle_t imu_madgwick_init();

/*
 * @brief   Set configuration parameters.
 *
 * @param 	handle Handle structure.
 * @param   config Configuration structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t imu_madgwick_set_config(imu_madgwick_handle_t handle, imu_madgwick_cfg_t config);

/*
 * @brief   Configuration parameters to run.
 *
 * @param 	handle Handle structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t imu_madgwick_config(imu_madgwick_handle_t handle);

/*
 * @brief   Set beta value.
 *
 * @param   handle Handle structure.
 * @param   beta Beta.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t imu_madgwick_set_beta(imu_madgwick_handle_t handle, float beta);

/*
 * @brief   Set sample frequency value.
 *
 * @param   handle Handle structure.
 * @param   sample_freq Sample frequency.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t imu_madgwick_set_sample_frequency(imu_madgwick_handle_t handle, float sample_freq);

/*
 * @brief   Get quaternion.
 *
 * @param   handle Handle structure.
 * @param   q0, q1, q2 q3 Quaternion data.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t imu_madgwick_get_quaternion(imu_madgwick_handle_t handle, float *q0, float *q1, float *q2, float *q3);

/*
 * @brief   Update Madgwick AHRS quaternion with 6 motions.
 *
 * @param   handle Handle structure.
 * @param   gx Gyroscope along x axis.
 * @param   gy Gyroscope along y axis.
 * @param   gz Gyroscope along z axis.
 * @param   ax Accelerometer along x axis.
 * @param   ay Accelerometer along y axis.
 * @param   az Accelerometer along z axis.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t imu_madgwick_update_6dof(imu_madgwick_handle_t handle, float gx, float gy, float gz, float ax, float ay, float az);

/*
 * @brief   Update Madgwick AHRS quaternion with 9 motions.
 *
 * @param   handle Handle structure.
 * @param   gx Gyroscope along x axis.
 * @param   gy Gyroscope along y axis.
 * @param   gz Gyroscope along z axis.
 * @param   ax Accelerometer along x axis.
 * @param   ay Accelerometer along y axis.
 * @param   az Accelerometer along z axis.
 * @param   mx Magnetometer along x axis.
 * @param   my Magnetometer along y axis.
 * @param   mz Magnetometer along z axis.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t imu_madgwick_update_9dof(imu_madgwick_handle_t handle, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);


#ifdef __cplusplus 
}
#endif

#endif /* _IMU_MADGWICK_H_ */