/*
 ******************************************************************************
 * @file    math_calib.h
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-19
 * @brief   This file provides a complete calibration routine for IMU,
 *          the calibration routine consist of bias, scale and misalignment
 *          calibration.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MATH_CALIB_H
#define __MATH_CALIB_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "math_misc.h"
#include "math_vector.h"
#include "math_matrix.h"
#include "drv_mpu6500.h"
#include "drv_usart.h"
#include "drv_timerdelay.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
	vect3f_t Ba;  // accel bias
	mat3f_t  Ma;  // accel coef matrix, include scale and misalignment
	vect3f_t Bg;  // gyro bias
	mat3f_t  Mg;  // gyro coef matrix, include scale and misalignment
	uint8_t  calibed; // calib status, 0 - not calib, 1 - calibed
}imu_calib_param_t;

typedef enum
{
	CUBE_X_POSITIVE_UP = 1,
	CUBE_X_NEGATIVE_UP = 2,
	CUBE_Y_POSITIVE_UP = 3,
	CUBE_Y_NEGATIVE_UP = 4,
	CUBE_Z_POSITIVE_UP = 5,
	CUBE_Z_NEGATIVE_UP = 6,
	CALC_CALIB_PARAM   = 7, // calculate calibration parameters
	CALIB_PARAM_COMP   = 8 // calibration parameters compensation
}cube_calib_cmd_t;

typedef struct
{
	/* cube direction - x positive */
	vect3f_t gyro_mean_x_positive;    // unit: deg/s
	vect3f_t accel_mean_x_positive;   // unit: g
	/* cube direction - x negative */
	vect3f_t gyro_mean_x_negative;
	vect3f_t accel_mean_x_negative;
	/* cube direction - y positive */
	vect3f_t gyro_mean_y_positive;
	vect3f_t accel_mean_y_positive;
	/* cube direction - y negative */
	vect3f_t gyro_mean_y_negative;
	vect3f_t accel_mean_y_negative;
	/* cube direction - z positive */
	vect3f_t gyro_mean_z_positive;
	vect3f_t accel_mean_z_positive;
	/* cube direction - z negative */
	vect3f_t gyro_mean_z_negative;
	vect3f_t accel_mean_z_negative;

	uint8_t collect_data_ready; // bit0-x positive data......bit5-z negative data
}cube_calib_collect_data_t;
/* Exported constants --------------------------------------------------------*/
#define CUBE_CALIB_SAMPLE_TIME_MS   (5000)
#define CUBE_CALIB_DELAY_MS         (5)
#define CUBE_CALIB_SAMPLE_COUNT     (CUBE_CALIB_SAMPLE_TIME_MS / CUBE_CALIB_DELAY_MS)

/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void cube_calib_init(void);
void cube_calib_state_machine(uint8_t calib_cmd);


#ifdef __cplusplus
}
#endif

#endif /* __MATH_CALIB_H */


/******************* (C) COPYRIGHT 2017 PENG ZHANG *****END OF FILE********/

