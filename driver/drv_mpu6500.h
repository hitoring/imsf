/*
 ******************************************************************************
 * @file    drv_mpu6500.h
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-08
 * @brief   This file provides a driver for IMU MPU6500.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRV_MPU6500_H
#define __DRV_MPU6500_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "math_misc.h"
#include "math_vector.h"
#include "drv_timerdelay.h"
#include "drv_spi.h"
#include "drv_usart.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float accel_x;
	float accel_y;
	float accel_z;
	float temp;
}imu_raw_t;

/* Exported constants --------------------------------------------------------*/

/*
 * @brief MPU6500 Registers Address 
 */
/* Gyroscope Self-Test Registers Address */
#define MPU6500_SELF_TEST_X_GYRO_ADDR		0X00	// gyro-x self-test register
#define MPU6500_SELF_TEST_Y_GYRO_ADDR		0X01	// gyro-y self-test register
#define MPU6500_SELF_TEST_Z_GYRO_ADDR		0X02	// gyro-z self-test register
/* Accelerometer Self-Test Registers Address */
#define MPU6500_SELF_TEST_X_ACCEL_ADDR		0X0D	// accel-x self-test register
#define MPU6500_SELF_TEST_Y_ACCEL_ADDR		0X0E	// accel-y self-test register
#define MPU6500_SELF_TEST_Z_ACCEL_ADDR		0X0F	// accel-z self-test register
/* Gyroscope Offset Adjustment Register Address */
#define MPU6500_XG_OFFSET_H_ADDR			0X13    // gyro-x offset register
#define MPU6500_XG_OFFSET_L_ADDR			0X14
#define MPU6500_YG_OFFSET_H_ADDR			0X15    // gyro-y offset register
#define MPU6500_YG_OFFSET_L_ADDR			0X16
#define MPU6500_ZG_OFFSET_H_ADDR			0X17    // gyro-z offset register
#define MPU6500_ZG_OFFSET_L_ADDR			0X18
/* Sample Rate Divider Register Address */
#define MPU6500_SMPLRT_DIV_ADDR				0X19	// sample rate divider register
/* Config Registers Address */
#define MPU6500_CONFIG_ADDR					0X1A	// config register
#define MPU6500_GYRO_CONFIG_ADDR		    0X1B	// gyroscope config register
#define MPU6500_ACCEL_CONFIG_ADDR			0X1C	// accelerometer config register
#define MPU6500_ACCEL_CONFIG2_ADDR			0X1D	// accelerometer config register 2
#define MPU6500_LP_ACCEL_ODR_ADDR           0X1E    // low power accelerometer ODR control register
#define MPU6500_WOM_THR_ADDR		        0X1F	// wake-on motion threshold register
/* FIFO Enable Registers Address */
#define MPU6500_FIFO_EN_ADDR			    0X23	// fifo enable register
/* Interrupt Enable Address */
#define MPU6500_INT_PIN_CFG_ADDR		    0X37	// interrupt pin/bypass enable config register
#define MPU6500_INT_ENABLE_ADDR				0X38	// interrupt enable register
#define MPU6500_INT_STATUS_ADDR				0X3A	// interrupt status register
/* Accelerometer Output Registers Address */
#define MPU6500_ACCEL_XOUT_H_ADDR			0X3B	// accel-x measurement output register
#define MPU6500_ACCEL_XOUT_L_ADDR			0X3C	
#define MPU6500_ACCEL_YOUT_H_ADDR			0X3D	// accel-y measurement output register
#define MPU6500_ACCEL_YOUT_L_ADDR			0X3E	
#define MPU6500_ACCEL_ZOUT_H_ADDR			0X3F	// accel-z measurement output register
#define MPU6500_ACCEL_ZOUT_L_ADDR			0X40	
/* Temperature Output Registers Address */
#define MPU6500_TEMP_OUT_H_ADDR		    	0X41	// temperature measurement output register
#define MPU6500_TEMP_OUT_L_ADDR		    	0X42	
/* Gyroscope Output Registers Address */
#define MPU6500_GYRO_XOUT_H_ADDR		    0X43	// gyro-x measurement output register
#define MPU6500_GYRO_XOUT_L_ADDR		    0X44	
#define MPU6500_GYRO_YOUT_H_ADDR		    0X45	// gyro-y measurement output register
#define MPU6500_GYRO_YOUT_L_ADDR		    0X46	
#define MPU6500_GYRO_ZOUT_H_ADDR		    0X47	// gyro-z measurement output register
#define MPU6500_GYRO_ZOUT_L_ADDR		    0X48	
/* Signal Path Reset & Power Management Registers Address */
#define MPU6500_SIGNAL_PATH_RESET_ADDR		0X68	// signal path reset register
#define MPU6500_ACC_INTEL_CTRL_ADDR	    	0X69	// accel interrupt control register
#define MPU6500_USER_CTRL_ADDR		    	0X6A	// user control register
#define MPU6500_PWR_MGMT_1_ADDR		    	0X6B	// power management 1 register, particularly reset value is 0x01
#define MPU6500_PWR_MGMT_2_ADDR		    	0X6C	// power management 2 register
#define MPU6500_FIFO_COUNTH_ADDR		    0X72	// fifo count register register
#define MPU6500_FIFO_COUNTL_ADDR		    0X73	
#define MPU6500_FIFO_R_W_ADDR				0X74	// fifo read write register
#define MPU6500_WHO_AM_I_ADDR		    	0X75	// who-am-i register, particularly reset value is 0x70
/* Accelerometer Offset Registers Address */
#define MPU6500_XA_OFFSET_H_ADDR		    0X77    // accel-x offset register
#define MPU6500_XA_OFFSET_L_ADDR		    0X78
#define MPU6500_YA_OFFSET_H_ADDR		    0X7A    // accel-y offset register
#define MPU6500_YA_OFFSET_L_ADDR		    0X7B
#define MPU6500_ZA_OFFSET_H_ADDR		    0X7D    // accel-z offset register
#define MPU6500_ZA_OFFSET_L_ADDR		    0X7E

/* The Default Value of WHO_AM_I Register Address */
#define MPU6500_WHO_AM_I				    0X70    // default value of WHO_AM_I resgister, used to verify the identity of the device



/*
 * @brief MPU6500 Registers Description 
 */
 
/* 
 * @brief    Bits definition of CONFIG register
 * @regdef   The DLPF is configured by DLPF_CFG, when FCHOICE_B [1:0] = 2b'00.
 *           The gyroscope and temperature sensor are filtered according to the value of
 *           DLPF_CFG as shown in the table below.
 * @regaddr  CONFIG @ 0X1A
 * @table
 *-----------------------------------------------------------------------
 *DLPF_CFG|               Gyroscope             |      Temperature      |
 *        |Bandwidth(Hz)|   Delay(ms)| Rate(kHz)|Bandwidth(Hz)|Delay(ms)|
 *-----------------------------------------------------------------------
 *   0    |      250    |   0.97     |     8    |    4000     |   0.04  |
 *-----------------------------------------------------------------------
 *   1    |      184    |   2.9      |     1    |    188      |   1.9   |
 *-----------------------------------------------------------------------
 *   2    |      92     |   3.9      |     1    |    98       |   2.8   |
 *-----------------------------------------------------------------------
 *   3    |      41     |   5.9      |     1    |    42       |   4.8   |
 *-----------------------------------------------------------------------
 *   4    |      20     |   9.9      |     1    |    20       |   8.3   |
 *-----------------------------------------------------------------------
 *   5    |      10     |   17.85    |     1    |    10       |   13.4  |
 *-----------------------------------------------------------------------
 *   6    |      5      |   33.48    |     1    |    5        |   18.6  |
 *-----------------------------------------------------------------------
 *   7    |      3600   |   0.17     |     8    |    4000     |   0.04  |
 *-----------------------------------------------------------------------
 */
/* DLPF_CFG*/
#define MPU6500_DLPF_GYRO_BW_250HZ_RATE_8KHZ                ((uint8_t)0)
#define MPU6500_DLPF_GYRO_BW_184HZ_RATE_1KHZ                ((uint8_t)1)
#define	MPU6500_DLPF_GYRO_BW_92HZ_RATE_1KHZ                 ((uint8_t)2)
#define	MPU6500_DLPF_GYRO_BW_41HZ_RATE_1KHZ                 ((uint8_t)3)
#define	MPU6500_DLPF_GYRO_BW_20HZ_RATE_1KHZ                 ((uint8_t)4)
#define	MPU6500_DLPF_GYRO_BW_10HZ_RATE_1KHZ                 ((uint8_t)5)
#define	MPU6500_DLPF_GYRO_BW_5HZ_RATE_1KHZ                  ((uint8_t)6)
#define	MPU6500_DLPF_GYRO_BW_3600HZ_RATE_8KHZ               ((uint8_t)7)

/*
 * @brief    Bits definition of GYRO_CONFIG register
 * @regdef   FS_SEL selects the full scale range of the gyroscope outputs.
 * @regaddr  GYRO_CONFIG @ 0X1B
 */
/* GYRO_FS_SEL */
#define	MPU6500_GYRO_FSR_250DPS                             ((uint8_t)(0 << 3))
#define	MPU6500_GYRO_FSR_500DPS                             ((uint8_t)(1 << 3))
#define	MPU6500_GYRO_FSR_1000DPS                            ((uint8_t)(2 << 3))
#define	MPU6500_GYRO_FSR_2000DPS                            ((uint8_t)(3 << 3))

/*
 * @brief    Bits definition of ACCEL_CONFIG register
 * @regdef   AFS_SEL selects the full scale range of the accelerometer outputs.
 * @regaddr  ACCEL_CONFIG @ 0X1C
 */
/* ACCEL_FS_SEL */
#define	MPU6500_ACCEL_FSR_2G                                ((uint8_t)(0 << 3))
#define	MPU6500_ACCEL_FSR_4G                                ((uint8_t)(1 << 3))
#define	MPU6500_ACCEL_FSR_8G                                ((uint8_t)(2 << 3))
#define	MPU6500_ACCEL_FSR_16G                               ((uint8_t)(3 << 3))

/* 
 * @brief    Bits definition of ACCEL_CONFIG2 register
 * @regdef   The A_DLPF is configured by A_DLPF_CFG, when ACCEL_FCHOICE_B = 0.
 *           The accelerometer sensor are filtered according to the value of
 *           A_DLPF_CFG as shown in the table below.
 * @regaddr  CONFIG @ 0X1D
 * @table
 *------------------------------------------------------------------------
 *A_DLPF_CFG|                       Gyroscope  Output                    |     
 *          |Bandwidth(Hz)|   Delay(ms)| Rate(kHz)|Noise Density(ug/rtHz)|
 *-----------------------------------------------------------------------
 *   0      |      460    |   1.94     |     1    |         220          |
 *-----------------------------------------------------------------------
 *   1      |      184    |   5.8      |     1    |         220          |
 *-----------------------------------------------------------------------
 *   2      |      92     |   7.8      |     1    |         220          |
 *-----------------------------------------------------------------------
 *   3      |      41     |   11.8     |     1    |         220          |
 *-----------------------------------------------------------------------
 *   4      |      20     |   19.8     |     1    |         220          |
 *-----------------------------------------------------------------------
 *   5      |      10     |   35.7     |     1    |         220          |
 *-----------------------------------------------------------------------
 *   6      |      5      |   66.96    |     1    |         220          |
 *-----------------------------------------------------------------------
 *   7      |      460   |    1.94     |     1    |         220          |
 *------------------------------------------------------------------------
 */
/* A_DLPF_CFG */
#define MPU6500_ADLPF_ACCEL_BW_460HZ_RATE_1KHZ               ((uint8_t)0)
#define MPU6500_ADLPF_ACCEL_BW_184HZ_RATE_1KHZ               ((uint8_t)1)
#define	MPU6500_ADLPF_ACCEL_BW_92HZ_RATE_1KHZ                ((uint8_t)2)
#define	MPU6500_ADLPF_ACCEL_BW_41HZ_RATE_1KHZ                ((uint8_t)3)
#define	MPU6500_ADLPF_ACCEL_BW_20HZ_RATE_1KHZ                ((uint8_t)4)
#define	MPU6500_ADLPF_ACCEL_BW_10HZ_RATE_1KHZ                ((uint8_t)5)
#define	MPU6500_ADLPF_ACCEL_BW_5HZ_RATE_1KHZ                 ((uint8_t)6)

/* 
 * @brief    Bits definition of SIGNAL_PATH_RESET register
 * @regdef   Reset gyro, accel and temp digital signal path
 * @regaddr  SIGNAL_PATH_RESET @ 0X68
 */
/* GYRO_RST */
#define GYRO_RST_EN                                          ((uint8_t)(1 << 2))
#define GYRO_RST_DIS                                         ((uint8_t)(0 << 2))
/* ACCEL_RST */
#define ACCEL_RST_EN                                         ((uint8_t)(1 << 1))
#define ACCEL_RST_DIS                                        ((uint8_t)(0 << 1))
/* TEMP_RST */
#define TEMP_RST_EN                                          ((uint8_t)(1 << 0))
#define TEMP_RST_DIS                                         ((uint8_t)(0 << 0))

/*
 * @brief    Bits definition of register PWR_MGMT_1
 * @register PWR_MGMT_1 @ 0X6B
 * @regdef   this register allows the user to configure the power mode and clock source.
 */
/* DEVICE_RESET */
#define DEVICE_RESET_EN                                      ((uint8_t)(1 << 7))
#define	DEVICE_RESET_DIS                                     ((uint8_t)(0 << 7))
/* SLEEP */
#define DEVICE_SLEEP_EN                                      ((uint8_t)(1 << 6))
#define DEVICE_SLEEP_DIS                                     ((uint8_t)(0 << 6))
/* CYCLE */
#define DEVICE_CYCLE_EN                                      ((uint8_t)(1 << 5))
#define DEVICE_CYCLE_DIS                                     ((uint8_t)(0 << 5))
/* GYRO_STANDBY */
#define DEVICE_GYRO_STANDBY_EN                               ((uint8_t)(1 << 4))
#define DEVICE_GYRO_STANDBY_DIS                              ((uint8_t)(0 << 4))
/* TEMP_DIS */
#define DEVICE_TEMP_DIS                                      ((uint8_t)(1 << 3))
#define DEVICE_TEMP_EN                                       ((uint8_t)(0 << 3))
/* CLKSEL */
#define MPU6500_CLKSRC_INTER_OSC                             ((uint8_t)(0 << 0))
#define MPU6500_CLKSRC_AUTO_SELECT                           ((uint8_t)(1 << 0))
#define MPU6500_CLOCK_STOP                                   ((uint8_t)(7 << 0))

/*
 * @brief    Bits definition of register PWR_MGMT_2
 * @register PWR_MGMT_2 @ 0X6C
 * @regdef   this register allows the user to enable accelerometer and gyroscope.
 */
/* DISABLE_XA */
#define ENABLE_ACCEL_X                                       ((uint8_t)(0 << 5))
#define DISABLE_ACCEL_X                                      ((uint8_t)(1 << 5))
/* DISABLE_YA */
#define ENABLE_ACCEL_Y                                       ((uint8_t)(0 << 4))
#define DISABLE_ACCEL_Y                                      ((uint8_t)(1 << 4))
/* DISABLE_ZA */
#define ENABLE_ACCEL_Z                                       ((uint8_t)(0 << 3))
#define DISABLE_ACCEL_Z                                      ((uint8_t)(1 << 3))
/* DISABLE_XG */
#define ENABLE_GYRO_X                                        ((uint8_t)(0 << 2))
#define DISABLE_GYRO_X                                       ((uint8_t)(1 << 2))
/* DISABLE_YG */
#define ENABLE_GYRO_Y                                        ((uint8_t)(0 << 1))
#define DISABLE_GYRO_Y                                       ((uint8_t)(1 << 1))
/* DISABLE_ZG */
#define ENABLE_GYRO_Z                                        ((uint8_t)(0 << 0))
#define DISABLE_GYRO_Z                                       ((uint8_t)(1 << 0))


#define MPU6500_TEMP_SCALE_FACTOR                    (1.0f/333.87f)
#define MPU6500_TEMP_OFFSET                          (21.0f) // in unit degC

/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
bool mpu6500_init(void);
float mpu6500_get_temp_raw_data(void);
void mpu6500_get_gyro_raw_data(vect3f_t* gyro_raw);
void mpu6500_get_accel_raw_data(vect3f_t* accel_raw);
void mpu6500_get_imu_raw_data(imu_raw_t* imu_raw);


#ifdef __cplusplus
}
#endif


#endif /* __DRV_MPU6500_H */


/******************* (C) COPYRIGHT 2017 PENG ZHANG *****END OF FILE********/

