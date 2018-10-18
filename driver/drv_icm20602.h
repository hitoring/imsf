/*
 ******************************************************************************
 * @file    drv_icm20602.h
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-08
 * @brief   This file provides a driver for IMU ICM-20602.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRV_ICM20602_H
#define __DRV_ICM20602_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "drv_delay.h"
#include "drv_spi.h"
#include "math_vector.h"

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

/*
 * @brief ICM20602 Registers Address 
 */

/* Accelerometer Self-Test Registers Address */
#define ICM20602_SELF_TEST_X_ACCEL_ADDR		0X0D	//自检寄存器X
#define ICM20602_SELF_TEST_Y_ACCEL_ADDR		0X0E	//自检寄存器Y
#define ICM20602_SELF_TEST_Z_ACCEL_ADDR		0X0F	//自检寄存器Z
/* Gyroscope Offset Adjustment Register Address */
#define ICM20602_XG_OFFS_USRH_ADDR			0X13
#define ICM20602_XG_OFFS_USRL_ADDR			0X14
#define ICM20602_YG_OFFS_USRH_ADDR			0X15
#define ICM20602_YG_OFFS_USRL_ADDR			0X16
#define ICM20602_ZG_OFFS_USRH_ADDR			0X17
#define ICM20602_ZG_OFFS_USRL_ADDR			0X18
/* Sample Rate Divider Register Address */
#define ICM20602_SMPLRT_DIV_ADDR				0X19	//采样频率分频器
/* Config Registers Address */
#define ICM20602_CONFIG_ADDR					0X1A	//配置寄存器
#define ICM20602_GYRO_CONFIG_ADDR		    0X1B	//陀螺仪配置寄存器
#define ICM20602_ACCEL_CONFIG_ADDR			0X1C	//加速度计配置寄存器
#define ICM20602_ACCEL_CONFIG2_ADDR			0X1D	//加速度计配置寄存器2
#define ICM20602_LP_MODE_CFG_ADDR            0X1E
#define ICM20602_ACCEL_WOM_THR_ADDR		    0X1F	//运动检测阀值设置寄存器
/* FIFO Enable Registers Address */
#define ICM20602_FIFO_EN_ADDR			    0X23	//FIFO使能寄存器
/* Interrupt Enable Address */
#define ICM20602_FSYNC_INT_ADDR				0X36	
#define ICM20602_INT_PIN_CFG_ADDR		    0X37	//中断/旁路设置寄存器
#define ICM20602_INT_ENABLE_ADDR				0X38	//中断使能寄存器
#define ICM20602_INT_STATUS_ADDR				0X3A	//中断状态寄存器
/* Accelerometer Output Registers Address */
#define ICM20602_ACCEL_XOUT_H_ADDR			0X3B	//加速度值,X轴高8位寄存器
#define ICM20602_ACCEL_XOUT_L_ADDR			0X3C	//加速度值,X轴低8位寄存器
#define ICM20602_ACCEL_YOUT_H_ADDR			0X3D	//加速度值,Y轴高8位寄存器
#define ICM20602_ACCEL_YOUT_L_ADDR			0X3E	//加速度值,Y轴低8位寄存器
#define ICM20602_ACCEL_ZOUT_H_ADDR			0X3F	//加速度值,Z轴高8位寄存器
#define ICM20602_ACCEL_ZOUT_L_ADDR			0X40	//加速度值,Z轴低8位寄存器
/* Temperature Output Registers Address */
#define ICM20602_TEMP_OUT_H_ADDR		    	0X41	//温度值高八位寄存器
#define ICM20602_TEMP_OUT_L_ADDR		    	0X42	//温度值低8位寄存器
/* Gyroscope Output Registers Address */
#define ICM20602_GYRO_XOUT_H_ADDR		    0X43	//陀螺仪值,X轴高8位寄存器
#define ICM20602_GYRO_XOUT_L_ADDR		    0X44	//陀螺仪值,X轴低8位寄存器
#define ICM20602_GYRO_YOUT_H_ADDR		    0X45	//陀螺仪值,Y轴高8位寄存器
#define ICM20602_GYRO_YOUT_L_ADDR		    0X46	//陀螺仪值,Y轴低8位寄存器
#define ICM20602_GYRO_ZOUT_H_ADDR		    0X47	//陀螺仪值,Z轴高8位寄存器
#define ICM20602_GYRO_ZOUT_L_ADDR		    0X48	//陀螺仪值,Z轴低8位寄存器
/* Gyroscope Self-Test Registers Address */
#define ICM20602_SELF_TEST_X_GYRO_ADDR       0X50
#define ICM20602_SELF_TEST_Y_GYRO_ADDR       0X51
#define ICM20602_SELF_TEST_Z_GYRO_ADDR       0X52
/* Signal Path Reset & Power Management Registers Address */
#define ICM20602_SIGNAL_PATH_RESET_ADDR		0X68	//信号通道复位寄存器
#define ICM20602_ACC_INTEL_CTRL_ADDR	    	0X69	//运动检测控制寄存器
#define ICM20602_USER_CTRL_ADDR		    	0X6A	//用户控制寄存器
#define ICM20602_PWR_MGMT_1_ADDR		    	0X6B	//电源管理寄存器1
#define ICM20602_PWR_MGMT_2_ADDR		    	0X6C	//电源管理寄存器2 
#define ICM20602_FIFO_COUNTH_ADDR		    0X72	//FIFO计数寄存器高八位
#define ICM20602_FIFO_COUNTL_ADDR		    0X73	//FIFO计数寄存器低八位
#define ICM20602_FIFO_R_W_ADDR				0X74	//FIFO读写寄存器
#define ICM20602_WHO_AM_I_ADDR		    	0X75	//器件ID寄存器
/* Accelerometer Offset Registers Address */
#define ICM20602_XA_OFFSET_H_ADDR		    0X77	
#define ICM20602_XA_OFFSET_L_ADDR		    0X78
#define ICM20602_YA_OFFSET_H_ADDR		    0X7A
#define ICM20602_YA_OFFSET_L_ADDR		    0X7B
#define ICM20602_ZA_OFFSET_H_ADDR		    0X7D
#define ICM20602_ZA_OFFSET_L_ADDR		    0X7E

/* The default value of WHO_AM_I register Address */
#define ICM20602_WHO_AM_I				    0X12



/*
 * @brief ICM20602 Registers Description 
 */
 
/* 
 * @brief    The DLPF is configured by DLPF_CFG, when FCHOICE_B [1:0] = 2b'00.
 *           The gyroscope and temperature sensor are filtered according to the value of
 *           DLPF_CFG as shown in the table below.
 * @register CONFIG @ 0X1A
 * @table
 *------------------------------------------------------------
 *DLPF_CFG|               Gyroscope             | Temperature|
 *        | 3-dB BW(Hz) |Noise BW(Hz)| Rate(kHz)| 3-dB BW(Hz)|
 *------------------------------------------------------------
 *   0    |      250    |   306.6    |     8    |    4000    |
 *------------------------------------------------------------
 *   1    |      176    |   177.0    |     1    |    188     |
 *------------------------------------------------------------
 *   2    |      92     |   108.6    |     1    |    98      |
 *------------------------------------------------------------
 *   3    |      41     |   59.0     |     1    |    42      |
 *------------------------------------------------------------
 *   4    |      20     |   30.5     |     1    |    20      |
 *------------------------------------------------------------
 *   5    |      10     |   15.6     |     1    |    10      |
 *------------------------------------------------------------
 *   6    |      5      |   8.0      |     1    |    5       |
 *------------------------------------------------------------
 *   7    |      3281   |   3451.0   |     1    |    4000    |
 *------------------------------------------------------------
 */

#define ICM20602_DLPF_GYRO_BW_250HZ_RATE_8KHZ                ((uint8_t)0)
#define ICM20602_DLPF_GYRO_BW_176HZ_RATE_1KHZ                ((uint8_t)1)
#define	ICM20602_DLPF_GYRO_BW_92HZ_RATE_1KHZ                 ((uint8_t)2)
#define	ICM20602_DLPF_GYRO_BW_41HZ_RATE_1KHZ                 ((uint8_t)3)
#define	ICM20602_DLPF_GYRO_BW_20HZ_RATE_1KHZ                 ((uint8_t)4)
#define	ICM20602_DLPF_GYRO_BW_10HZ_RATE_1KHZ                 ((uint8_t)5)
#define	ICM20602_DLPF_GYRO_BW_5HZ_RATE_1KHZ                  ((uint8_t)6)
#define	ICM20602_DLPF_GYRO_BW_3281HZ_RATE_8KHZ               ((uint8_t)7)

/*
 * @brief    FS_SEL selects the full scale range of the gyroscope outputs.
 * @register GYRO_CONFIG @ 0X1B
 */
#define	ICM20602_GYRO_FSR_250DPS    ((uint8_t)(0 << 3))
#define	ICM20602_GYRO_FSR_500DPS    ((uint8_t)(1 << 3))
#define	ICM20602_GYRO_FSR_1000DPS   ((uint8_t)(2 << 3))
#define	ICM20602_GYRO_FSR_2000DPS   ((uint8_t)(3 << 3))

/*
 * @brief    AFS_SEL selects the full scale range of the accelerometer outputs.
 * @register ACCEL_CONFIG @ 0X1C
 */
#define	ICM20602_ACCEL_FSR_2G    ((uint8_t)(0 << 3))
#define	ICM20602_ACCEL_FSR_4G    ((uint8_t)(1 << 3))
#define	ICM20602_ACCEL_FSR_8G    ((uint8_t)(2 << 3))
#define	ICM20602_ACCEL_FSR_16G   ((uint8_t)(3 << 3))

/*
 * @brief    This register determines which sensor measurements are loaded into the
 *           FIFO buffer.
 * @register FIFO_EN @ 0X23
 */
typedef enum
{
	TEMP_FIFO_DIS = 0,
	TEMP_FIFO_EN  = 1
}TEMP_FIFO_EN_TypeDef;

typedef enum
{
	GYROX_FIFO_DIS = 0,
	GYROX_FIFO_EN  = 1
}GYROX_FIFO_EN_TypeDef;

typedef enum
{
	GYROY_FIFO_DIS = 0,
	GYROY_FIFO_EN  = 1
}GYROY_FIFO_EN_TypeDef;

typedef enum
{
	GYROZ_FIFO_DIS = 0,
	GYROZ_FIFO_EN  = 1
}GYROZ_FIFO_EN_TypeDef;

typedef enum
{
	ACCEL_FIFO_DIS = 0,
	ACCEL_FIFO_EN  = 1
}ACCEL_FIFO_EN_TypeDef;

typedef struct
{
	TEMP_FIFO_EN_TypeDef temp_fifo_en_bit;
	GYROX_FIFO_EN_TypeDef gyrox_fifo_en_bit;
	GYROY_FIFO_EN_TypeDef gyroy_fifo_en_bit;
	GYROZ_FIFO_EN_TypeDef gyroz_fifo_en_bit;
	ACCEL_FIFO_EN_TypeDef accel_fifo_en_bit;
}FIFO_EN_TypeDef;

/*
 * @brief    This register allows the user to enable or disable the FIFO buffer,
 *           I2C Master Mode, and primary I2C interface. 
 * @register USER_CTRL @ 0X6A
 */
typedef enum
{
	FIFO_DIS = 0,
	FIFO_EN  = 1
}USER_CTRL_FIFO_EN_TypeDef;

typedef enum
{
	I2C_MST_DIS = 0,
	I2C_MST_EN  = 1
}USER_CTRL_I2C_MST_EN_TypeDef;

typedef struct
{
	USER_CTRL_FIFO_EN_TypeDef user_ctrl_fifo_en_bit;
	USER_CTRL_I2C_MST_EN_TypeDef user_ctrl_i2c_mst_en_bit;
}USER_CTRL_CFG_TypeDef;

/*
 * @brief    This register allows the user to configure the power mode 
 *           and clock source.
 * @register PWR_MGMT_1 @ 0X6B
 */
typedef enum
{
	DEVICE_RESET_OFF = 0,
	DEVICE_RESET_ON  = 1
}DEVICE_RESET_TypeDef;

typedef enum
{
	DEVICE_SLEEP_OFF = 0,
	DEVICE_SLEEP_ON  = 1
}DEVICE_SLEEP_TypeDef;

typedef enum
{
	DEVICE_CYCLE_OFF = 0,
	DEVICE_CYCLE_ON  = 1
}DEVICE_CYCLE_TypeDef;

typedef enum
{
	DEVICE_TEMP_EN =   0,
	DEVICE_TEMP_DIS  = 1
}DEVICE_TEMP_EN_TypeDef;

typedef enum
{
	ICM20602_CLKSRC_INTER_OSC     = 0,
	ICM20602_CLKSRC_PLL_GYROX_REF = 1,
	ICM20602_CLKSRC_PLL_GYROY_REF = 2,
	ICM20602_CLKSRC_PLL_GYROZ_REF = 3,
	ICM20602_CLKSRC_EXT_32768HZ   = 4,
	ICM20602_CLKSRC_EXT_19MHZ     = 5,
	ICM20602_CLOCK_STOP           = 7
}ICM20602_CLKSRC_CFG_TypeDef;

typedef struct
{
	DEVICE_RESET_TypeDef device_reset_bit;
	DEVICE_SLEEP_TypeDef device_sleep_bit;
	DEVICE_CYCLE_TypeDef device_cycle_bit;
	DEVICE_TEMP_EN_TypeDef device_temp_en_bit;
	ICM20602_CLKSRC_CFG_TypeDef mpu6050_clksrc_cfg_bit;
}PWR_MGMT_1_CFG_TypeDef;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define MAKE_WORD(HIGH, LOW)  (((uint16_t)((HIGH) << 8)) | (LOW))

/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
bool icm20602_init(void);
float icm20602_get_temp_raw_data(void);
void icm20602_get_gyro_raw_data(vect3f_t* gyro_raw);
void icm20602_get_accel_raw_data(vect3f_t* accel_raw);
void icm20602_get_imu_raw_data(imu_raw_t* imu_raw);

#ifdef __cplusplus
}
#endif





#endif /* __DRV_ICM20602_H */


/******************* (C) COPYRIGHT 2017 PENG ZHANG *****END OF FILE********/
