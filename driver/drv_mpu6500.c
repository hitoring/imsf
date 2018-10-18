/*
 ******************************************************************************
 * @file    drv_mpu6500.c
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @brief   This file provides a driver for IMU MPU6500
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_mpu6500.h"

/* Private function prototypes -----------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const static float mpu6500_gyro_sensitivity_scale_factor[4]  = {1.0f/131.0f, 1.0f/65.5f, 1.0f/32.8f, 1.0f/16.4f};
const static float mpu6500_accel_sensitivity_scale_factor[4] = {1.0f/16384.0f, 1.0f/8192.0f, 1.0f/4096.0f, 1.0f/2048.0f};
static float gyro_selected_sensitivity_scale_factor  = 1.0f/32.8f;     // default gyro scale, corresponding to GYRO_FSR = 1000dps
static float accel_selected_sensitivity_scale_factor = 1.0f/16384.0f;  // default accel scale, corresponding to ACCEL_FSR = 2G
/* Private functions ---------------------------------------------------------*/

/*
 * @brief  Read out one byte data from MPU6500 spacific register through SPI bus 
 * @param  reg - MPU6500 register
 * @note   None
 * @retval Data read out
 */
static uint8_t mpu6500_read_one_byte(uint8_t reg)
{
	uint8_t retval;
	
	GPIO_ResetBits(MPU6500_SPI_CS_GPIO_PORT, MPU6500_SPI_CS_PIN); 
    SPI1_Read_Write_Byte(reg | 0X80); 	 // send read cmd and register addr
	retval = SPI1_Read_Write_Byte(0XFF); // read out value from register
	GPIO_SetBits(MPU6500_SPI_CS_GPIO_PORT, MPU6500_SPI_CS_PIN);   
	
	return retval;		
}

/*
 * @brief  Write one byte data to MPU6500 spacific register through SPI bus 
 * @param  reg - register address
 * @param  data - data to be writed
 * @note   None
 * @retval R/W status
 */
static uint8_t mpu6500_write_one_byte(uint8_t reg, uint8_t data) 				 
{ 
    uint8_t status;
	
	GPIO_ResetBits(MPU6500_SPI_CS_GPIO_PORT, MPU6500_SPI_CS_PIN);
	status = SPI1_Read_Write_Byte(reg);  // send write cmd and register addr
	SPI1_Read_Write_Byte(data);		     // write value into register 
	GPIO_SetBits(MPU6500_SPI_CS_GPIO_PORT, MPU6500_SPI_CS_PIN);
	
	return status;	
}


/*
 * @brief  Read out sequent multi bytes data from MPU6500 spacific register through SPI bus 
 * @param  reg - MPU6500 register
 * @param  len - data length
 * @param  buf - data storage buffer
 * @note   None
 * @retval None
 */
static void mpu6500_read_multi_bytes(uint8_t reg, uint8_t len, uint8_t* buf)
{ 
	uint8_t i;
	
	GPIO_ResetBits(MPU6500_SPI_CS_GPIO_PORT, MPU6500_SPI_CS_PIN);
 	SPI1_Read_Write_Byte(reg | 0X80); 	 // send read cmd and register addr
 	for (i=0; i<len; i++)
 	{
		buf[i] = SPI1_Read_Write_Byte(0XFF); // read out value from register circularly
 	}
	GPIO_SetBits(MPU6500_SPI_CS_GPIO_PORT, MPU6500_SPI_CS_PIN);
}

/*
 * @brief  Set MPU6500 gyro full scale range
 * @param  fsr: 0,¡À250dps;1,¡À500dps;2,¡À1000dps;3,¡À2000dps
 * @note   None
 * @retval W/R status
 */
static uint8_t mpu6500_set_gyro_fsr(uint8_t fsr)
{
	switch(fsr)
	{
		case MPU6500_GYRO_FSR_250DPS:
		{
			gyro_selected_sensitivity_scale_factor = mpu6500_gyro_sensitivity_scale_factor[0];
			break;
		}
		case MPU6500_GYRO_FSR_500DPS:
		{
			gyro_selected_sensitivity_scale_factor = mpu6500_gyro_sensitivity_scale_factor[1];
			break;
		}
		case MPU6500_GYRO_FSR_1000DPS:
		{
			gyro_selected_sensitivity_scale_factor = mpu6500_gyro_sensitivity_scale_factor[2];
			break;
		}
		case MPU6500_GYRO_FSR_2000DPS:
		{
			gyro_selected_sensitivity_scale_factor = mpu6500_gyro_sensitivity_scale_factor[3];
			break;
		}
		default:
		{
			break;
		}
	}
	return mpu6500_write_one_byte(MPU6500_GYRO_CONFIG_ADDR, fsr); 
}


/*
 * @brief  Set MPU6500 accel full scale range
 * @param  fsr: 0,¡À2g;1,¡À4g;2,¡À8g;3,¡À16g
 * @note   None
 * @retval W/R status
 */

static uint8_t mpu6500_set_accel_fsr(uint8_t fsr)
{
	switch(fsr)
	{
		case MPU6500_ACCEL_FSR_2G:
		{
			accel_selected_sensitivity_scale_factor = mpu6500_accel_sensitivity_scale_factor[0];
			break;
		}
		case MPU6500_ACCEL_FSR_4G:
		{
			accel_selected_sensitivity_scale_factor = mpu6500_accel_sensitivity_scale_factor[1];
			break;
		}
		case MPU6500_ACCEL_FSR_8G:
		{
			accel_selected_sensitivity_scale_factor = mpu6500_accel_sensitivity_scale_factor[2];
			break;
		}
		case MPU6500_ACCEL_FSR_16G:
		{
			accel_selected_sensitivity_scale_factor = mpu6500_accel_sensitivity_scale_factor[3];
			break;
		}
		default:
		{
			break;
		}
	}
	return mpu6500_write_one_byte(MPU6500_ACCEL_CONFIG_ADDR, fsr);  
}


 /*
  * @brief  Set MPU6500 DLPF
  * @param  dlpf: digital low pass filter frequency
  * @note   None
  * @retval W/R status
  */
static uint8_t mpu6500_set_dlpf(uint8_t dlpf)
{
	return mpu6500_write_one_byte(MPU6500_CONFIG_ADDR, dlpf); 
}


/*
 * @brief	Set MPU6500 ADLPF
 * @param	adlpf: accel digital low pass filter frequency
 * @note	None
 * @retval  W/R status
 */
static uint8_t mpu6500_set_adlpf(uint8_t adlpf)
{
	return mpu6500_write_one_byte(MPU6500_ACCEL_CONFIG2_ADDR, adlpf);
}


/*
 * @brief	Set MPU6500 sample rate, if dlpf != 0 && dlpf != 7, while Fs = 1kHz
 * @param	rate: internal sample rate-(4~1000Hz)
 * @note	None
 * @retval  W/R status
 */
static uint8_t mpu6500_set_sample_rate(uint16_t rate)
{
	uint8_t data;
	if(rate > 1000)
	{
		rate = 1000;
	}
	if(rate < 4)
	{
		rate = 4;
	}
	data = (1000 / rate - 1);
	return mpu6500_write_one_byte(MPU6500_SMPLRT_DIV_ADDR, data);
}
/*
 * @brief  MPU6500 Init
 * @param  None
 * @note   None
 * @retval Init status
 */
bool mpu6500_init(void)
{ 
	uint8_t res;
	
	SPI1_Init(); // SPI init
	mpu6500_write_one_byte(MPU6500_PWR_MGMT_1_ADDR, DEVICE_RESET_EN);	// mpu6500 reset
    Delay_Ms(100);
	mpu6500_write_one_byte(MPU6500_SIGNAL_PATH_RESET_ADDR, GYRO_RST_EN|ACCEL_RST_EN|TEMP_RST_EN);	// gyro, accel and temp reset
    Delay_Ms(100);

	res = mpu6500_read_one_byte(MPU6500_WHO_AM_I_ADDR); // read device id
	//printf("The ID of the device is 0x%02x\r\n", res);

	if(res == MPU6500_WHO_AM_I) // if device_id is right
	{
		mpu6500_write_one_byte(MPU6500_PWR_MGMT_1_ADDR, MPU6500_CLKSRC_AUTO_SELECT);	// set auto select clock source
		mpu6500_write_one_byte(MPU6500_PWR_MGMT_2_ADDR, ENABLE_ACCEL_X|ENABLE_ACCEL_Y|ENABLE_ACCEL_Z|ENABLE_GYRO_X|ENABLE_GYRO_Y|ENABLE_GYRO_Z);	// enable 3-axis gyro and accel
		mpu6500_set_gyro_fsr(MPU6500_GYRO_FSR_1000DPS);				// gyro fsr 1000dps
		mpu6500_set_accel_fsr(MPU6500_ACCEL_FSR_2G);				// accel fsr 2g	
	    mpu6500_set_dlpf(MPU6500_DLPF_GYRO_BW_20HZ_RATE_1KHZ);     // set dlpf 184hz, delay 2.9ms
	    mpu6500_set_adlpf(MPU6500_ADLPF_ACCEL_BW_20HZ_RATE_1KHZ);  // set adlpf 460hz, delay 1.94ms
		mpu6500_set_sample_rate(1000);						        // set internal sample rate 1kHz
		mpu6500_write_one_byte(MPU6500_FIFO_EN_ADDR, 0X00);	        // disable fifo
		mpu6500_write_one_byte(MPU6500_INT_ENABLE_ADDR, 0X00);	    // disable all interrupt
		mpu6500_write_one_byte(MPU6500_USER_CTRL_ADDR, 0X00);	    // user ctrl
 	}
	else 
	{
		return false; // init error
	}
	return true;
}

/*
 * @brief  Get IMU temperature raw data
 * @param  None
 * @note   None
 * @retval Temp raw data
 */
float mpu6500_get_temp_raw_data(void)
{
	uint8_t buf[2]; 
	float temp_raw;
	mpu6500_read_multi_bytes(MPU6500_TEMP_OUT_H_ADDR, 2, buf);
	temp_raw = MAKE_WORD(buf[0], buf[1]) * MPU6500_TEMP_SCALE_FACTOR + MPU6500_TEMP_OFFSET;

	return temp_raw;
}

/*
 * @brief  Get IMU gyroscope raw data
 * @param  None
 * @note   None
 * @retval Gyro raw data
 */

void mpu6500_get_gyro_raw_data(vect3f_t* gyro_raw)
{
    uint8_t buf[6];  
	mpu6500_read_multi_bytes(MPU6500_GYRO_XOUT_H_ADDR, 6, buf);
	gyro_raw->x = MAKE_WORD(buf[0], buf[1]) * gyro_selected_sensitivity_scale_factor; 
	gyro_raw->y = MAKE_WORD(buf[2], buf[3]) * gyro_selected_sensitivity_scale_factor; 
	gyro_raw->z = MAKE_WORD(buf[4], buf[5]) * gyro_selected_sensitivity_scale_factor; 
}

/*
 * @brief  Get IMU accelerometer raw data
 * @param  None
 * @note   None
 * @retval Accel raw data
 */

void mpu6500_get_accel_raw_data(vect3f_t* accel_raw)
{
    uint8_t buf[6];  
	mpu6500_read_multi_bytes(MPU6500_ACCEL_XOUT_H_ADDR, 6, buf);
	accel_raw->x = MAKE_WORD(buf[0], buf[1]) * accel_selected_sensitivity_scale_factor;  
	accel_raw->y = MAKE_WORD(buf[2], buf[3]) * accel_selected_sensitivity_scale_factor;  
	accel_raw->z = MAKE_WORD(buf[4], buf[5]) * accel_selected_sensitivity_scale_factor;
}

/*
 * @brief  Get IMU all output raw data
 * @param  None
 * @note   All = gyro + accel + temp
 * @retval All raw data
 */
void mpu6500_get_imu_raw_data(imu_raw_t* imu_raw)
{
	uint8_t buf[14];  
	mpu6500_read_multi_bytes(MPU6500_ACCEL_XOUT_H_ADDR, 14, buf);
	/* accel data */
	imu_raw->accel_x = MAKE_WORD(buf[0],  buf[1])  * accel_selected_sensitivity_scale_factor;  
	imu_raw->accel_y = MAKE_WORD(buf[2],  buf[3])  * accel_selected_sensitivity_scale_factor;  
	imu_raw->accel_z = MAKE_WORD(buf[4],  buf[5])  * accel_selected_sensitivity_scale_factor;
	/* temp data */
	imu_raw->temp    = MAKE_WORD(buf[6],  buf[7])  * MPU6500_TEMP_SCALE_FACTOR + MPU6500_TEMP_OFFSET;
	/* gyro data */
	imu_raw->gyro_x  = MAKE_WORD(buf[8],  buf[9])  * gyro_selected_sensitivity_scale_factor; 
	imu_raw->gyro_y  = MAKE_WORD(buf[10], buf[11]) * gyro_selected_sensitivity_scale_factor; 
	imu_raw->gyro_z  = MAKE_WORD(buf[12], buf[13]) * gyro_selected_sensitivity_scale_factor;
}


/******************* (C) COPYRIGHT 2017 PENG ZHANG *****END OF FILE********/

