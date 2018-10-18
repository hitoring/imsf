/*
 ******************************************************************************
 * @file    drv_icm20602.c
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @brief   This file provides a driver for IMU ICM-20602
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_icm20602.h"


/* Private function prototypes -----------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const static float icm20602_gyro_sensitivity_scale_factor[4]  = {1/131.0f, 1/65.5f, 1/32.8f, 1/16.4f};
const static float icm20602_accel_sensitivity_scale_factor[4] = {1/16384.0f, 1/8192.0f, 1/4096.0f, 1/2048.0f};
const static float icm20602_temp_sensitivity_scale_factor = 1/326.8f;
static float gyro_selected_sensitivity_scale_factor = 1/32.8f; // default gyro scale, corresponding to GYRO_FSR = 1000dps
static float accel_selected_sensitivity_scale_factor = 1/16384.0f; // default accel scale, corresponding to ACCEL_FSR = 2G
/* Private functions ---------------------------------------------------------*/


/*
 * @brief  通过SPI从MPU6050读一个字节 
 * @param  reg:寄存器地址 
 * @note   None
 * @retval 读到的数据
 */
uint8_t ICM20602_Read_One_Byte(uint8_t reg)
{
	uint8_t res;
    SPI_start(); 
	SPI_send_One_Byte((ICM20602_ADDR << 1) | 0);//发送器件地址+写命令	
	SPI_wait_ack();		//等待应答 
    SPI_send_One_Byte(reg);	//写寄存器地址
    SPI_wait_ack();		//等待应答
    SPI_start();
	SPI_send_One_Byte((ICM20602_ADDR << 1) | 1);//发送器件地址+读命令	
    SPI_wait_ack();		//等待应答 
	res = SPI_recv_One_Byte(0);//读取数据,发送nACK 
    SPI_stop();			//产生一个停止条件 
	return res;		
}

/*
 * @brief  通过SPI向MPU6050写入一个字节 
 * @param  reg:寄存器地址 
 *         data: 要写入的数据
 * @note   None
 * @retval 0，正常；其他，错误代码
 */
uint8_t ICM20602_Write_One_Byte(uint8_t reg, uint8_t data) 				 
{ 
    SPI_start(); 
	SPI_send_One_Byte((ICM20602_ADDR << 1) | 0);//发送器件地址+写命令	
	if(SPI_wait_ack())	//等待应答
	{
		SPI_stop();		 
		return 1;		
	}
    SPI_send_One_Byte(reg);	//写寄存器地址
    SPI_wait_ack();		//等待应答 
	SPI_send_One_Byte(data);//发送数据
	if(SPI_wait_ack())	//等待ACK
	{
		SPI_stop();	 
		return 1;		 
	}		 
    SPI_stop();	 
	return 0;
}

/*
 * @brief  连续通过SPI从MPU6050读取多个字节 
 * @param  addr: 器件地址 
 *         reg: 要读取的寄存器地址 
 *         len: 要读取的长度，比如几个字节
 *         buf: 读取到的数据存储区
 * @note   None
 * @retval 0，正常；其他，错误代码
 */
uint8_t ICM20602_Read_Multi_Bytes(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{ 
 	SPI_start(); 
	SPI_send_One_Byte((addr << 1) | 0);//发送器件地址+写命令	
	if(SPI_wait_ack())	//等待应答
	{
		SPI_stop();		 
		return 1;		
	}
    SPI_send_One_Byte(reg);	//写寄存器地址
    SPI_wait_ack();		//等待应答
    SPI_start();
	SPI_send_One_Byte((addr << 1) | 1);//发送器件地址+读命令	
    SPI_wait_ack();		//等待应答 
	while(len)
	{
		if(len == 1)
		{
			*buf = SPI_recv_One_Byte(0);//读数据,发送nACK 
		}
		else 
		{
			*buf = SPI_recv_One_Byte(1);		//读数据,发送ACK  
		}
		len--;
		buf++; 
	}    
    SPI_stop();	//产生一个停止条件 
	return 0;	
}

/*
 * @brief  连续通过SPI向MPU6050写入多个字节 
 * @param  addr: 器件地址 
 *         reg: 要写入的寄存器地址 
 *         len: 要写入的长度
 *         buf: 写入的数据存储区
 * @note   None
 * @retval 0，正常；其他，错误代码
 */
uint8_t ICM20602_Write_Multi_Bytes(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
	uint8_t i; 
    SPI_start(); 
	SPI_send_One_Byte((addr << 1) | 0);//发送器件地址+写命令	
	if(SPI_wait_ack())	//等待应答
	{
		SPI_stop();		 
		return 1;		
	}
    SPI_send_One_Byte(reg);	//写寄存器地址
    SPI_wait_ack();		//等待应答
	for(i=0;i<len;i++)
	{
		SPI_send_One_Byte(buf[i]);	//发送数据
		if(SPI_wait_ack())		//等待ACK
		{
			SPI_stop();	 
			return 1;		 
		}		
	}    
    SPI_stop();	 
	return 0;	
} 

/*
 * @brief  设置MPU6050陀螺仪满量程范围
 * @param  fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
 * @note   None
 * @retval 0,设置成功；其他,设置失败
 */
uint8_t ICM20602_Set_Gyro_FSR(uint8_t fsr)
{
	switch(fsr)
	{
		case ICM20602_GYRO_FSR_250DPS:
		{
			gyro_selected_sensitivity_scale_factor = icm20602_gyro_sensitivity_scale_factor[0];
			break;
		}
		case ICM20602_GYRO_FSR_500DPS:
		{
			gyro_selected_sensitivity_scale_factor = icm20602_gyro_sensitivity_scale_factor[1];
			break;
		}
		case ICM20602_GYRO_FSR_1000DPS:
		{
			gyro_selected_sensitivity_scale_factor = icm20602_gyro_sensitivity_scale_factor[2];
			break;
		}
		case ICM20602_GYRO_FSR_2000DPS:
		{
			gyro_selected_sensitivity_scale_factor = icm20602_gyro_sensitivity_scale_factor[3];
			break;
		}
		default:
		{
			break;
		}
	}
	return ICM20602_Write_One_Byte(ICM20602_GYRO_CONFIG_ADDR, fsr); //设置陀螺仪满量程范围  
}

/*
 * @brief  设置MPU6050加速度计满量程范围
 * @param  fsr:0,±2g;1,±4g;2,±8g;3,±16g
 * @note   None
 * @retval 0,设置成功；其他,设置失败
 */
uint8_t ICM20602_Set_Accel_FSR(uint8_t fsr)
{
	switch(fsr)
	{
		case ICM20602_ACCEL_FSR_2G:
		{
			accel_selected_sensitivity_scale_factor = icm20602_accel_sensitivity_scale_factor[0];
			break;
		}
		case ICM20602_ACCEL_FSR_4G:
		{
			accel_selected_sensitivity_scale_factor = icm20602_accel_sensitivity_scale_factor[1];
			break;
		}
		case ICM20602_ACCEL_FSR_8G:
		{
			accel_selected_sensitivity_scale_factor = icm20602_accel_sensitivity_scale_factor[2];
			break;
		}
		case ICM20602_ACCEL_FSR_16G:
		{
			accel_selected_sensitivity_scale_factor = icm20602_accel_sensitivity_scale_factor[3];
			break;
		}
		default:
		{
			break;
		}
	}
	return ICM20602_Write_One_Byte(ICM20602_ACCEL_CONFIG_ADDR, fsr);//设置加速度传感器满量程范围  
}

/*
 * @brief 设置MPU6050的数字低通滤波器
 * @param  dlpf: 数字低通滤波频率项
 * @note   None
 * @retval 0, 设置成功；其他，设置失败
 */
uint8_t ICM20602_Set_DLPF(uint8_t dlpf)
{
	return ICM20602_Write_One_Byte(ICM20602_CONFIG_ADDR, dlpf);//设置数字低通滤波器  
}

/*
 * @brief  设置MPU6050的陀螺仪采样率(如果dlpf不等于0和7，则Fs=1KHz)，
 *         (加速度计采样率恒为1000kHz)
 * @param  rate: 4~1000(Hz)
 * @note   None
 * @retval 0, 设置成功；其他，设置失败
 */
uint8_t ICM20602_Set_Sample_Rate(uint16_t rate)
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
	return ICM20602_Write_One_Byte(ICM20602_SMPLRT_DIV_ADDR, data);	//设置数字低通滤波器
}

/*
 * @brief 初始化MPU6050
 * @param  None
 * @note   None
 * @retval 0, 成功；其他，错误代码
 */
uint8_t ICM20602_Init(void)
{ 
	uint8_t res;
	SPI_Init(); // SPI init
	ICM20602_Write_One_Byte(ICM20602_PWR_MGMT_1_ADDR, DEVICE_RESET_EN);	// icm20602 reset
    delay_ms(100);
	ICM20602_Write_One_Byte(ICM20602_SIGNAL_PATH_RESET_ADDR, GYRO_RST_EN|ACCEL_RST_EN|TEMP_RST_EN);	// gyro, accel and temp reset
    delay_ms(100);
	ICM20602_Write_One_Byte(ICM20602_PWR_MGMT_1_ADDR, 0X00);	// awaken icm20602, necessary?
	ICM20602_Set_Gyro_FSR(ICM20602_GYRO_FSR_1000DPS);				//gyro fsr 1000dps
	ICM20602_Set_Accel_FSR(ICM20602_ACCEL_FSR_2G);					//accel fsr 2g
    ICM20602_Set_DLPF(ICM20602_DLPF_GYRO_BW_176HZ_RATE_1KHZ); // set dlpf
	ICM20602_Set_Sample_Rate(200);						// set sample rate 200Hz
	ICM20602_Write_One_Byte(ICM20602_INT_ENABLE_ADDR, 0X00);	// disable all interrupt
	ICM20602_Write_One_Byte(ICM20602_USER_CTRL_ADDR, 0X00);	//SPI主模式关闭
	ICM20602_Write_One_Byte(ICM20602_FIFO_EN_ADDR, 0X00);	// disable fifo
	ICM20602_Write_One_Byte(ICM20602_INT_PIN_CFG_ADDR, 0X80);	//INT引脚低电平有效
	res = ICM20602_Read_One_Byte(ICM20602_WHO_AM_I_ADDR);
	if(res == ICM20602_WHO_AM_I)//器件ID正确
	{
		ICM20602_Write_One_Byte(ICM20602_PWR_MGMT_1_ADDR, 0X01);	//设置CLKSEL,PLL X轴为参考
		ICM20602_Write_One_Byte(ICM20602_PWR_MGMT_2_ADDR, 0X00);	//加速度与陀螺仪都工作
		ICM20602_Set_Sample_Rate(200);						//设置采样率为50Hz
 	}
	else 
	{
		return 1; //初始化错误
	}
	return 0;
}

/*
 * @brief  得到IMU温度原始值
 * @param  imu_raw_data结构体
 * @note   temp=36.53 + ((double)raw)/340
 * @retval 0, 成功；其他，错误代码
 */
uint8_t ICM20602_Get_Temp_Raw_Data(float temp_raw)
{
	uint8_t buf[2], res;  
	res=ICM20602_Read_Multi_Bytes(ICM20602_ADDR, ICM20602_TEMP_OUT_H_ADDR, 2, buf);
	if(res==0)
	{
		temp_raw = MAKE_WORD(buf[0], buf[1]) * icm20602_temp_sensitivity_scale_factor + 25.0f;
	} 	
    return res;
}

/*
 * @brief  得到IMU中陀螺仪原始值
 * @param  imu_raw_data结构体
 * @note   None
 * @retval 0, 成功；其他，错误代码
 */
uint8_t ICM20602_Get_Gyro_Raw_Data(vect3f_t* gyro_raw)
{
    uint8_t buf[6], res;  
	res=ICM20602_Read_Multi_Bytes(ICM20602_ADDR, ICM20602_GYRO_XOUT_H_ADDR, 6, buf);
	if(res==0)
	{
		gyro_raw.x = MAKE_WORD(buf[0], buf[1]) * gyro_selected_sensitivity_scale_factor; 
		gyro_raw.y = MAKE_WORD(buf[2], buf[3]) * gyro_selected_sensitivity_scale_factor; 
		gyro_raw.z = MAKE_WORD(buf[4], buf[5]) * gyro_selected_sensitivity_scale_factor; 
	} 	
    return res;
}

/*
 * @brief  得到IMU中加速度计原始值
 * @param  imu_raw_data结构体
 * @note   None
 * @retval 0, 成功；其他，错误代码
 */
uint8_t ICM20602_Get_Accel_Raw_Data(vect3f_t* accel_raw)
{
    uint8_t buf[6], res;  
	res=ICM20602_Read_Multi_Bytes(ICM20602_ADDR, ICM20602_ACCEL_XOUT_H_ADDR, 6, buf);
	if(res==0)
	{
		accel_raw.x = MAKE_WORD(buf[0], buf[1]) * accel_selected_sensitivity_scale_factor;  
		accel_raw.y = MAKE_WORD(buf[2], buf[3]) * accel_selected_sensitivity_scale_factor;  
		accel_raw.z = MAKE_WORD(buf[4], buf[5]) * accel_selected_sensitivity_scale_factor;
	} 	
    return res;
}



/******************* (C) COPYRIGHT 2017 PENG ZHANG *****END OF FILE********/
