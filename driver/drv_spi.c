/*
 ******************************************************************************
 * @file    drv_spi.c
 * @author  Shi Daolin
 * @version V1.0.0
 * @date    2017-6-8
 * @brief   This file provides a driver for GCD spi
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_spi.h"

/* Private function prototypes -----------------------------------------------*/
uint32_t ICM20602_TIMEOUT_UserCallback(void);

/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

__IO uint32_t  ICM20602Timeout = ICM20602_FLAG_TIMEOUT;  


/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the low level interface used to drive the ICM20602
  * @param  None
  * @retval None
  */
void SPI1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

  /* Enable the SPI periph */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

  /* Enable SCK, MOSI and MISO GPIO clocks */
  RCC_AHBPeriphClockCmd(ICM20602_SPI_SCK_GPIO_CLK | ICM20602_SPI_MISO_GPIO_CLK | ICM20602_SPI_MOSI_GPIO_CLK, ENABLE);

  /* Enable CS  GPIO clock */
  RCC_AHBPeriphClockCmd(ICM20602_SPI_CS_GPIO_CLK, ENABLE);

  GPIO_PinAFConfig(ICM20602_SPI_SCK_GPIO_PORT, ICM20602_SPI_SCK_SOURCE, ICM20602_SPI_SCK_AF);
  GPIO_PinAFConfig(ICM20602_SPI_MISO_GPIO_PORT, ICM20602_SPI_MISO_SOURCE, ICM20602_SPI_MISO_AF);
  GPIO_PinAFConfig(ICM20602_SPI_MOSI_GPIO_PORT, ICM20602_SPI_MOSI_SOURCE, ICM20602_SPI_MOSI_AF);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;//GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = ICM20602_SPI_SCK_PIN;
  GPIO_Init(ICM20602_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  /* SPI  MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  ICM20602_SPI_MOSI_PIN;
  GPIO_Init(ICM20602_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /* SPI MISO pin configuration */
  GPIO_InitStructure.GPIO_Pin = ICM20602_SPI_MISO_PIN;
  GPIO_Init(ICM20602_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

  /* SPI configuration -------------------------------------------------------*/
  SPI_I2S_DeInit(ICM20602_SPI);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_Init(ICM20602_SPI, &SPI_InitStructure);

  /* Configure the RX FIFO Threshold */
  SPI_RxFIFOThresholdConfig(ICM20602_SPI, SPI_RxFIFOThreshold_QF);
  /* Enable SPI1  */
  SPI_Cmd(ICM20602_SPI, ENABLE);

  /* Configure GPIO PIN for Lis Chip select */
  GPIO_InitStructure.GPIO_Pin = ICM20602_SPI_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(ICM20602_SPI_CS_GPIO_PORT, &GPIO_InitStructure);

  /* Deselect : Chip Select low */
  GPIO_ResetBits(ICM20602_SPI_CS_GPIO_PORT, ICM20602_SPI_CS_PIN);
}

/**
  * @brief  Sends a Byte through the SPI interface and return the Byte received 
  *         from the SPI bus.
  * @param  Byte : Byte send.
  * @retval The received byte value
  */
uint8_t SPI1_Read_Write_Byte(uint8_t byte)
{
  /* Loop while DR register in not empty */
  ICM20602Timeout = ICM20602_FLAG_TIMEOUT;
  while (SPI_I2S_GetFlagStatus(ICM20602_SPI, SPI_I2S_FLAG_TXE) == RESET)
  {
    if((ICM20602Timeout--) == 0) return ICM20602_TIMEOUT_UserCallback();
  }
  
  /* Send a Byte through the SPI peripheral */
  SPI_SendData8(ICM20602_SPI, byte);
  
  /* Wait to receive a Byte */
  ICM20602Timeout = ICM20602_FLAG_TIMEOUT;
  while (SPI_I2S_GetFlagStatus(ICM20602_SPI, SPI_I2S_FLAG_RXNE) == RESET)
  {
    if((ICM20602Timeout--) == 0) return ICM20602_TIMEOUT_UserCallback();
  }
  
  /* Return the Byte read from the SPI bus */
  return (uint8_t)SPI_ReceiveData8(ICM20602_SPI);
}

/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t ICM20602_TIMEOUT_UserCallback(void)
{
  return 0;
}



/******************* (C) COPYRIGHT 2017 PENG ZHANG *****END OF FILE********/

