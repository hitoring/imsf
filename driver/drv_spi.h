/*
 ******************************************************************************
 * @file    drv_spi.h
 * @author  Shi Daolin
 * @version V1.0.0
 * @date    2017-6-8
 * @brief   This file provides a driver for GCD spi
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRV_SPI_H__
#define __DRV_SPI_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f30x.h"
#include "stm32f30x_spi.h"
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/     
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/     
#define ICM20602_FLAG_TIMEOUT             ((uint32_t)0x1000)
     
/*ICM20602 SPI Interface pins
    CSn   --------  PA15 (14) ------   SPI1_NSS
    CLK   --------  PB3 (39)  ------   SPI1_SCK
    MISO  --------  PB4 (40)  ------   SPI1_MISO
    MOSI  --------  PB5 (41)  ------   SPI1_MOSI
*/   
#define ICM20602_SPI                       SPI1
#define ICM20602_SPI_CLK                   RCC_APB2Periph_SPI1

#define ICM20602_SPI_SCK_PIN               GPIO_Pin_3                  /* PB.04 */
#define ICM20602_SPI_SCK_GPIO_PORT         GPIOB                       /* GPIOB */
#define ICM20602_SPI_SCK_GPIO_CLK          RCC_AHBPeriph_GPIOB
#define ICM20602_SPI_SCK_SOURCE            GPIO_PinSource3
#define ICM20602_SPI_SCK_AF                GPIO_AF_5

#define ICM20602_SPI_MISO_PIN              GPIO_Pin_4                  /* PB.4 */
#define ICM20602_SPI_MISO_GPIO_PORT        GPIOB                       /* GPIOB */
#define ICM20602_SPI_MISO_GPIO_CLK         RCC_AHBPeriph_GPIOB
#define ICM20602_SPI_MISO_SOURCE           GPIO_PinSource4
#define ICM20602_SPI_MISO_AF               GPIO_AF_5

#define ICM20602_SPI_MOSI_PIN              GPIO_Pin_5                  /* PB.5 */
#define ICM20602_SPI_MOSI_GPIO_PORT        GPIOB                       /* GPIOB */
#define ICM20602_SPI_MOSI_GPIO_CLK         RCC_AHBPeriph_GPIOB
#define ICM20602_SPI_MOSI_SOURCE           GPIO_PinSource5
#define ICM20602_SPI_MOSI_AF               GPIO_AF_5

#define ICM20602_SPI_CS_PIN                GPIO_Pin_15                  /* PA.15 */
#define ICM20602_SPI_CS_GPIO_PORT          GPIOA                        /* GPIOA */
#define ICM20602_SPI_CS_GPIO_CLK           RCC_AHBPeriph_GPIOA

/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void SPI1_Init(void);
uint8_t SPI1_Read_Write_Byte(uint8_t byte);

#ifdef __cplusplus
}
#endif



#endif


/******************* (C) COPYRIGHT 2017 PENG ZHANG *****END OF FILE********/

