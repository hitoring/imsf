/*
 ******************************************************************************
 * @file    second_order_butterworth_lpf.h
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-08-12
 * @brief   This file provides some routine for low pass filter.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SECOND_ORDER_BUTTERWORTH_LPF_H
#define __SECOND_ORDER_BUTTERWORTH_LPF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "math_misc.h"
/* Exported types ------------------------------------------------------------*/
typedef struct
{
	float b0;
	float b1;
	float b2;
	float a1;
	float a2;
	float buf_data_1;
	float buf_data_2;
	float fc;
}second_order_butterworth_lpf_t;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void second_order_butterworth_lpf_init(float sample_freq, float cutoff_freq, second_order_butterworth_lpf_t* filter);
float second_order_butterworth_lpf_apply(float sample, second_order_butterworth_lpf_t* filter);


#ifdef __cplusplus
}
#endif

#endif /* __SECOND_ORDER_BUTTERWORTH_LPF_H */


/******************* (C) COPYRIGHT 2017 PENG ZHANG *****END OF FILE********/

