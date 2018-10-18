/*
 ******************************************************************************
 * @file    second_order_butterworth_lpf.c
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-08-12
 * @brief   This file provides some routine for second order butterworth low pass filter.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "second_order_butterworth_lpf.h"

/* Private function prototypes -----------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*
 * @brief  Second order butterworth low pass filter initialization
 * @param  sample_freq - sample frequency
 * @param  cutoff_freq - cut-off frequency
 * @param  filter - filter data structure, include the filter coefficient
 * @note   [cutoff_freq > 0] 
 * @retval None
 */
void second_order_butterworth_lpf_init(float sample_freq, float cutoff_freq, second_order_butterworth_lpf_t* filter)
{
	filter->fc = cutoff_freq;
	if (cutoff_freq <= 0.0f)
	{
		return;
	}
	float fr = sample_freq / cutoff_freq;
	float ohm = tan(C_PI / fr);
	float c = 1.0f + 2.0f * cos(C_PI / 4.0f) * ohm + ohm * ohm;
	
	filter->b0 = ohm * ohm / c;
	filter->b1 = 2.0f * filter->b0;
	filter->b2 = filter->b0;
	filter->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
	filter->a2 = (1.0f - 2.0f * cos(C_PI / 4.0f) * ohm + ohm * ohm) / c;

	filter->buf_data_1 = 0.0f;
	filter->buf_data_2 = 0.0f;
}

/*
 * @brief  Second order butterworth low pass filter application
 * @param  sample - data sample which to be filtered
 * @param  filter - filter data structure, include the filter coefficient
 * @note   [cutoff_freq > 0] 
 * @retval Filtered data
 */
float second_order_butterworth_lpf_apply(float sample, second_order_butterworth_lpf_t* filter)
{
	if (filter->fc <= 0.0f) 
	{
		return sample; // no filtering
	}

	/* apply the filtering */
	float buf_data_0 = sample - filter->buf_data_1 * filter->a1 - filter->buf_data_2 * filter->a2;
	float output = buf_data_0 * filter->b0 + filter->buf_data_1 * filter->b1 + filter->buf_data_2 * filter->b2;
	
	filter->buf_data_2 = filter->buf_data_1;
	filter->buf_data_1 = buf_data_0;

	return output;
}


/******************* (C) COPYRIGHT 2017 PENG ZHANG *****END OF FILE********/

