/*
 *********************************************************************************************************
 * @file    imsf_sins_imu.cpp
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @note    This file is based on PSINS library written by Prof. Gongmin Yan @ NWPU, 
            great thanks and respect to Prof. Yan!
 * @brief   This file defines some member functions of IMU class
            for intelligent multi-sensor fusion library.
 *********************************************************************************************************
 */

/*
 *********************************************************************************************************
 *                                          INCLUDE HEADER FILES
 *********************************************************************************************************
 */
/* Include standard library header files ----------------------------------------------------------------*/
/* Include local library header files -------------------------------------------------------------------*/
#include "imsf_sins_imu.h"
/*
 *********************************************************************************************************
 *                                          NAMESPACE DEFINITION
 *********************************************************************************************************
 */
namespace imsf {
 
/*
 *********************************************************************************************************
 *                                       CLASS FUNCTIONS DEFINITION
 *********************************************************************************************************
 */

static const double kConeScullCoef[5][5] = {{2.0/3}, 								                               // 2
   								            {9.0/20, 27.0/20},							                           // 3
	                                        {54.0/105, 92.0/105, 214.0/105},				                       // 4
	                                        {250.0/504, 525.0/504, 650.0/504, 1375.0/504},                         // 5
										    {2315.0/4620, 4558.0/4620, 7296.0/4620, 7834.0/4620, 15797.0/4620}};   // 6


IMU::IMU(void)
{
    updated_ = 0;
	subsamples_ = 1;
	sub_ts_ = 0.0;
    time_ = 0.0;
	
	phim_.SetZero();
	dvbm_.SetZero();
	delta_angle_prev_.SetZero();
	delta_vel_prev_.SetZero();
	dphim_.SetZero();
	rotm_.SetZero();
	scullm_.SetZero();
}

IMU::~IMU()
{
}

void IMU::Init(const int subsamples, const double sub_ts)
{
    subsamples_ = subsamples;
    sub_ts_ = sub_ts;
}
void IMU::Update(const Vector3d* delta_angle, const Vector3d* delta_vel)
{
	Vector3d delta_angle_sum(0.0);
	Vector3d delta_vel_sum(0.0);
	Vector3d delta_angle_coef(0.0);
	Vector3d delta_vel_coef(0.0);
	
	if (1 == subsamples_)  // one-plus-previous sample
	{
		delta_angle_sum = delta_angle[0];
		delta_vel_sum = delta_vel[0];

		/* Coning compensation */
		dphim_ = delta_angle_prev_.Cross(delta_angle[0]) / 12;
		/* Sculling compensation */
		scullm_ = (delta_angle_prev_.Cross(delta_vel[0]) + delta_vel_prev_.Cross(delta_angle[0])) / 12;

		delta_angle_prev_ = delta_angle[0];
		delta_vel_prev_ = delta_vel[0];
	}
	else if (subsamples_ > 1)
	{
		for(int i = 0; i < (subsamples_-1); ++i)
		{
			delta_angle_sum += delta_angle[i];
			delta_vel_sum += delta_vel[i];
			delta_angle_coef += kConeScullCoef[subsamples_-2][i] * delta_angle[i];
			delta_vel_coef += kConeScullCoef[subsamples_-2][i] * delta_vel[i];
		}
		
		delta_angle_sum += delta_angle[subsamples_-1];
		delta_vel_sum += delta_vel[subsamples_-1];
		/* Coning compensation */
		dphim_ = delta_angle_coef.Cross(delta_angle[subsamples_-1]);
		/* Sculling compensation */
		scullm_ = delta_angle_coef.Cross(delta_vel[subsamples_-1]) + delta_vel_coef.Cross(delta_angle[subsamples_-1]);
	}
	
	phim_ = delta_angle_sum + dphim_;
	rotm_ = delta_angle_sum.Cross(delta_vel_sum) / 2;
	dvbm_ = delta_vel_sum + rotm_ + scullm_;
}


} /* namspace imsf */
/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/


