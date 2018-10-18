/*
 *********************************************************************************************************
 * @file    imsf_initial_align.cpp
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @note    This file is based on PSINS library written by Prof. Gongmin Yan @ NWPU, 
            great thanks and respect to Prof. Yan!
 * @brief   This file defines some member functions of InitialAlign class
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
#include "imsf_initial_align.h"
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


InitialAlign::InitialAlign(void)
{
    earth_param_ = EarthParam();
    imu_ = IMU();

    time_ = 0.0;
	t0 = t1 = t2 = 0; 
	delta_angle_sum_.SetZero();
    delta_vel_sum_.SetZero();
    vib0.SetZero();
    vi0.SetZero();
    Pib01.SetZero();
    Pib02.SetZero();
    Pi01.SetZero();
    Pi02.SetZero();
	qib0b.SetIdentity();
}

InitialAlign::~InitialAlign()
{
}

void InitialAlign::Init(const Vector3d& pos, const Vector3d& vn, const int subsamples, const double nts)
{
    earth_param_.Init();
    nts_ = nts;
    double sub_ts = nts_ / subsamples;
    imu_.Init(subsamples, sub_ts);
	earth_param_.Update(pos, vn);

    time_ = 0.0;
	t0 = t1 = 10; 
}

Quaterniond InitialAlign::Update(const Vector3d* delta_angle, const Vector3d* delta_vel)
{
	imu_.Update(delta_angle, delta_vel);
	delta_angle_sum_ += imu_.phim_;  
    delta_vel_sum_ += imu_.dvbm_;
	// vtmp = qib0b * (delta_vel + 1/2 * delta_angle X delta_vel)
	Vector3d vtmp = qib0b*imu_.dvbm_;
	// vtmp1 = qni0' * [dvn+(wnin+wnie)Xvn-gn] * ts;
    time_ += nts_;
    Matrix3d Ci0n = PosToCen(Vector3d(earth_param_.pos_.x, earth_param_.wie*time_, 0.0));
	Vector3d vtmp1 = Ci0n*(-earth_param_.gn*nts_);
	// Pib02 = Pib02 + vib0*ts, Pi02 = Pi02 + vi0*ts
	vib0 = vib0 + vtmp,		 vi0 = vi0 + vtmp1;
	Pib02 = Pib02 + vib0*nts_, Pi02 = Pi02 + vi0*nts_;
	//
	if(++t2>3*t0)
	{
		t0 = t1, Pib01 = tmpPib0, Pi01 = tmpPi0;
	}
	else if(t2>2*t0 && t1==t0)
	{
		t1 = t2, tmpPib0 = Pib02, tmpPi0 = Pi02;
	}
	//
	qib0b = qib0b*RVToQuat(imu_.phim_);
	// qnb=qni0*qiib0*qib0b
	Quaterniond qnb;
	if(t2<100)
	{
		qnb.SetIdentity();
	}
	else if(t2<1000)
	{
		qnb = AngleToQuat(AlignCoarse(delta_angle_sum_, delta_vel_sum_, earth_param_.pos_.x));
	}
	else
	{
		qnb = (~(DCMToQuat(Ci0n)))*DCMToQuat(DoubleVectorToDCM(Pi01, Pi02, Pib01, Pib02))*qib0b;
	}
	return qnb;
}

Vector3d AlignCoarse(Vector3d delta_angle_sum, Vector3d delta_vel_sum, double latitude)
{
	double T11, T12, T13, T21, T22, T23, T31, T32, T33;
	double cl = cos(latitude), tl = tan(latitude), nn;
	Vector3d wbib = delta_angle_sum / delta_angle_sum.GetNorm();
    Vector3d fb = delta_vel_sum / delta_vel_sum.GetNorm();
	T31 = fb.x,				 T32 = fb.y,			 	T33 = fb.z;
	T21 = wbib.x/cl-T31*tl,	 T22 = wbib.y/cl-T32*tl,	T23 = wbib.z/cl-T33*tl;		nn = sqrt(T21*T21+T22*T22+T23*T23);  T21 /= nn, T22 /= nn, T23 /= nn;
	T11 = T22*T33-T23*T32,	 T12 = T23*T31-T21*T33,		T13 = T21*T32-T22*T31;		nn = sqrt(T11*T11+T12*T12+T13*T13);  T11 /= nn, T12 /= nn, T13 /= nn;
	Matrix3d Cnb(T11, T12, T13, T21, T22, T23, T31, T32, T33);
	return DCMToAngle(Cnb);
}

} /* namspace imsf */
/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/


