/*
 *********************************************************************************************************
 * @file    imsf_sins_earth.cpp
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @brief   This file defines some member functions of EarthParam class
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
#include "imsf_sins_earth.h"
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

const double kEarthMeanRadius = 6371000.0; 
const double kEarthSemiMajorRadius = 6378137.0;
const double kEarthFlattening = 1 / 298.257;
const double kEarthSpinRate = 7.2921151467e-5;
const double kGravityForce = 9.7803267714;

EarthParam::EarthParam(void)
{
	time_ = 0.0;
	ts_ = 0.0;
	
	Rm = 0.0;	
	Re = 0.0;
	f = 0.0;
	Rp = 0.0;
	e = 0.0;
	e2 = 0.0;
	ep = 0.0;
	ep2 = 0.0;
	wie = 0.0;
	g0 = 0.0;

    
	sl = 0.0;
	sl2 = 0.0;
	sl4 = 0.0;
	cl = 0.0;
	tl = 0.0;
	RMh = 0.0; 
	RNh = 0.0; 
	clRNh = 0.0;
	f_RMh = kINF;
	f_RNh = kINF;
	f_clRNh = kINF;
	
	gn.SetZero();
	gcc.SetZero();
	wnie.SetZero();
	wnen.SetZero();
	wnin.SetZero();
    pos_.SetZero();
    vn_.SetZero();
}

EarthParam::~EarthParam()
{
}

void EarthParam::Init(void)
{
	Rm = kEarthMeanRadius;	
	Re = kEarthSemiMajorRadius;
	f  = kEarthFlattening;
	Rp = (1 - f) * Re;
	e = sqrt(2*f - f*f);
	e2 = e * e;
	ep = sqrt(Re*Re - Rp*Rp) / Rp;
	ep2 = ep * ep;
	wie = kEarthSpinRate;
	g0 = kGravityForce;
	
	gn = Vector3d(0.0, 0.0, -g0);
}

void EarthParam::Update(const Vector3d& pos, const Vector3d& vn)
{
#ifdef USE_LOW_GRADE_MEMS_IMU
    pos_ = pos;
    vn_ = vn;
	sl = sin(pos.x);
	sl2 = sl*sl;
	cl = cos(pos.x);
	tl = sl/cl;
	double sq = 1 - e2*sl*sl;
	double sq2 = sqrt(sq);
	RMh = Re*(1-e2)/sq/sq2 + pos.z;	
	f_RMh = 1 / RMh;
	RNh = Re/sq2 + pos.z;    
	clRNh = cl * RNh;  
	f_RNh = 1 / RNh; 
	f_clRNh = 1 / clRNh;
	wnin = wnie = wnen = Vector3d(0.0, 0.0, 0.0);
	gn.z = -(g0*(1 + 5.27094e-3*sl2) - 3.086e-6*pos.z);
	gcc = gn;
#else
    pos_ = pos;
    vn_ = vn;
	sl = sin(pos.x); 
	sl2 = sl*sl;
	sl4 = sl2*sl2;
	cl = cos(pos.x); 
	tl = sl/cl;
	double sq = 1-e2*sl*sl;
	double sq2 = sqrt(sq);
	RMh = Re*(1-e2)/sq/sq2 + pos.z;	
	f_RMh = 1.0/RMh;
	RNh = Re/sq2 + pos.z;    
	clRNh = cl*RNh;  
	f_RNh = 1.0/RNh; 
	f_clRNh = 1.0/clRNh;
	wnie.x = 0.0,			wnie.y = wie*cl,		wnie.z = wie*sl;
	wnen.x = -vn.y*f_RMh,	wnen.y = vn.x*f_RNh,	wnen.z = wnen.y*tl;
	wnin = wnie + wnen;
	gn.z = -(g0*(1 + 5.27094e-3*sl2 + 2.32718e-5*sl4) - 3.086e-6*pos.z);
	gcc = gn - (wnie + wnin)*vn;
	
#endif
}

Vector3d EarthParam::VelToDeltaPos(const Vector3d& vn, const double ts) const
{
	return Vector3d(vn.y*f_RMh, vn.x*f_clRNh, vn.z)*ts;
}


} /* namspace imsf */
/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/


