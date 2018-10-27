/*
 *********************************************************************************************************
 * @file    imsf_math_misc.cpp
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @brief   This file provides some basic math functions and class 
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
#include "imsf_math_misc.h"
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

const int kMatrixVectorMaxDim = 15;
const double kPI  = 3.1415926535897932;
const double kEPS = 2.220446049e-16;
const double kINF = 3.402823466e+30;
const Unit unit;


Unit::Unit(void)
{
    Re = kEarthSemiMajorRadius;
    wie = kEarthSpinRate;
    g = kGravityForce
    mg = g/1000.0;
    ug = mg/1000.0;
    deg = kPI/180.0;
    min = deg/60.0;
    sec = min/60.0;
    ppm = 1.0e-6;
    hour = 3600.0;
    dps = deg/1.0;
    dph = deg/hour;
    dpsh = deg/sqrt(hour);
    dphpsh = dph/sqrt(hour);
    ugpsHz = ug/sqrt(1.0);
    ugpsh = ug/sqrt(hour);
    mpsh = 1/sqrt(hour); 
    mpspsh = 1/1/sqrt(hour);
    ppmpsh = ppm/sqrt(hour);
    secpsh = sec/sqrt(hour);
}

inline double DegToRad(double deg)
{
    return (deg / 180.0 * kPI);
}

inline double RadToDeg(double rad)
{
    return (rad * 180.0 / kPI);
}

/*
 * @breif  Constrain the input double x between min and max
 * @param  x, min and max
 * @note   This function can't detect parameter min < max case
 * @retval Calculation result
 */
double Constraind(double x, double min, double max)
{
    if (isnan(x))
    {
        return ((min + max) / 2.0f);
    }
    
    if (x > max)
    {
        return max;
    }
    else if (x < min)
    {
        return min;
    }
    
    return x;
}

/*
 * @breif  [0, 2pi] -> [-pi, pi]
 * @param  x, min and max
 * @note   None
 * @retval Calculation result
 */
double WrapRad(double rad)
{
    if (rad > kPI)
    {
        rad -= 2*kPI;
    }
    else if (rad < -kPI)
    {
        rad += 2*kPI;
    }

    return rad;
}

/*
 * @breif  [0, 360] -> [-180, 180]
 * @param  x, min and max
 * @note   None
 * @retval Calculation result
 */
double WrapDeg(double deg)
{
    if (deg > 180.0f)
    {
        deg -= 360.0f;
    }
    else if (deg < -180.0f)
    {
        deg += 360.0f;
    }
    
    return deg;
}

/*
 * @brief  linear interpolation based on a variable in a range
 * @param  x - input x value
 * @param  [x0, y0], [x1, y1] - two points in the coordinate system
 * @note   None
 * @retval Linear interpolation result
 */
double LinearInterpolate(double x, double x0, double x1, double y0, double y1)
{
    if (x <= x0)
    {
        return y0;
    }
    if (x >= x1)
    {
        return y1;
    }
    /* calculate the slope */
    double slope = (x - x0) / (x1 - x0);
    return (y0 + slope * (y1 - y0));
}


} /* namspace imsf */
/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/


