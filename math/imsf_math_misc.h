/*
 *********************************************************************************************************
 * @file    imsf_math_misc.h
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @brief   This file provides some basic math functions and class
            for intelligent multi-sensor fusion library.
 *********************************************************************************************************
 */

/* Define to prevent recursive inclusion ----------------------------------------------------------------*/
#ifndef IMSF_MATH_MISC_H_
#define IMSF_MATH_MISC_H_

/*
 *********************************************************************************************************
 *                                          INCLUDE HEADER FILES
 *********************************************************************************************************
 */
/* Include standard library header files ----------------------------------------------------------------*/
/* Include local library header files -------------------------------------------------------------------*/
#include "imsf_math.h"
/*
 *********************************************************************************************************
 *                                          NAMESPACE DEFINITION
 *********************************************************************************************************
 */
namespace imsf {
/*
 *********************************************************************************************************
 *                                          CLASS DEFINITION
 *********************************************************************************************************
 */
 
class Unit
{
public:
    double Re;
    double wie;
    double g;
	double mg;
    double ug;
    double deg;
    double min;
    double sec;
    double hour;
    double ppm;
    double ppmpsh;
	double dps;
    double dph;
    double dpsh;
    double dphpsh;
    double ugpsh;
    double ugpsHz;
    double mpsh;
    double mpspsh;
    double secpsh;

	Unit(void);
};


extern const int kMatrixVectorMaxDim;
extern const double kPI;
extern const double kEPS;
extern const double kINF;
extern const Unit unit;


inline double DegToRad(double deg);
inline double RadToDeg(double rad);
double Constraind(double x, double min, double max);
double WrapRad(double rad);
double WrapDeg(double deg);
double LinearInterpolate(double x, double x0, double x1, double y0, double y1);







} /* namespace imsf */

#endif /* IMSF_MATH_MISC_H_ */

/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/



