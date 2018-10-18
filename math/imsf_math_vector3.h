/*
 *********************************************************************************************************
 * @file    imsf_math_vector3.h
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @brief   This file defines a 3-dimension double vector class 
            for intelligent multi-sensor fusion library.
 *********************************************************************************************************
 */

/* Define to prevent recursive inclusion ----------------------------------------------------------------*/
#ifndef IMSF_MATH_VECTOR3_H_
#define IMSF_MATH_VECTOR3_H_

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

class Vector3d 
{
public:
	/* Member variables */
	double x; // first element of the 3-dimension double vector
	double y; // second element of the 3-dimension double vector
	double z; // third element of the 3-dimension double vector

	/* Constructors */
	Vector3d(void);
	Vector3d(const double s);
	Vector3d(double x0, double y0, double z0);
	Vector3d(const double *array);

	/* Destructor */
	~Vector3d();

	/* Member functions */
	Vector3d operator+(const Vector3d& v) const; 
	Vector3d operator+(double s) const;
	Vector3d operator-(const Vector3d& v) const;
	Vector3d operator-(double s) const;
	Vector3d operator*(const Matrix3d& m) const;
	Matrix3d operator*(const Vector3d& v) const;
	Vector3d operator*(double s) const;
	Vector3d operator/(double s) const;
	Vector3d& operator+=(const Vector3d& v);
	Vector3d& operator+=(double s);
	Vector3d& operator-=(const Vector3d& v);
	Vector3d& operator-=(double s);
	Vector3d& operator*=(double s);
	Vector3d& operator/=(double s);
	double operator()(int i);
	double Dot(const Vector3d& v);
	Vector3d Cross(const Vector3d& v);
	double GetNorm(void) const;
	int Normlize(void);
    void SetZero(void);
    void SetInf(void);

	/* Friend funstions */
	friend Vector3d operator-(const Vector3d& v);
};

namespace Vector3d_ {

/* Outside class functions */
Vector3d Zero(void);

}


} /* namespace imsf */

#endif /* IMSF_MATH_VECTOR3_H_ */

/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/


