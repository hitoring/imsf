/*
 *********************************************************************************************************
 * @file    imsf_math_vector3.cpp
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @note    This file is based on PSINS library written by Prof. Gongmin Yan @ NWPU, 
            great thanks and respect to Prof. Yan!
 * @brief   This file defines some member functions of Vector3d class 
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
#include "imsf_math_vector3.h"
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

Vector3d::Vector3d(void)
{
	x = 0.0;
	y = 0.0;
	z = 0.0;
}

Vector3d::Vector3d(const double s)
{
	x = s;
	y = s;
	z = s;
}

Vector3d::Vector3d(const double x0, const double y0, const double z0)
{
	x = x0;
	y = y0;
	z = z0;
}

Vector3d::Vector3d(const double* array)
{
	x = array[0]; 
	y = array[1]; 
	z = array[2];
}

Vector3d::~Vector3d()
{
}

Vector3d Vector3d::operator+(const Vector3d& v) const
{
	return Vector3d(x+v.x, y+v.y, z+v.z);
}

Vector3d Vector3d::operator+(const double s) const
{
	return Vector3d(x+s, y+s, z+s);
}

Vector3d Vector3d::operator-(const Vector3d& v) const
{
	return Vector3d(x-v.x, y-v.y, z-v.z);
}

Vector3d Vector3d::operator-(const double s) const
{
	return Vector3d(x-s, y-s, z-s);
}

Vector3d Vector3d::operator*(const Matrix3d& m) const
{
	Vector3d v_tmp(0.0);

	v_tmp.x = x*m.a.x + y*m.b.x + z*m.c.x;
	v_tmp.y = x*m.a.y + y*m.b.y + z*m.c.y;
	v_tmp.z = x*m.a.z + y*m.b.z + z*m.c.z;

	return v_tmp;
}

Matrix3d Vector3d::operator*(const Vector3d& v) const
{
	Matrix3d m_tmp(0.0);

	m_tmp.a = Vector3d(x*v.x, x*v.y. x*v.z);
	m_tmp.b = Vector3d(y*v.x, y*v.y. y*v.z);
	m_tmp.c = Vector3d(z*v.x, z*v.y. z*v.z);

	return m_tmp;
}
	
Vector3d Vector3d::operator*(const double s) const
{
	return Vector3d(x*s, y*s, z*s);
}
	
Vector3d Vector3d::operator/(const double s) const
{
	return Vector3d(x/s, y/s, z/s);
}

Vector3d& Vector3d::operator+=(const Vector3d& v)
{ 
	x += v.x; 
	y += v.y; 
	z += v.z;
	
	return *this;
}

Vector3d& Vector3d::operator+=(const double s)
{ 
	x += s; 
	y += s; 
	z += s;
	
	return *this;
}

Vector3d& Vector3d::operator-=(const Vector3d& v)
{ 
	x -= v.x; 
	y -= v.y; 
	z -= v.z;
	
	return *this;
}

Vector3d& Vector3d::operator-=(const double s)
{ 
	x -= s; 
	y -= s; 
	z -= s;
	
	return *this;
}


Vector3d& Vector3d::operator*=(const double s)
{ 
	x *= s; 
	y *= s;
	z *= s;
	
	return *this;
}

Vector3d& Vector3d::operator/=(const double s)
{ 
	x /= s; 
	y /= s;
	z /= s;
	
	return *this;
}

double Vector3d::operator()(const int i) const
{
	assert((i >= 0) && (i < 3));
	
	if (0 == i)
	{
		return x;
	}
	else if (1 == i)
	{
		return y;
	}
	else if (2 == i)
	{
		return z;
	}
}

double Vector3d::Dot(const Vector3d& v)
{
	return (x*v.x + y*v.y + z*v.z);
}

Vector3d Vector3d::Cross(const Vector3d& v)
{
	Vector3d v_tmp(0.0);

	v_tmp.x = y*v.z - z*v.y;
	v_tmp.y = z*v.x - x*v.z;
	v_tmp.z = x*v.y - y*v.x;
	
	return v_tmp;
}

double Vector3d::GetNorm(void) const
{
	return sqrt(x*x + y*y + z*z);
}

int Vector3d::Normlize(void)
{
	double norm = GetNorm();
	assert(norm > 0);
	
	if (norm > 0)
	{
		x /= norm;
		y /= norm;
		z /= norm;

		return 1;
	}
	else
	{
		return 0;
	}
}

void Vector3d::SetZero(void)
{
	x = 0.0;
	y = 0.0;
	z = 0.0;
}

void Vector3d::SetInf(void)
{
	x = kINF;
	y = kINF;
	z = kINF;
}


Vector3d operator-(const Vector3d& v)
{
	return Vector3d(-v.x, -v.y, -v.z);
}

namespace Vector3d_ {
	
Vector3d Zero(void)
{
	return Vector3d(0.0, 0.0, 0.0);
}


}/* namspace Vector3d_ */

} /* namspace imsf */
/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/


