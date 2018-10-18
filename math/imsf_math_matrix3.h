/*
 *********************************************************************************************************
 * @file    imsf_math_matrix3.h
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @note    This file is based on PSINS library written by Prof. Gongmin Yan @ NWPU, 
            great thanks and respect to Prof. Yan!
 * @brief   This file defines a 3x3 double matrix class 
            for intelligent multi-sensor fusion library.
 *********************************************************************************************************
 */

/* Define to prevent recursive inclusion ----------------------------------------------------------------*/
#ifndef IMSF_MATH_MATRIX3_H_
#define IMSF_MATH_MATRIX3_H_

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

class Matrix3d 
{
public:
	/* Member variables */
	Vector3d a; // first row of the 3x3 double matrix
	Vector3d b; // second row of the 3x3 double matrix
	Vector3d c; // third row of the 3x3 double matrix

	/* Constructors */
	Matrix3d(void);
	Matrix3d(const double s);
	Matrix3d(double ax, double ay, double az, 
		        double bx, double by, double bz,
		        double cx, double cy, double cz);
	Matrix3d(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2);
	Matrix3d(const double* a0, const double* a1, const double* a2);
	Matrix3d(const double (*array)[3]);

	/* Destructor */
	~Matrix3d();

	/* Member functions */
	Matrix3d operator+(const Matrix3d& m) const;
	Matrix3d operator+(double s) const;
	Matrix3d operator-(const Matrix3d& m) const;
	Matrix3d operator-(double s) const;
	Matrix3d operator*(const Matrix3d& m) const;
	Vector3d operator*(const Vector3d& v) const;
	Matrix3d operator*(double s) const;
	Matrix3d operator/(double s) const;
	Matrix3d& operator+=(const Matrix3d& m);
	Matrix3d& operator+=(double s);
	Matrix3d& operator-=(const Matrix3d& m);
	Matrix3d& operator-=(double s);
	Matrix3d& operator*=(double s);
	Matrix3d& operator/=(double s);
	Vector3d GetRow(int i);
	Vector3d GetClm(int j);
	double operator()(int i, int j);
    void SetZero(void);
    void SetIdentity(void);
    Vector3d GetDiag(void) const;
    void SetDiag(const Vector3d& v);
    void SetAskew(const Vector3d& v);

	/* Friend functions */
	Matrix3d operator-(const Matrix3d& m);
	Matrix3d operator~(const Matrix3d& m);
};

namespace Matrix3d_ {

/* Outside class functions */
Matrix3d SetClms(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2);
Matrix3d Zero(void);
Matrix3d Identity(void);
Vector3d DiagVector(const Matrix3d& m);
Matrix3d DiagMatrix(const Vector3d& v);
double Det(const Matrix3d& m);
Matrix3d Inv(const Matrix3d& m);
Matrix3d Askew(const Vector3d& v);


} /* namespace Matrix3d_ */


} /* namespace imsf */

#endif /* IMSF_MATH_MATRIX3_H_ */

/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/


