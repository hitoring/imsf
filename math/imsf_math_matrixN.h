/*
 *********************************************************************************************************
 * @file    imsf_math_matrixN.h
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @note    This file is based on PSINS library written by Prof. Gongmin Yan @ NWPU, 
            great thanks and respect to Prof. Yan!
 * @brief   This file defines a NxN double matrix 
            for intelligent multi-sensor fusion library.
 *********************************************************************************************************
 */

/* Define to prevent recursive inclusion ----------------------------------------------------------------*/
#ifndef IMSF_MATH_MATRIXN_H_
#define IMSF_MATH_MATRIXN_H_

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
class MatrixNd
{
public:
    /* Member variables */
    int row_;
    int clm_;
    double data_[kMatrixVectorMaxDim][kMatrixVectorMaxDim];

    /* Constructors */
    MatrixNd(void);
    MatrixNd(int row, int clm);
    MatrixNd(int row, int clm, double s);

    /* Destructor */
    ~MatrixNd();

    /* Member functions */
    MatrixNd operator+(const MatrixNd& m) const;
    MatrixNd operator+(const VectorNd& v) const;
    MatrixNd operator+(double s) const;
    MatrixNd operator-(const MatrixNd& m) const;
    MatrixNd operator-(const VectorNd& v) const;
    MatrixNd operator-(double s) const;
    MatrixNd operator*(const MatrixNd& m) const;
    VectorNd operator*(const VectorNd& v) const;
    MatrixNd operator*(double s) const;
    MatrixNd operator/(double s) const;
    MatrixNd& operator+=(const MatrixNd& m);
    MatrixNd& operator+=(const VectorNd& v);
    MatrixNd& operator+=(double s);
    MatrixNd& operator-=(const MatrixNd& m);
    MatrixNd& operator-=(const VectorNd& v);
    MatrixNd& operator-=(double s);
    MatrixNd& operator*=(double s);
    MatrixNd& operator/=(double s);
    double& operator()(int row, int clm);
    void Symmetry(void);
    void SetRow(int i, const VectorNd& v);
    void SetClm(int j, const VectorNd& v);
    void SetMatrix3d(int i, int j, const Matrix3d& m);
    VectorNd GetRow(int i) const;
    VectorNd GetClm(int j) const;
    Matrix3d GetMatrix3d(int i, int j) const;
    void ZeroRow(int i);
    void ZeroClm(int j);
    void SetZero(void);
    void SetIdentity(void);
    void SetDiag(const VectorNd& v);
    VectorNd GetDiag(void) const;

    /* Friend functions */
    friend MatrixNd operator~(const MatrixNd& m);
    friend MatrixNd operator-(const MatrixNd& m);
};

namespace MatrixNd_ {

/* Outside class functions */
MatrixNd Zero(int row, int clm);
MatrixNd Identity(int dim);
VectorNd DiagVector(const MatrixNd m);
MatrixNd DiagMatrix(const VectorNd& v);

} /* namespace MatrixNd_ */

} /* namespace imsf */

#endif /* IMSF_MATH_MATRIXN_H_ */

/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/

