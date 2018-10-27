/*
 *********************************************************************************************************
 * @file    imsf_math_vectorN.h
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @brief   This file defines a N-dimension double vector class 
            for intelligent multi-sensor fusion library.
 *********************************************************************************************************
 */

/* Define to prevent recursive inclusion ----------------------------------------------------------------*/
#ifndef IMSF_MATH_VECTORN_H_
#define IMSF_MATH_VECTORN_H_

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
class VectorNd
{
public:
    /* Member variables */
    int dim_;
    double data_[kMatrixVectorMaxDim];

    /* Constructors */
    VectorNd(void);
    VectorNd(int dim);
    VectorNd(int dim, double s);
    VectorNd(int dim, const double* array);

    /* Destructor */
    ~VectorNd();

    /* Member functions */
    VectorNd operator+(const VectorNd& v) const;
    VectorNd operator+(double s) const;
    VectorNd operator-(const VectorNd& v) const;
    VectorNd operator-(double s) const;
    VectorNd operator*(const MatrixNd& m) const;
    MatrixNd operator*(const VectorNd& v) const;
    VectorNd operator*(double s) const;
    VectorNd operator/(double s) const;
    VectorNd& operator+=(const VectorNd& v);
    VectorNd& operator+=(double s);
    VectorNd& operator-=(const VectorNd &v);
    VectorNd& operator-=(double s);
    VectorNd& operator*=(double s);
    VectorNd& operator/=(double s);
    double& operator()(int i) const;
    double Dot(const VectorNd& v);
    double GetNorm(void);
    int Normlize(void);
    Vector3d GetVector3d(const int i) const;
    void SetVector3d(const int i, const Vector3d& v);
    void SetZero(void);

    /* Friend functions */
    friend VectorNd operator-(const VectorNd& v);

};

namespace VectorNd_ {
/* Outside class functions */
VectorNd Zero(const int dim);

} /* namespace VectorNd_ */


} /* namespace imsf */

#endif /* IMSF_MATH_VECTORN_H_ */

/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/


