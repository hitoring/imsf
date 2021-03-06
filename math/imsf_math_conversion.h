/*
 *********************************************************************************************************
 * @file    imsf_math_conversion.h
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @brief   This file provides some conversion functions of rotation representation
            for intelligent multi-sensor fusion library.
 *********************************************************************************************************
 */

/* Define to prevent recursive inclusion ----------------------------------------------------------------*/
#ifndef IMSF_MATH_CONVERSION_H_
#define IMSF_MATH_CONVERSION_H_

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

Matrix3d AngleToDCM(const Vector3d& angle);
Quaterniond AngleToQuat(const Vector3d& angle);
Vector3d DCMToAngle(const Matrix3d& Cnb);
Quaterniond DCMToQuat(const Matrix3d& Cnb);
Vector3d QuatToAngle(const Quaterniond& qnb);
Matrix3d QuatToDCM(const Quaterniond& qnb);
Vector3d QuatToRV(const Quaterniond& qnb);
Quaterniond RVToQuat(const Vector3d& rv);
Matrix3d DoubleVectorToDCM(const Vector3d& va1, const Vector3d& va2, const Vector3d& vb1, const Vector3d& vb2);
Matrix3d PosToCen(const Vector3d& pos);


} /* namespace imsf */

#endif /* IMSF_MATH_CONVERSION_H_ */

/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/


