/*
 *********************************************************************************************************
 * @file    imsf_math_quaternion.h
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @brief   This file defines a double quaternion class 
            for intelligent multi-sensor fusion library.
 *********************************************************************************************************
 */

/* Define to prevent recursive inclusion ----------------------------------------------------------------*/
#ifndef IMSF_MATH_QUATERNION_H_
#define IMSF_MATH_QUATERNION_H_

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

class Quaterniond
{
public:
    /* Member variables */
	double q0, q1, q2, q3;

    /* Constructors */
	Quaterniond(void);
    Quaterniond(const double w, const double x, const double y, const double z);
    Quaterniond(const double* array);

    /* Destructor */
    ~Quaterniond();

    /* Member functions */
	Quaterniond operator+(const Vector3d& phi) const;	// true quaternion add misalign angles
    Quaterniond& operator+=(const Vector3d& phi);
    Quaterniond operator-(const Vector3d& phi) const;	// calculated quaternion delete misalign angles
    Quaterniond& operator-=(const Vector3d& phi);
    Vector3d operator-(Quaterniond& quat) const;
    Quaterniond operator*(const Quaterniond& quat) const; // quaternion multiplication
    Vector3d operator*(const Vector3d& v) const; // quaternion multiply vector
    Quaterniond& operator*=(const Quaterniond& quat);
    double GetNorm(void) const; 
    int Normlize(void); // quaternion norm
    void SetIdentity(void);
    void Update(const Vector3d& rv); // quaternion update

    /* Friend functions */
	friend Quaterniond operator~(const Quaterniond& quat);		// quaternion conjugate
};

namespace Quaterniond_ {
/* Outside class functions */
Quaterniond Identity(void);

} /* namespace Quaterniond_ */

} /* namespace imsf */

#endif /* IMSF_MATH_QUATERNION_H_ */

/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/


