/*
 *********************************************************************************************************
 * @file    imsf_math_quaternion.cpp
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @brief   This file defines some member functions of Quaterniond class 
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
#include "imsf_math_quaternion.h"
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

Quaterniond::Quaterniond(void)
{
    q0 = 1.0;
    q1 = 0.0;
    q2 = 0.0;
    q3 = 0.0;
}

Quaterniond::Quaterniond(const double w, const double x, const double y, const double z)
{
    q0 = w;
    q1 = x;
    q2 = y;
    q3 = z;
}

Quaterniond::Quaterniond(const double* array)
{
    q0 = array[0];
    q1 = array[1];
    q2 = array[2];
    q3 = array[3];
}

Quaterniond::~Quaterniond()
{
}


/* qpb = qnb + phi */
Quaterniond Quaterniond::operator+(const Vector3d& phi) const
{   
    Quaterniond qpb;
    Quaterniond dq;
    
    dq = RVToQuat(-phi);
    qpb = dq * (*this); // qnb = *this
    
    return qpb;
}

Quaterniond& Quaterniond::operator+=(const Vector3d& phi)
{
    Quaterniond dq = RVToQuat(-phi);
    return (*this=dq*(*this));  
}

/* qnb = qpb - phi */
Quaterniond Quaterniond::operator-(const Vector3d& phi) const
{
    Quaterniond qnb;
    Quaterniond dq;
    
    dq = RVToQuat(phi);
    qnb = dq * (*this); // qpb = *this
    
    return qnb;
}

Quaterniond& Quaterniond::operator-=(const Vector3d& phi)
{
    Quaterniond dq = RVToQuat(phi);
    return (*this=dq*(*this));  
}

/* phi = qpb - qnb */
Vector3d Quaterniond::operator-(Quaterniond& quat) const
{
    Quaterniond dq;
    Vector3d phi;
    
    dq = quat * (~(*this));
    phi = QuatToRV(dq);

    return phi;
}

Quaterniond Quaterniond::operator*(const Quaterniond& quat) const
{
    Quaterniond q_tmp;
    
    q_tmp.q0 = q0*quat.q0 - q1*quat.q1 - q2*quat.q2 - q3*quat.q3;
    q_tmp.q1 = q0*quat.q1 + q1*quat.q0 + q2*quat.q3 - q3*quat.q2;
    q_tmp.q2 = q0*quat.q2 + q2*quat.q0 + q3*quat.q1 - q1*quat.q3;
    q_tmp.q3 = q0*quat.q3 + q3*quat.q0 + q1*quat.q2 - q2*quat.q1;
    
    return q_tmp;
}

Vector3d Quaterniond::operator*(const Vector3d& v) const
{
    Matrix3d Cnb = QuatToDCM(*this);
    return (Cnb*v);
}

Quaterniond& Quaterniond::operator*=(const Quaterniond& quat)
{
    return (*this=*this*quat);
}

double Quaterniond::GetNorm(void) const
{
    return std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
}

int Quaterniond::Normlize(void)
{
    double norm = GetNorm();
    assert(norm > 0);
    
    if (norm > 0)
    {
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;

        return 1;
    }
    else
    {
        return 0;
    }
}

void Quaterniond::SetIdentity(void)
{
    q0 = 1.0;
    q1 = 0.0;
    q2 = 0.0;
    q3 = 0.0;
}

void Quaterniond::Update(const Vector3d& rv)
{
    Quaterniond dq = RVToQuat(rv);
    *this = (*this)*dq;
    Normlize();
}


Quaterniond operator~(const Quaterniond& quat)
{
    return Quaterniond(quat.q0, -quat.q1, -quat.q2, -quat.q3);
}

namespace Quaterniond_ {

Quaterniond Identity(void)
{
    return Quaterniond(1.0, 0.0, 0.0, 0.0);
}

} /* namespace Quaterniond */

} /* namspace imsf */
/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/


