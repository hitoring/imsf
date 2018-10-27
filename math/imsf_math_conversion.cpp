/*
 *********************************************************************************************************
 * @file    imsf_math_conversion.cpp
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @brief   This file provides some conversion functions of rotation representation
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
#include "imsf_math_conversion.h"
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

Matrix3d AngleToDCM(const Vector3d& angle)
{
    Matrix3d Cnb;
    
    double sr = std::sin(angle.x), sp = std::sin(angle.y), sy = std::sin(angle.z); 
    double cr = std::cos(angle.x), cp = std::cos(angle.y), cy = std::cos(angle.z);

    Cnb.a.x = cp*cy; Cnb.a.y = -cr*sy+sr*sp*cy; Cnb.a.z = sr*sy;
    Cnb.b.x = cp*sy; Cnb.b.y = cr*cy+sr*sp*sy;  Cnb.b.z = -sr*cy+cr*sp*sy;
    Cnb.c.x = -sp;   Cnb.c.y = sr*cp;           Cnb.c.z = cr*cp;

    return Cnb;
}

Quaterniond AngleToQuat(const Vector3d& angle)
{   
    Quaterniond qnb;
    
    double sr = std::sin(angle.x/2), sp = std::sin(angle.y/2), sy = std::sin(angle.z/2); 
    double cr = std::cos(angle.x/2), cp = std::cos(angle.y/2), cy = std::cos(angle.z/2);

    qnb.q0 = cr*cp*cy - sr*sp*sy;
    qnb.q1 = sr*cp*cy - cr*sp*sy;
    qnb.q2 = cr*sp*cy + sr*cp*sy;
    qnb.q3 = cr*cp*sy + sr*sp*cy;

    return qnb;
    
}

Vector3d DCMToAngle(const Matrix3d& Cnb)
{
    Vector3d angle;
    
    angle.x = std::atan2(Cnb.c.y, Cnb.c.z);
    angle.y = std::asin(Constraind(-Cnb.c.x, -1.0, 1.0)); // bound the input of asin between [-1,1]
    angle.z = std::atan2(Cnb.b.x, Cnb.a.x);

    
    return angle;
}

Quaterniond DCMToQuat(const Matrix3d& Cnb)
{
    Quaterniond qnb;
    double tmp;

    tmp = 1.0 + Cnb.a.x + Cnb.b.y + Cnb.c.z;
    qnb.q0 = std::sqrt(std::abs(tmp)) / 2.0;
    tmp = 1.0 + Cnb.a.x - Cnb.b.y - Cnb.c.z;
    qnb.q1 = std::sqrt(std::abs(tmp)) / 2.0;
    tmp = 1.0 - Cnb.a.x + Cnb.b.y - Cnb.c.z;
    qnb.q2 = std::sqrt(std::abs(tmp)) / 2.0;
    tmp = 1.0 - Cnb.a.x - Cnb.b.y + Cnb.c.z;
    qnb.q3 = std::sqrt(std::abs(tmp)) / 2.0;

    /* sign decision */
    if ((Cnb.c.y - Cnb.b.z) < 0)
    {
        qnb.q1 = -qnb.q1;
    }
    if ((Cnb.a.z - Cnb.c.x) < 0)
    {
        qnb.q2 = -qnb.q2;
    }
    if ((Cnb.b.x - Cnb.a.y) < 0)
    {
        qnb.q3 = -qnb.q3;
    }

    /* normlize */
    qnb.Normlize();
    
    return qnb;
}

Vector3d QuatToAngle(const Quaterniond& qnb)
{
    Vector3d angle;

    double q00 = qnb.q0*qnb.q0, q01 = qnb.q0*qnb.q1, q02 = qnb.q0*qnb.q2, q03 = qnb.q0*qnb.q3;
    double q11 = qnb.q1*qnb.q1, q12 = qnb.q1*qnb.q2, q13 = qnb.q1*qnb.q3;
    double q22 = qnb.q2*qnb.q2, q23 = qnb.q2*qnb.q3;
    double q33 = qnb.q3*qnb.q3;

    double C11 = q00 + q11 - q22 - q33;
    double C21 = 2.0 * (q12 + q03);
    double C31 = 2.0 * (q13 - q02), C32 = 2.0 * (q23 + q01), C33 = q00 - q11 - q22 + q33;

    angle.x = std::atan2(C32, C33);
    angle.y = std::asin(Constraind(-C31, -1.0, 1.0)); // constrain the input of asin between [-1,1]
    angle.z = std::atan2(C21, C11);
    
    return angle;
}

Matrix3d QuatToDCM(const Quaterniond& qnb)
{
    Matrix3d Cnb;
    
    double q00 = qnb.q0*qnb.q0, q01 = qnb.q0*qnb.q1, q02 = qnb.q0*qnb.q2, q03 = qnb.q0*qnb.q3;
    double q11 = qnb.q1*qnb.q1, q12 = qnb.q1*qnb.q2, q13 = qnb.q1*qnb.q3;
    double q22 = qnb.q2*qnb.q2, q23 = qnb.q2*qnb.q3;
    double q33 = qnb.q3*qnb.q3;

    Cnb.a.x = q00 + q11 - q22 - q33; 
    Cnb.b.x = 2.0 * (q12 + q03);
    Cnb.c.x = 2.0 * (q13 - q02);
    Cnb.a.y = 2.0 * (q12 - q03); 
    Cnb.b.y = q00 - q11 + q22 - q33;
    Cnb.c.y = 2.0 * (q23 + q01);
    Cnb.a.z = 2.0 * (q13 + q02);
    Cnb.b.z = 2.0 * (q23 - q01);
    Cnb.c.z = q00 - q11 - q22 + q33;

    return Cnb;
}

Vector3d QuatToRV(const Quaterniond& qnb)
{   
    Vector3d rv;
    double n2, f;
    
    if (qnb.q0 < 0)
    {
        qnb.q0 = -qnb.q0;
        qnb.q1 = -qnb.q1;
        qnb.q2 = -qnb.q2;
        qnb.q3 = -qnb.q3;
    }
    if (qnb.q0 > 1.0)
    {
        qnb.q0 = 1.0;
    }
    
    n2 = std::acos(qnb.q0);
    if (n2 > 1.0e-20)
    {
        f = 2.0 * n2 / std::sin(n2);
    }
    else
    {
        f = 2.0;
    }

    rv.x = qnb.q1 * f;
    rv.y = qnb.q2 * f;  
    rv.z = qnb.q3 * f;

    return rv;
}

Quaterniond RVToQuat(const Vector3d& rv)
{
    /*Constants: Fk=2^k*k! */
    const int kF1 =       2 * 1;
    const int kF2 = kF1 * 2 * 2;
    const int kF3 = kF2 * 2 * 3;
    const int kF4 = kF3 * 2 * 4;
    const int kF5 = kF4 * 2 * 5;
    
    Quaterniond qnb;
    double norm, s;
    double norm_sq = rv.x*rv.x + rv.y*rv.y + rv.z*rv.z;
    
    if (norm_sq < 1.0e-8) // (PI/180.0*PI/180.0) from psins.cpp
    {
        qnb.q0 = 1.0 - norm_sq * (1.0/kF2- norm_sq/kF4);
        s = 0.5 - norm_sq * (1.0/kF3 - norm_sq/kF5);
    }
    else
    {
        norm = std::sqrt(norm_sq);
        qnb.q0 = std::cos(norm/2.0);
        s = std::sin(norm/2.0) / norm;
    }
    qnb.q1 = s * rv.x;
    qnb.q2 = s * rv.y;
    qnb.q3 = s * rv.z;

    return qnb;

}

Matrix3d DoubleVectorToDCM(const Vector3d& va1, const Vector3d& va2, const Vector3d& vb1, const Vector3d& vb2)
{
    Vector3d a=va1*va2, b=vb1*vb2, aa=a*va1, bb=b*vb1;
    Matrix3d Ma(va1/va1.GetNorm(), a/a.GetNorm(), aa/aa.GetNorm()), 
    Matrix3d Mb(vb1/vb1.GetNorm(), b/b.GetNorm(), bb/bb.GetNorm());
    return (~Ma)*(Mb);  //Cab
}

Matrix3d PosToCen(const Vector3d& pos)
{
    double si = sin(pos.x), ci = cos(pos.x), sj = sin(pos.y), cj = cos(pos.y);
    return Matrix3d(-sj, -si*cj,  ci*cj,  
                     cj, -si*sj,  ci*sj,  
                     0,   ci,     si    );  //Cen
}



} /* namspace imsf */
/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/


