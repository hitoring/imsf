/*
 *********************************************************************************************************
 * @file    imsf_math_matrix3.cpp
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @note    This file is based on PSINS library written by Prof. Gongmin Yan @ NWPU, 
            great thanks and respect to Prof. Yan!
 * @brief   This file defines some member functions of Matrix3d class 
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
#include "imsf_math_matrix3.h"
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

Matrix3d::Matrix3d(void)
{
    a = Vector3d(0.0, 0.0, 0.0);
    b = Vector3d(0.0, 0.0, 0.0);
    c = Vector3d(0.0, 0.0, 0.0);
}

Matrix3d::Matrix3d(const double s)
{
    a = Vector3d(s);
    b = Vector3d(s);
    c = Vector3d(s);
}

Matrix3d::Matrix3d(const double ax, const double ay, const double az, 
                       const double bx, const double by, const double bz,
                       const double cx, const double cy, const double cz)
{
    a = Vector3d(ax, ay, az);
    b = Vector3d(bx, by, bz);
    c = Vector3d(cx, cy, cz);
}

Matrix3d::Matrix3d(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2)
{
    a = v0;
    b = v1;
    c = v2;
}

Matrix3d::Matrix3d(const double* a0, const double* a1, const double* a2)
{
    a = Vector3d(a0);
    b = Vector3d(a1);
    c = Vector3d(a2);
}

Matrix3d::Matrix3d(const double (*array)[3])
{
    a = Vector3d(array[0]);
    b = Vector3d(array[1]);
    c = Vector3d(array[2]);
}

Matrix3d::~Matrix3d()
{
}

Matrix3d Matrix3d::operator+(const Matrix3d& m) const
{
    return Matrix3d(a+m.a, b+m.b, c+m.c);
}

Matrix3d Matrix3d::operator+(const double s) const
{
    return Matrix3d(a+s, b+s, c+s);
}

Matrix3d Matrix3d::operator-(const Matrix3d& m) const
{
    return Matrix3d(a-m.a, b-m.b, c-m.c);
}

Matrix3d Matrix3d::operator-(const double s) const
{
    return Matrix3d(a-s, b-s, c-s);
}

Matrix3d Matrix3d::operator*(const Matrix3d& m) const
{
    return Matrix3d(a*m, b*m, c*m);
}

Vector3d Matrix3d::operator*(const Vector3d& v) const
{
    return Vector3d(a.Dot(v), b.Dot(v), c.Dot(v));
}

Matrix3d Matrix3d::operator*(const double s) const
{
    return Matrix3d(a*s, b*s, c*s);
}

Matrix3d Matrix3d::operator/(const double s) const
{
    return Matrix3d(a/s, b/s, c/s);
}

Matrix3d& Matrix3d::operator+=(const Matrix3d& m)
{
    a += m.a;
    b += m.b;
    c += m.c;
    
    return *this;
}

Matrix3d& Matrix3d::operator+=(const double s)
{
    a += s;
    b += s;
    c += s;
    
    return *this;
}

Matrix3d& Matrix3d::operator-=(const Matrix3d& m)
{
    a -= m.a;
    b -= m.b;
    c -= m.c;
    
    return *this;
}

Matrix3d& Matrix3d::operator-=(const double s)
{
    a -= s;
    b -= s;
    c -= s;
    
    return *this;
}

Matrix3d& Matrix3d::operator*=(const double s)
{
    a *= s;
    b *= s;
    c *= s;
    
    return *this;
}

Matrix3d& Matrix3d::operator/=(const double s)
{
    a /= s;
    b /= s;
    c /= s;
    
    return *this;
}

Vector3d Matrix3d::GetRow(const int i) const
{
    assert((i >= 0) && (i < 3));
    
    if (0 == i)
    {
        return a;
    }
    else if (1 == i)
    {
        return b;
    }
    else if (2 == i)
    {
        return c;
    }
}

Vector3d Matrix3d::GetClm(const int j) const
{
    assert((j >= 0) && (j < 3));
    
    if (0 == j)
    {
        return Vector3d(a.x, b.x, c.x);
    }
    else if (1 == j)
    {
        return Vector3d(a.y, b.y, c.y);
    }
    else if (2 == j)
    {
        return Vector3d(a.z, b.z, c.z);
    }

}

double Matrix3d::operator()(const int i, const int j) const
{
    assert((i >= 0) && (i < 3));
    assert((j >= 0) && (j < 3));
    
    if (0 == i)
    {
        return a(j);
    }
    else if (1 == i)
    {
        return b(j);
    }
    else if (2 == i)
    {
        return c(j);
    }

}

void Matrix3d::SetZero(void)
{
    a.SetZero();
    b.SetZero());
    c.SetZero();
}

void Matrix3d::SetIdentity(void)
{
    a = Vector3d(1.0, 0.0, 0.0);
    b = Vector3d(0.0, 1,0, 0.0);
    c = Vector3d(0.0, 0.0, 1,0);
}

Vector3d Matrix3d::GetDiag(void) const
{
    return Vector3d(a.x, b.y, c.z);
}

void Matrix3d::SetDiag(const Vector3d& v)
{   
    a = Vector3d(v.x, 0.0, 0.0);
    b = Vector3d(0.0, v.y, 0.0);
    c = Vector3d(0.0, 0.0, v.z);
}

void Matrix3d::SetAskew(const Vector3d& v)
{
    a = Vector3d(0.0, -v.z, v.y);
    b = Vector3d(v.z, 0.0, -v.x);
    c = Vector3d(-v.y, v.x, 0.0);
}


Matrix3d operator-(const Matrix3d& m)
{
    return Matrix3d(-m.a, -m.b, -m.c);
}

Matrix3d operator~(const Matrix3d& m)
{
    Matrix3d m_tmp(0.0);

    m_tmp.a = Vector3d(m.a.x, m.b.x, m.c.x);
    m_tmp.b = Vector3d(m.a.y, m.b.y, m.c.y);
    m_tmp.c = Vector3d(m.a.z, m.b.z, m.c.z);

    return m_tmp;
}

namespace Matrix3d_ {
Matrix3d SetClms(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2)
{
    Matrix3d m_tmp(0.0);

    m_tmp.a = Vector3d(v0.x, v1.x, v2.x);
    m_tmp.b = Vector3d(v0.y, v1.y, v2.y);
    m_tmp.c = Vector3d(v0.z, v1.z, v2.z);

    return m_tmp;
}

Matrix3d Zero(void)
{
    Matrix3d m_tmp(0.0);

    m_tmp.a = Vector3d_::Zero();
    m_tmp.b = Vector3d_::Zero();
    m_tmp.c = Vector3d_::Zero();

    return m_tmp;
}

Matrix3d Identity(void)
{
    Matrix3d m_tmp(0.0);

    m_tmp.a = Vector3d(1.0, 0.0, 0.0);
    m_tmp.b = Vector3d(0.0, 1,0, 0.0);
    m_tmp.c = Vector3d(0.0, 0.0, 1,0);

    return m_tmp;
}

Vector3d DiagVector(const Matrix3d& m)
{
    return Vector3d(m.a.x, m.b.y, m.c.z);
}

Matrix3d DiagMatrix(const Vector3d& v)
{
    Matrix3d m_tmp(0.0);
    
    m_tmp.a = Vector3d(v.x, 0.0, 0.0);
    m_tmp.b = Vector3d(0.0, v.y, 0.0);
    m_tmp.c = Vector3d(0.0, 0.0, v.z);
    
    return m_tmp;
}

double Det(const Matrix3d& m)
{
    return (m.a.x*(m.b.y*m.c.z - m.b.z*m.c.y) \
          - m.a.y*(m.b.x*m.c.z - m.b.z*m.c.x) \
          + m.a.z*(m.b.x*m.c.y - m.b.y*m.c.x);
}
Matrix3d Inv(const Matrix3d& m)
{
    double det = Det(m);
    assert(det > 0);
    
    Matrix3d m_tmp(0.0);
    
    m_tmp.a.x =  (m.b.y*m.c.z - m.b.z*m.c.y) / det;
    m_tmp.b.x = -(m.b.x*m.c.z - m.b.z*m.c.x) / det;
    m_tmp.c.x =  (m.b.x*m.c.y - m.b.y*m.c.x) / det;
    m_tmp.a.y = -(m.a.y*m.c.z - m.a.z*m.c.y) / det;
    m_tmp.b.y =  (m.a.x*m.c.z - m.a.z*m.c.x) / det;
    m_tmp.c.y = -(m.a.x*m.c.y - m.a.y*m.c.x) / det;
    m_tmp.a.z =  (m.a.y*m.b.z - m.a.z*m.b.y) / det;
    m_tmp.b.z = -(m.a.x*m.b.z - m.a.z*m.b.x) / det;
    m_tmp.c.z =  (m.a.x*m.b.y - m.a.y*m.b.x) / det;
    
    return m_tmp;
}

Matrix3d Askew(const Vector3d& v)
{
    Matrix3d m_tmp(0.0);

    m_tmp.a = Vector3d(0.0, -v.z, v.y);
    m_tmp.b = Vector3d(v.z, 0.0, -v.x);
    m_tmp.c = Vector3d(-v.y, v.x, 0.0);
    
    return m_tmp;
}


}/* namspace Matrix3d */

} /* namspace imsf */
/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/


