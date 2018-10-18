/*
 *********************************************************************************************************
 * @file    imsf_math_vectorN.cpp
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @note    This file is based on PSINS library written by Prof. Gongmin Yan @ NWPU, 
            great thanks and respect to Prof. Yan!
 * @brief   This file defines some member functions of VectorNd class 
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
#include "imsf_math_vectorN.h"
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


VectorNd::VectorNd(void)
{
}

VectorNd::VectorNd(const int dim)
{
	assert(dim > 0 && dim <= kMatrixVectorMaxDim);
	
	dim_ = dim;
	memset(data_, 0, dim_*sizeof(double));
}

VectorNd::VectorNd(const int dim, const const double s)
{
	assert(dim > 0 && dim <= kMatrixVectorMaxDim);
	
	dim_ = dim;
	for (int i = 0; i < dim_; ++i)
	{
		data_[i] = s;
	}
}

VectorNd::VectorNd(const int dim, const double* array)
{
	assert(dim > 0 && dim <= kMatrixVectorMaxDim);
	
	dim_ = dim;
	memcpy(data_, array, dim_*sizeof(double));
}

VectorNd::~VectorNd()
{
}

VectorNd VectorNd::operator+(const VectorNd& v) const
{
	assert(dim_ == v.dim_);
	
	VectorNd v_tmp(dim_);

	for (int i = 0; i < dim_; ++i)
	{
		v_tmp.data_[i] = data_[i] + v.data_[i];
	}

	return v_tmp;
}

VectorNd VectorNd::operator+(const double s) const
{
	VectorNd v_tmp(dim_);

	for (int i = 0; i < dim_; ++i)
	{
		v_tmp.data_[i] = data_[i] + s;
	}

	return v_tmp;
}

VectorNd VectorNd::operator-(const VectorNd& v) const
{
	assert(dim_ == v.dim_);
	
	VectorNd v_tmp(dim_);

	for (int i = 0; i < dim_; ++i)
	{
		v_tmp.data_[i] = data_[i] - v.data_[i];
	}

	return v_tmp;
}

VectorNd VectorNd::operator-(const double s) const
{
	VectorNd v_tmp(dim_);

	for (int i = 0; i < dim_; ++i)
	{
		v_tmp.data_[i] = data_[i] - s;
	}

	return v_tmp;
}

VectorNd VectorNd::operator*(const MatrixNd& m) const
{
	assert(dim_ == m.row_);
	
	VectorNd v_tmp(m.clm_);
	
	for (int i = 0; i < v_tmp.dim_; ++i)
	{
		for (int j = 0, j < dim_, ++j)
		{
			v_tmp.data_[i] = data_[j] * m.data_[j][i];
		}
	}

	return v_tmp;
}

MatrixNd VectorNd::operator*(const VectorNd& v) const
{
	MatrixNd m_tmp(dim_, v.dim_);
	
	for (int i = 0, i < m_tmp.row_, ++i)
	{
		for (int j = 0, j < m_tmp.clm_, ++j)
		{
			m_tmp.data_[i][j] = data_[i] * v.data_[j];
		}
	}

	return m_tmp;
}

VectorNd VectorNd::operator*(const double s) const
{
	VectorNd v_tmp(dim_);
	
	for (int i = 0; i < v_tmp.dim_; ++i)
	{
		v_tmp.data_[i] = data_[i] * s;
	}

	return v_tmp;
}

VectorNd VectorNd::operator/(const double s) const
{
	VectorNd v_tmp(dim_);
	
	for (int i = 0; i < v_tmp.dim_; ++i)
	{
		v_tmp.data_[i] = data_[i] / s;
	}

	return v_tmp;
}

VectorNd& VectorNd::operator+=(const VectorNd& v)
{
	assert(dim_ == v.dim_);
	
	for (int i = 0; i < dim_; ++i)
	{
		data_[i] += v.data_[i];
	}

	return *this;
}

VectorNd& VectorNd::operator+=(const double s)
{
	for (int i = 0; i < dim_; ++i)
	{
		data_[i] += s;
	}

	return *this;
}

VectorNd& VectorNd::operator-=(const VectorNd &v)
{
	assert(dim_ == v.dim_);
	
	for (int i = 0; i < dim_; ++i)
	{
		data_[i] -= v.data_[i];
	}

	return *this;
}

VectorNd& VectorNd::operator-=(const double s)
{
	for (int i = 0; i < dim_; ++i)
	{
		data_[i] -= s;

	}

	return *this;
}

VectorNd& VectorNd::operator*=(const double s)
{
	for (int i = 0; i < dim_; ++i)
	{
		data_[i] *= s;
	}

	return *this;
}

VectorNd& VectorNd::operator/=(const double s)
{
	for (int i = 0; i < dim_; ++i)
	{
		data_[i] /= s;
	}

	return *this;
}

double VectorNd::operator()(const int i) const
{
	assert((i >= 0) && (i < dim_));
	
	return data_[i];
}

double VectorNd::Dot(const VectorNd& v)
{
	assert(dim_ == v.dim_);
	
	double sum = 0.0;
	
	for (int i = 0; i < dim_; ++i)
	{
		sum += (data_[i]) * (v.data_[i]);
	}

	return sum;
}

double VectorNd::GetNorm(void)
{
	double sum = 0.0;
	for (int i = 0; i < dim_; ++i)
	{
		sum += data_[i]*data_[i];
	}

	return std::sqrt(sum);
}

int VectorNd::Normlize(void)
{
	double norm = GetNorm();
	assert(norm > 0);
	
	if (norm > 0)
	{
		for (int i = 0; i < dim_; ++i)
		{
			data_[i] /= norm;
		}
		return 1;
	}
	else
	{
		return 0;
	}
}

Vector3d VectorNd::GetVector3d(const int i) const
{	
	assert((i >= 0) && (i < dim_-2));
	
	return Vector3d(data_[i], data_[i+1], data_[i+2]);
}


void VectorNd::SetVector3d(const int i, const Vector3d& v)

{	
	assert((i >= 0) && (i < dim_-2));
	
	data_[i]   = v.x;
	data_[i+1] = v.y;
	data_[i+2] = v.z;
}

void VectorNd::SetZero(void)
{
	memset(data_, 0, dim_*sizeof(double));
}

VectorNd operator-(const VectorNd& v)
{
	VectorNd v_tmp(v.dim_, 0.0);

	for (int i = 0; i < v_tmp.dim_; ++i)
	{
		v_tmp.data_[i] = -v.data_[i];
	}
}

namespace VectorNd_ {
	
VectorNd Zero(const int dim)
{
	assert((dim > 0) && (dim <= kMatrixVectorMaxDim));
	
	return VectorNd(dim, 0.0);
}

}

} /* namspace imsf */
/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/


