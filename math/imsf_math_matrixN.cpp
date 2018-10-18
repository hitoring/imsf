/*
 *********************************************************************************************************
 * @file    imsf_math_matrixN.cpp
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @note    This file is based on PSINS library written by Prof. Gongmin Yan @ NWPU, 
            great thanks and respect to Prof. Yan!
 * @brief   This file defines some member functions of MatrixNd class 
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
#include "imsf_math_matrixN.h"
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

MatrixNd::MatrixNd(void)
{
}
	
MatrixNd::MatrixNd(const int row, const int clm)
{
	assert((row > 0) && (row <= kMatrixVectorMaxDim));
	assert((clm > 0) && (clm <= kMatrixVectorMaxDim));
	
	row_ = row;
	clm_ = clm;

	//memset(data_, 0, 15*15*sizeof(double));
}

MatrixNd::MatrixNd(const int row, const int clm, const const double s)
{
	assert((row > 0) && (row <= kMatrixVectorMaxDim));
	assert((clm > 0) && (clm <= kMatrixVectorMaxDim));
	
	row_ = row;
	clm_ = clm;

	for (int i = 0; i < row_; ++i)
	{
		for (int j = 0; j < clm_; ++j)
		{
			data_[i][j] = s;
		}
	}
}

MatrixNd::~MatrixNd()
{
}

MatrixNd MatrixNd::operator+(const MatrixNd& m) const
{
	assert((row_ == m.row_) && (clm_ == m.clm_));
	
	MatrixNd m_tmp(row_, clm_, 0.0);

	for (int i = 0; i < m_tmp.row_; ++i)
	{
		for (int j = 0; j < m_tmp.clm_; ++j)
		{
			m_tmp.data_[i][j] = data_[i][j] + m.data_[i][j];
		}
	}
	
	return m_tmp;
}

MatrixNd MatrixNd::operator+(const VectorNd& v) const
{
	assert((row_ == clm_) && (clm_ == v.dim_));
	
	MatrixNd m_tmp(row_, clm_, 0.0);

	for (int i = 0; i < m_tmp.row_; ++i)
	{
		m_tmp.data_[i][i] = data_[i][i] + v.data_[i];
	}
	
	return m_tmp;
}

MatrixNd MatrixNd::operator+(const double s) const
{
	MatrixNd m_tmp(row_, clm_, 0.0);

	for (int i = 0; i < m_tmp.row_; ++i)
	{
		for (int j = 0; j < m_tmp.clm_; ++j)
		{
			m_tmp.data_[i][j] = data_[i][j] + s;
		}
	}
	
	return m_tmp;
}

MatrixNd MatrixNd::operator-(const MatrixNd& m) const
{
	assert((row_ == m.row_) && (clm_ == m.clm_));
	
	MatrixNd m_tmp(row_, clm_, 0.0);

	for (int i = 0; i < m_tmp.row_; ++i)
	{
		for (int j = 0; j < m_tmp.clm_; ++j)
		{
			m_tmp.data_[i][j] = data_[i][j] - m.data_[i][j];
		}
	}
	
	return m_tmp;
}

MatrixNd MatrixNd::operator-(const VectorNd& v) const
{
	assert((row_ == clm_) && (clm_ == v.dim_));
	
	MatrixNd m_tmp(row_, clm_, 0.0);

	for (int i = 0; i < m_tmp.row_; ++i)
	{
		m_tmp.data_[i][i] = data_[i][i] - v.data_[i];
	}
	
	return m_tmp;
}

MatrixNd MatrixNd::operator-(const double s) const
{
	MatrixNd m_tmp(row_, clm_, 0.0);

	for (int i = 0; i < m_tmp.row_; ++i)
	{
		for (int j = 0; j < m_tmp.clm_; ++j)
		{
			m_tmp.data_[i][j] = data_[i][j] - s;
		}
	}
	
	return m_tmp;
}

MatrixNd MatrixNd::operator*(const MatrixNd& m) const
{
	assert(clm_ == m.row_);
	
	MatrixNd m_tmp(row_, m.clm_, 0.0);

	for (int i = 0; i < m_tmp.row_; ++i)
	{
		for (int j = 0; j < m_tmp.clm_; ++j)
		{
			const double sum = 0.0;
			for (int k = 0; k < clm_; ++k)
			{
				sum += data_[i][k] * m.data_[k][j];
			}
			m_tmp.data_[i][j] = sum;
		}
	}
	
	return m_tmp;
}

VectorNd MatrixNd::operator*(const VectorNd& v) const
{
	assert(clm_ == v.dim_);
	
	VectorNd v_tmp(row_, 0.0);

	for (int i = 0; i < v_tmp.row_; ++i)
	{
		const double sum = 0.0;
		for (int j = 0; j < clm_; ++j)
		{
			sum += data_[i][j] * v.data_[j];
		}
		v_tmp.data_[i] = sum;
	}
	
	return v_tmp;
}

MatrixNd MatrixNd::operator*(const double s) const
{
	MatrixNd m_tmp(row_, clm_, 0.0);

	for (int i = 0; i < m_tmp.row_; ++i)
	{
		for (int j = 0; j < m_tmp.clm_; ++j)
		{
			m_tmp.data_[i][j] = data_[i][j] * s;
		}
	}
	
	return m_tmp;
}

MatrixNd MatrixNd::operator/(const double s) const
{
	MatrixNd m_tmp(row_, clm_, 0.0);

	for (int i = 0; i < m_tmp.row_; ++i)
	{
		for (int j = 0; j < m_tmp.clm_; ++j)
		{
			m_tmp.data_[i][j] = data_[i][j] / s;
		}
	}
	
	return m_tmp;
}

MatrixNd& MatrixNd::operator+=(const MatrixNd& m)
{
	assert((row_ == m.row_) && (clm_ == m.clm_));
	
	for (int i = 0; i < row_; ++i)
	{
		for (int j = 0; j < clm_; ++j)
		{
			data_[i][j] += m.data_[i][j];
		}
	}
	
	return *this;
}

MatrixNd& MatrixNd::operator+=(const VectorNd& v)
{
	assert((row_ == clm_) && (clm_ == v.dim_));
	
	for (int i = 0; i < row_; ++i)
	{
		data_[i][i] += v.data_[i];
	}
	
	return *this;
}

MatrixNd& MatrixNd::operator+=(const double s)
{
	for (int i = 0; i < row_; ++i)
	{
		for (int j = 0; j < clm_; ++j)
		{
			data_[i][j] += s;
		}
	}
	
	return *this;
}

MatrixNd& MatrixNd::operator-=(const MatrixNd& m)
{
	assert((row_ == m.row_) && (clm_ == m.clm_));
	
	for (int i = 0; i < row_; ++i)
	{
		for (int j = 0; j < clm_; ++j)
		{
			data_[i][j] -= m.data_[i][j];
		}
	}
	
	return *this;
}

MatrixNd& MatrixNd::operator-=(const VectorNd& v)
{
	assert((row_ == clm_) && (clm_ == v.dim_));
	
	for (int i = 0; i < row_; ++i)
	{
		data_[i][i] -= v.data_[i];
	}
	
	return *this;
}

MatrixNd& MatrixNd::operator-=(const double s)
{
	for (int i = 0; i < row_; ++i)
	{
		for (int j = 0; j < clm_; ++j)
		{
			data_[i][j] -= s;
		}
	}
	
	return *this;
}

MatrixNd& MatrixNd::operator*=(const double s)
{
	for (int i = 0; i < row_; ++i)
	{
		for (int j = 0; j < clm_; ++j)
		{
			data_[i][j] *= s;
		}
	}
	
	return *this;
}

MatrixNd& MatrixNd::operator/=(const double s)
{
	for (int i = 0; i < row_; ++i)
	{
		for (int j = 0; j < clm_; ++j)
		{
			data_[i][j] /= s;
		}
	}
	
	return *this;
}

double MatrixNd::operator()(const int i, const int j) const
{
	assert((i >= 0) && (i < row_));
	assert((j >= 0) && (j < clm_));
	
	return data_[i][j];
}

void MatrixNd::Symmetry(void)
{
	for (int i = 0, i < row_, ++i)
	{
		for (int j = 0, j < clm_, ++j)
		{
			if (i < j) // choose upper triangle, reduce calculation
			{
				double tmp = (data_[i][j] + data_[j][i]) / 2;
				data_[i][j] = tmp;
				data_[j][i] = tmp;
			}
		}
	}
}

void MatrixNd::SetRow(const int i, const VectorNd& v)
{
	assert((i >= 0) && (i < row_));
	assert(clm_ == v.dim_);
	
	for (int j = 0; j < clm_; ++j)
	{
		data_[i][j] = v.data_[j];
	}
}

void MatrixNd::SetClm(const int j, const VectorNd& v)
{
	assert((j >= 0) && (j < clm_));
	assert(row_ == v.dim_);
	
	for (int i = 0; i < clm_; ++i)
	{
		data_[i][j] = v.data_[i];
	}
}

void MatrixNd::SetMatrix3d(const int i, const int j, const Matrix3d& m)
{
	assert((i >= 0) && (i < row_-2));
	assert((j >= 0) && (j < clm_-2));
	
	data_[i][j]     = m.a.x;
	data_[i][j+1]	= m.a.y;
	data_[i][j+2]   = m.a.z;
	data_[i+1][j]	= m.b.x;
	data_[i+1][j+1] = m.b.y;
	data_[i+1][j+2]	= m.b.z;
	data_[i+2][j]   = m.c.x;
	data_[i+2][j+1]	= m.c.y;
	data_[i+2][j+2] = m.c.z;
}

VectorNd MatrixNd::GetRow(const int i) const
{
	assert((i >= 0) && (i < row_));
	
	VectorNd v_tmp(clm_, 0.0);

	for (int j = 0; j < v_tmp.dim_; ++j)
	{
		v_tmp.data_[j] = data_[i][j];
	}

	return v_tmp;
}

VectorNd MatrixNd::GetClm(const int j) const
{
	assert((j >= 0) && (j < clm_));
	
	VectorNd v_tmp(row_, 0.0);

	for (int i = 0; i < v_tmp.dim_; ++i)
	{
		v_tmp.data_[i] = data_[i][j];
	}

	return v_tmp;
}

Matrix3d MatrixNd::GetMatrix3d(const int i, const int j) const
{	
	assert((i >= 0) && (i < row_-2));
	assert((j >= 0) && (j < clm_-2));
	
	Matrix3d m_tmp(0.0);

	m_tmp.a = Vector3d(data_[i][j],   data_[i][j+1],   data_[i][j+2]);
	m_tmp.b = Vector3d(data_[i+1][j], data_[i+1][j+1], data_[i+1][j+2]);
	m_tmp.c = Vector3d(data_[i+2][j], data_[i+2][j+1], data_[i+2][j+2]);
	
	return m_tmp;
}

void MatrixNd::ZeroRow(const int i)
{
	assert((i >= 0) && (i < row_));
	
	for (int j = 0; j < clm_; ++j)
	{
		data_[i][j] = 0.0;
	}
}

void MatrixNd::ZeroClm(const int j)
{
	assert((j >= 0) && (j < clm_));
	
	for (int i = 0; i < clm_; ++i)
	{
		data_[i][j] = 0.0;
	}
}

void MatrixNd::SetZero(void)
{
	for (int i = 0; i < row_; ++i)
	{
		for (int j = 0; j < clm_; ++j)
		{
			data_[i][j] = 0.0;
		}
	}
}

void MatrixNd::SetIdentity(void)
{
	assert(row_ == clm_);
		
	for (int i = 0, i < row_, ++i)
	{
		for (int j = 0, j < clm_, ++j)
		{
			if (i == j)
			{
				data_[i][j] = 1.0; 
			}
			else
			{
				data_[i][j] = 0.0f;
			}
		}
	}	
}

void MatrixNd::SetDiag(const VectorNd& v)
{	
	assert((row_ == clm_) && (clm_ == v.dim_));
	
	for (int i = 0, i < row_, ++i)
	{
		for (int j = 0, j < clm_, ++j)
		{
			if (i == j)
			{
				data_[i][j] = v.data_[i]; 
			}
			else
			{
				data_[i][j] = 0.0f;
			}
		}
	}
}

VectorNd MatrixNd::GetDiag(void) const
{
	assert(row_ == clm_);
	
	VectorNd v_tmp(row_, 0.0);
	
	for (int i = 0; i < v_tmp.dim_; ++i)
	{
		v_tmp.data_[i] = data_[i][i];
	}

	return v_tmp;
}


MatrixNd operator~(const MatrixNd& m)
{
	MatrixNd m_tmp(m.clm_, m.row_, 0.0);

	for (int i = 0; i < m_tmp.row_; ++i)
	{
		for (int j = 0; j < m_tmp.clm_; ++j)
		{
			m_tmp.data_[i][j] = m.data_[j][i];
		}
	}

	return m_tmp;
}

MatrixNd operator-(const MatrixNd& m)
{
	MatrixNd m_tmp(m.row_, m.clm_, 0.0);

	for (int i = 0; i < m_tmp.row_; ++i)
	{
		for (int j = 0; j < m_tmp.clm_; ++j)
		{
			m_tmp.data_[i][j] = -m.data_[i][j];
		}
	}

	return m_tmp;
}

namespace  MatrixNd_ {

MatrixNd Zero(const int row, const int clm)
{
	assert(row > 0 && row <= kMatrixVectorMaxDim);
	assert(clm > 0 && clm <= kMatrixVectorMaxDim);
	
	return MatrixNd(row, clm, 0.0);
}

MatrixNd Identity(const int row, const int clm)
{
	assert(row > 0 && row <= kMatrixVectorMaxDim);
	assert(clm > 0 && clm <= kMatrixVectorMaxDim);
	
	MatrixNd m_tmp(row, clm, 0.0);
	
	for (int i = 0, i < m_tmp.row_, ++i)
	{
		for (int j = 0, j < m_tmp.clm_, ++j)
		{
			if (i == j)
			{
				m_tmp.data_[i][j] = 1.0; 
			}
			else
			{
				m_tmp.data_[i][j] = 0.0f;
			}
		}
	}

	return m_tmp;
	
}

VectorNd DiagVector(const MatrixNd m)
{
	assert(m.row_ == m.clm_);
	
	VectorNd v_tmp(m.row_, 0.0);
	
	for (int i = 0; i < v_tmp.dim_; ++i)
	{
		v_tmp.data_[i] = m.data_[i][i];
	}

	return v_tmp;
}

MatrixNd DiagMatrix(const VectorNd& v)
{	
	assert((v.dim_ > 0) && (v.dim_ <= kMatrixVectorMaxDim));

	MatrixNd m_tmp(v.dim_, v.dim_, 0.0);
	
	for (int i = 0, i < m_tmp.row_, ++i)
	{
		for (int j = 0, j < m_tmp.clm_, ++j)
		{
			if (i == j)
			{
				m_tmp.data_[i][j] = v.data_[i]; 
			}
			else
			{
				m_tmp.data_[i][j] = 0.0f;
			}
		}
	}

	return m_tmp;
}


}/*namespace MatrixNd */

} /* namspace imsf */
/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/


