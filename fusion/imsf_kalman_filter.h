/*
 *********************************************************************************************************
 * @file    imsf_kalman_filter.h
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @note    This file is based on PSINS library written by Prof. Gongmin Yan @ NWPU, 
            great thanks and respect to Prof. Yan!
 * @brief   This file defines a Kalman filter class 
            for intelligent multi-sensor fusion library (imsf).
 *********************************************************************************************************
 */

/* Define to prevent recursive inclusion ----------------------------------------------------------------*/
#ifndef IMSF_KALMAN_FILTER_H_
#define IMSF_KALMAN_FILTER_H_

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

/* class explanation */
/* X = Ft*X + Gt*W */
class KalmanFilter
{
public:
    /* Member variables */
	double time_; // time stamp
	double ts_;   // sample time
	int states_num_; // states number
	int system_noise_num_; // system noise number
	VectorNd Xk; // states
	VectorNd Xmax, Xmin; // maximum and minimum of states, for constraining states strategy
	MatrixNd Pk; // covariance matrix
	VectorNd Pmax, Pmin; // the maximum and minimum of the diagonal of covariance matrix
	MatrixNd Ft; // system transition matrix
	MatrixNd Gt; // noise distribution matrix
	VectorNd Qt; // system noise matrix
	VectorNd FbXk, FbMax, FbTau, FbTotal;  // parameters for partial feedback control

    /* Constructors */
	KalmanFilter(void);

    /* Destructor */
	~KalmanFilter();

    /* Member functions */
    void Init(const int states_num, const int system_noise_num, const double ts);
	void TimeUpdate(void);
	void SequentialMeasurementUpdate(const double z, const double r, const VectorNd& Hi);
	void SequentialMeasurementUpdate(const Vector3d& Zk, const Vector3d& Rk, const MatrixNd& Hk);
	void ConstrainStates(void);
	void ConstrainCovariance(void);
	void PartialFeedback(double fb_dt);
};



} /* namespace imsf */

#endif /* IMSF_KALMAN_FILTER_H_ */

/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/


