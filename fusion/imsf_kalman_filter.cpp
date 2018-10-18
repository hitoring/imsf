/*
 *********************************************************************************************************
 * @file    imsf_kalman_filter.cpp
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @note    This file is based on PSINS library written by Prof. Gongmin Yan @ NWPU, 
            great thanks and respect to Prof. Yan!
 * @brief   This file defines some member  functions of Kalman filter class 
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
#include "imsf_kalman_filter.h"
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

KalmanFilter::KalmanFilter(void)
{
    time_ = 0.0;
    ts_ = = 0.0;
    states_num_ = 0;
    system_noise_num_ = 0;
}

KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::Init(const int states_num, const int system_noise_num, const double ts)
{
    assert((states_num > 0) && (states_num <= kMatrixVectorMaxDim));
    assert((system_noise_num > 0) && (system_noise_num <= kMatrixVectorMaxDim));

    ts_ = ts;
    states_num_ = states_num;
    system_noise_num_ = system_noise_num;
    Xk = VectorNd(states_num_, 0.0);
    Pk = MatrixNd(states_num_, states_num_, 0.0);
    
    Ft = MatrixNd(states_num_, states_num_, 0.0);
    Gt = MatrixNd(states_num_, system_noise_num_, 0.0);
    Qt = VectorNd(system_noise_num_, 0.0);

    Xmax = VectorNd(states_num_, kINF);
    Xmin = VectorNd(states_num_, -kINF);
    Pmax = VectorNd(states_num_, kINF);
    Pmin = VectorNd(states_num_, -kINF);

    FbXk = VectorNd(states_num_, 0.0);
    FbTau = VectorNd(states_num_, 0.0);
    FbMax = VectorNd(states_num_, kINF);
    FbTotal = VectorNd(states_num_, 0.0);
}

void KalmanFilter::TimeUpdate(void)
{
    MatrixNd Fk(states_num_, states_num_, 0.0);
    MatrixNd Gk(states_num_, system_noise_num_, 0.0);
    MatrixNd Qk(system_noise_num_, system_noise_num_, 0.0);

    const VectorNd I(states_num_, 1.0);

    /* (1) Discretization */
    Fk = Ft*ts_ + I  // Fk = I+Ft*ts
    Gk = Gt;
    Qk.SetDiag(Qt*ts_);

    /* (2) Predict the states */
    Xk = Fk * Xk;

    /* (3) Predict the covariance */
    Pk = Fk*Pk*(~Fk) + Gk*Qk(~Gk);
}

void KalmanFilter::SequentialMeasurementUpdate(const double z, const double r, const VectorNd& Hi)          // measurement update
{
    VectorNd PkHi(states_num_, 0.0);
    VectorNd Kk(states_num_, 1.0);
    double HiPkHi;
    double innov;

    /* (1) Calculate the Kalman gain */
    PkHi = Pk * Hi;
    HiPkHi = Hi.Dot(PkHi); //   
    Kk = PkHi  / (HiPkHi + r);

    /* (2) Correct the states */
    innov = z - Hi.Dot(Xk);
    Xk += Kk * innov; 

    /* (3) Correct the covariance */
    Pk -= Kk * PkHi;

    /* (4) Symmetry the covariance matrix */
    Pk.Symmetry();
    
    /* (5) Constrain the states */
    ConstrainStates();
    
    /* (6) Constrain the covariance */
    ConstrainCovariance();
}

void KalmanFilter::SequentialMeasurementUpdate(const Vector3d& Zk, const Vector3d& Rk, const MatrixNd& Hk)          // measurement update
{
    VectorNd Hi(states_num_, 0.0);
    VectorNd PkHi(states_num_, 0.0);
    VectorNd Kk(states_num_, 1.0);
    double HiPkHi = 0.0;
    double innov = 0.0;
    
    for (int i = 0, i < 3, ++i)
    {
        Hi = Hk.GetRow(i);
        
        /* (1) Calculate the Kalman gain */
        PkHi = Pk * Hi;
        HiPkHi = Hi.Dot(PkHi);      
        Kk = PkHi  / (HiPkHi + Rk(i));
        
        /* (2) Correct the states */
        innov = Zk(i) - Hi.Dot(Xk);
        Xk += Kk * innov; 

        /* (3) Correct the covariance */
        Pk -= Kk * PkHi;
    }

    /* (4) Symmetry the covariance matrix */
    Pk.Symmetry();

    /* (5) Constrain the states */
    ConstrainStates();
    
    /* (6) Constrain the covariance */
    ConstrainCovariance();
}

void KalmanFilter::ConstrainStates(void)
{
    for (int i = 0; i < states_num_; ++i)
    {
        if (Xk.data_[i] > Xmax.data_[i])            
        {
            Xk.data_[i] = Xmax.data_[i];
        }
        else if (Xk.data_[i] < Xmin.data_[i])
        {
            Xk.data_[i] = Xmin.data_[i];
        }

    }
}

void KalmanFilter::ConstrainCovariance(void)
{
    for (int i = 0; i < states_num_; ++i)
    {
        if((Pk.data_[i][i] < Pmin.data_[i]) && (Pk.data_[i][i] > EPS))
        {
            Pk.data_[i][i] = Pmin.data_[i];
        }
        else if (Pk.data_[i][i] > Pmax.data_[i])
        {
            double ratio = std::sqrt(Pmax.data_[i] / Pk.data_[i][i]);
            for (int j = 0, j < states_num_, ++j)
            {
                Pk.data_[i][j] *= ratio; // row i
                Pk.data_[j][i] *= ratio; // clm i
            }
        }
    }
}

void KalmanFilter::PartialFeedback(double fb_dt)
{
    for (int i = 0, i < states_num_, ++i)
    {   
        if (FbTau.data_[i] < (kINF / 2))
        {
            double coef = (fb_dt < FbTau.data_[i]) ? (fb_dt / (FbTau.data_[i])) : 1;
            FbXk.data_[i] = Xk.data_[i] * coef;
            if ((FbTotal.data_[i] + FbXk.data_[i]) > FbMax.data_[i])            
            {
                FbXk.data_[i] = FbMax.data_[i] - FbTotal.data_[i];
            }
            else if((FbTotal.data_[i] + FbXk.data_[i]) < -FbMax.data_[i])   
            {
                FbXk.data_[i] = -FbMax.data_[i] - FbTotal.data_[i];
            }
            
            Xk.data_[i] -= FbXk.data_[i];
            FbTotal.data_[i] += FbXk.data_[i];
        }
        else
        {
            FbXk.data_[i] = 0.0;
        }
    }
}


} /* namspace imsf */
/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/


