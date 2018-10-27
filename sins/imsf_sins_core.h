/*
 *********************************************************************************************************
 * @file    imsf_sins_core.h
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @brief   This file defines a class for strapdown inertial navigation system 
            for intelligent multi-sensor fusion library.
 *********************************************************************************************************
 */

/* Define to prevent recursive inclusion ----------------------------------------------------------------*/
#ifndef IMSF_SINS_CORE_H_
#define IMSF_SINS_CORE_H_

/*
 *********************************************************************************************************
 *                                          INCLUDE HEADER FILES
 *********************************************************************************************************
 */
/* Include standard library header files ----------------------------------------------------------------*/
/* Include local library header files -------------------------------------------------------------------*/
#include "imsf_math.h"
#include "imsf_sins_imu.h"
#include "imsf_sins_earth.h"
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

class SINS
{
public:
    /* Member variables */
    EarthParam earth_param_;
    IMU imu_;

    double time_;
    double nts_;
    
    Quaterniond qnb_;
    Quaterniond qnb0_; // to save initial alignment
    Matrix3d Cnb_;
    Matrix3d Cbn_;
    Vector3d euler_angle_;
    Vector3d pos_;
    Vector3d vn_;
    Vector3d vb_;
    Vector3d an_;
    Vector3d fn_;
    Vector3d fb_;
    Vector3d wib_;
    Vector3d web_;
    Vector3d wnb_;
    
    Matrix3d Kg_;
    Matrix3d Ka_;
    Vector3d gyro_bias_;
    Vector3d acc_bias_;
    Vector3d tau_gyro_;
    Vector3d tau_acc_;
    
    Matrix3d Maa_, Mav_, Map_, Mva_, Mvv_, Mvp_, Mpv_, Mpp_;    // for error transition matrix

    /* Constructors */
    SINS(const Quaterniond& qnb0, const Vector3d& vn0, const Vector3d& pos0);

    /* Destructor */
    ~SINS();

    /* Member functions */
    void Init(const Quaterniond& qnb0, const Vector3d& vn0, const Vector3d& pos0, const int subsamples, const double nts);
    void Update(const Vector3d* delta_angle, const Vector3d* delta_vel);
    void CalcErrorTransitionMatrix(void);

};



} /* namespace imsf */

#endif /* IMSF_SINS_CORE_H_ */

/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/


