/*
 *********************************************************************************************************
 * @file    imsf_initial_align.h
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @note    This file is based on PSINS library written by Prof. Gongmin Yan @ NWPU, 
            great thanks and respect to Prof. Yan!
 * @brief   This file defines a initial alignment class 
            for intelligent multi-sensor fusion library.
 *********************************************************************************************************
 */

/* Define to prevent recursive inclusion ----------------------------------------------------------------*/
#ifndef IMSF_INITIAL_ALIGN_H_
#define IMSF_INITIAL_ALIGN_H_

/*
 *********************************************************************************************************
 *                                          INCLUDE HEADER FILES
 *********************************************************************************************************
 */
/* Include standard library header files ----------------------------------------------------------------*/
/* Include local library header files -------------------------------------------------------------------*/
#include "imsf_math.h"
#include "imsf_sins_core.h"
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

class InitialAlign
{
public:
    /* Member variables */
    EarthParam earth_param_;
    IMU imu_;

    double time_;
    double nts_;
    int t0, t1, t2;
    Vector3d delta_angle_sum_, delta_vel_sum_; 
    Vector3d vib0, vi0, Pib01, Pib02, Pi01, Pi02, tmpPib0, tmpPi0;
    Quaterniond qib0b;

    /* Constructors */
    InitialAlign(void);

    /* Destructor */
    ~InitialAlign();

    /* Member functions */
    void Init(const Vector3d& pos, const Vector3d& vn, const int subsamples, const double nts);
    Quaterniond Update(const Vector3d* delta_angle, const Vector3d* delta_vel);
};


Vector3d AlignCoarse(Vector3d delta_angle_sum, Vector3d delta_vel_sum, double latitude);



} /* namespace imsf */

#endif /* IMSF_INITIAL_ALIGN_H_ */

/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/


