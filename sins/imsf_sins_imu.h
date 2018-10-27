/*
 *********************************************************************************************************
 * @file    imsf_fusion_imu.h
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @brief   This file defines a IMU setting class 
            for intelligent multi-sensor fusion library.
 *********************************************************************************************************
 */

/* Define to prevent recursive inclusion ----------------------------------------------------------------*/
#ifndef IMSF_SINS_IMU_H_
#define IMSF_SINS_IMU_H_

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
class IMU
{
public:
    /* Member variables */
    int updated_; // flag for IMU data updated or not
    int subsamples_; // subsample number
    double sub_ts_;
    double time_;
    Vector3d phim_; // rotation vector after coning compensation
    Vector3d dvbm_; // velocity increment after rotation & sculling compensation
    Vector3d delta_angle_prev_; // last angular increment
    Vector3d delta_vel_prev_; // last velocity increment
    Vector3d dphim_; // attitude coning error
    Vector3d rotm_; // velocity rotation error
    Vector3d scullm_; // velocity sculling error

    /* Constructors */
    IMU(void);
    
    /* Destructor */
    ~IMU();

    /* Member functions */
    void Init(const int subsamples, const double sub_ts);
    void Update(const Vector3d* delta_angle, const Vector3d* delta_vel);
};



} /* namespace imsf */

#endif /* IMSF_SINS_IMU_H_ */

/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/


