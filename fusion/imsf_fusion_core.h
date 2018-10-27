/*
 *********************************************************************************************************
 * @file    imsf_fusion_core.h
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @brief   This file defines a class for 15 error states Kalman filter 
            for intelligent multi-sensor fusion library.
            The 15 states including:
                (1) attitude_error(3)
                (2) velocity_error(3)
                (3) position_error(3)
                (4) gyroscope_bias(3)
                (5) accelerometer_bias(3)
 *********************************************************************************************************
 */

/* Define to prevent recursive inclusion ----------------------------------------------------------------*/
#ifndef IMSF_FUSION_CORE_H_
#define IMSF_FUSION_CORE_H_

/*
 *********************************************************************************************************
 *                                          INCLUDE HEADER FILES
 *********************************************************************************************************
 */
/* Include standard library header files ----------------------------------------------------------------*/
/* Include local library header files -------------------------------------------------------------------*/
#include "imsf_math.h"
#include "imsf_kalman_filter.h"
#include "imsf_sins_core.h"
#include "imsf_initial_align.h"
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

typedef enum
{
    ERROR_STATE_ATT_X_ERR_IDX   = 0,
    ERROR_STATE_ATT_Y_ERR_IDX   = 1,
    ERROR_STATE_ATT_Z_ERR_IDX   = 2,
    ERROR_STATE_VEL_X_ERR_IDX   = 3,
    ERROR_STATE_VEL_Y_ERR_IDX   = 4,
    ERROR_STATE_VEL_Z_ERR_IDX   = 5,
    ERROR_STATE_POS_X_ERR_IDX   = 6,
    ERROR_STATE_POS_Y_ERR_IDX   = 7,
    ERROR_STATE_POS_Z_ERR_IDX   = 8,
    ERROR_STATE_GYRO_X_BIAS_IDX = 9,
    ERROR_STATE_GYRO_Y_BIAS_IDX = 10,
    ERROR_STATE_GYRO_Z_BIAS_IDX = 11,
    ERROR_STATE_ACC_X_BIAS_IDX  = 12,
    ERROR_STATE_ACC_Y_BIAS_IDX  = 13,
    ERROR_STATE_ACC_Z_BIAS_IDX  = 14
}ErrorStateIndex;

class OutputStates
{
public:
    /* Member variables */
    Quaterniond qnb_; // quaternion from body frame to navigation frame
    Vector3d vn_;     // veclocity in navigation frame
    Vector3d pos_;    // position in navigation frame
    Vector3d an_;     // acceleration in navigation frame
    Vector3d wnb_;    // angular rate from body frame to navigation frame

    /* Constructors */
    OutputStates(void);
};

class ErrorStateKalmanFilter
{
public:
    /* Member variables */
    double time_;
    int filter_initialized_;
    KalmanFilter kf_;
    SINS sins_;
    OutputStates output_states_;
    
    Vector3d att_err_;   // attitude error
    Vector3d vel_err_;   // velocity error
    Vector3d pos_err_;   // position error
    Vector3d gyro_bias_; // gyroscope bias
    Vector3d acc_bias_;  // accelerometer bias
    
    Vector3d att_err_var_;   // attitude error variance
    Vector3d vel_err_var_;   // velocity error variance
    Vector3d pos_err_var_;   // position error variance
    Vector3d gyro_bias_var_; // gyroscope bias variance
    Vector3d acc_bias_var_;  // accelerometer bias variance

    /* Constructors */
    ErrorStateKalmanFilter(void);

    /* Destructor */
    ~ErrorStateKalmanFilter();

    /* Member functions */
    void Init(const Quaterniond& qnb0, const Vector3d& vn0, const Vector3d& pos0);
    int Update(void);
    void StatesVectorToStatesElements(void);
    void CovarianceMatrixToVarianceElements(void);
    VectorNd StatesElementsToStatesVector(void);
    VectorNd VarianceElementsToCovarianceDiagonal(void);
    void VarianceElementsToCovarianceMatrix(void);
    void SetQt(void);
    void FuseWithPosition(const double time, const Vector3d& pos, const Vector3d& pos_var);
    void FuseWithVelocity(const double time, const Vector3d& vel, const Vector3d& vel_var);
    void FuseWithAttitude(const double time, const Quaterniond& qnb, const Vector3d& att_var);
    void FuseWithAltitude(const double time, const double alt, const double alt_var);
    void FuseWithYaw(const double time, const double yaw, const double yaw_var);
    void CorrectNavigationStates(void);
    void SetSystemMatrix(void);
    void CalcOutputStates(void);
    void ControlFusionModes(void);
};


} /* namespace imsf */

#endif /* IMSF_FUSION_CORE_H_ */

/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/


