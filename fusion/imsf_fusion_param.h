/*
 *********************************************************************************************************
 * @file    imsf_fusion_param.h
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @note    This file is based on PSINS library written by Prof. Gongmin Yan @ NWPU, 
            great thanks and respect to Prof. Yan!
 * @brief   This file defines some fusion configuration parameters
            for intelligent multi-sensor fusion library.
 *********************************************************************************************************
 */

/* Define to prevent recursive inclusion ----------------------------------------------------------------*/
#ifndef IMSF_FUSION_PARAM_H_
#define IMSF_FUSION_PARAM_H_

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

/* IMU subsampling and filter updating correlation constants */
static const int    kSubsampleNumber = 1;
static const double kFilterUpdatePeriod = 0.01;

/* Kalman filter initialization */
static const int kStatesNumber = 15;
static const int kSystemNoiseNumber = 12;

/* Time correlation constants of first order Markov process */
static const double kMarkovTauGyroX = 1.0;
static const double kMarkovTauGyroY = 1.0;
static const double kMarkovTauGyroZ = 1.0;
static const double kMarkovTauAccX = 1.0;
static const double kMarkovTauAccY = 1.0;
static const double kMarkovTauAccZ = 1.0;

/* Initial states (X0) */
static const double kInitialAttXError = 0.0;
static const double kInitialAttYError = 0.0;
static const double kInitialAttZError = 0.0;
static const double kInitialVelXError = 0.0;
static const double kInitialVelYError = 0.0;
static const double kInitialVelZError = 0.0;
static const double kInitialPosXError = 0.0;
static const double kInitialPosYError = 0.0;
static const double kInitialPosZError = 0.0;
static const double kInitialGyroXBias = 0.0;
static const double kInitialGyroYBias = 0.0;
static const double kInitialGyroZBias = 0.0;
static const double kInitialAccXBias = 0.0;
static const double kInitialAccYBias = 0.0;
static const double kInitialAccZBias = 0.0;

/* Maximum states (Xmax) */
static const double kMaxAttXError = 1.0;
static const double kMaxAttYError = 1.0;
static const double kMaxAttZError = 1.0;
static const double kMaxVelXError = 1.0;
static const double kMaxVelYError = 1.0;
static const double kMaxVelZError = 1.0;
static const double kMaxPosXError = 1.0;
static const double kMaxPosYError = 1.0;
static const double kMaxPosZError = 1.0;
static const double kMaxGyroXBias = 1.0;
static const double kMaxGyroYBias = 1.0;
static const double kMaxGyroZBias = 1.0;
static const double kMaxAccXBias = 1.0;
static const double kMaxAccYBias = 1.0;
static const double kMaxAccZBias = 1.0;

/* Minimum states (Xmin) */
static const double kMinAttXError = -1.0;
static const double kMinAttYError = -1.0;
static const double kMinAttZError = -1.0;
static const double kMinVelXError = -1.0;
static const double kMinVelYError = -1.0;
static const double kMinVelZError = -1.0;
static const double kMinPosXError = -1.0;
static const double kMinPosYError = -1.0;
static const double kMinPosZError = -1.0;
static const double kMinGyroXBias = -1.0;
static const double kMinGyroYBias = -1.0;
static const double kMinGyroZBias = -1.0;
static const double kMinAccXBias = -1.0;
static const double kMinAccYBias = -1.0;
static const double kMinAccZBias = -1.0;

/* Initial covariance matrix (P0) */
static const double kInitialAttXErrorVariance = 10.0*unit.deg;
static const double kInitialAttYErrorVariance = 10.0*unit.deg;
static const double kInitialAttZErrorVariance = 30.0*unit.deg;
static const double kInitialVelXErrorVariance = 1.0;
static const double kInitialVelYErrorVariance = 1.0;
static const double kInitialVelZErrorVariance = 1.0;
static const double kInitialPosXErrorVariance = 100.0/unit.Re;
static const double kInitialPosYErrorVariance = 100.0/unit.Re;
static const double kInitialPosZErrorVariance = 100.0;
static const double kInitialGyroXBiasVariance = 1000.0*unit.dph;
static const double kInitialGyroYBiasVariance = 1000.0*unit.dph;
static const double kInitialGyroZBiasVariance = 1000.0*unit.dph;
static const double kInitialAccXBiasVariance  = 10.0*unit.mg;
static const double kInitialAccYBiasVariance  = 10.0*unit.mg;
static const double kInitialAccZBiasVariance  = 10.0*unit.mg;

/* Maximum covariance matrix (Pmax) */
static const double kMaxAttXErrorVariance = 10.0*unit.deg;
static const double kMaxAttYErrorVariance = 10.0*unit.deg;
static const double kMaxAttZErrorVariance = 30.0*unit.deg;
static const double kMaxVelXErrorVariance = 50.0;
static const double kMaxVelYErrorVariance = 50.0;
static const double kMaxVelZErrorVariance = 50.0;
static const double kMaxPosXErrorVariance = 1.0e4/unit.Re;
static const double kMaxPosYErrorVariance = 1.0e4/unit.Re;
static const double kMaxPosZErrorVariance = 1.0e4;
static const double kMaxGyroXBiasVariance = 1000.0*unit.dph;
static const double kMaxGyroYBiasVariance = 1000.0*unit.dph;
static const double kMaxGyroZBiasVariance = 1000.0*unit.dph;
static const double kMaxAccXBiasVariance = 100*unit.mg;
static const double kMaxAccYBiasVariance = 100*unit.mg;
static const double kMaxAccZBiasVariance = 100*unit.mg;

/* Minimum covariance matrix (Pmin) */
static const double kMinAttXErrorVariance = 1.0*unit.min;
static const double kMinAttYErrorVariance = 1.0*unit.min;
static const double kMinAttZErrorVariance = 1.0*unit.min;
static const double kMinVelXErrorVariance = 0.01;
static const double kMinVelYErrorVariance = 0.01;
static const double kMinVelZErrorVariance = 1.0;
static const double kMinPosXErrorVariance = 1.0/unit.Re;
static const double kMinPosYErrorVariance = 1.0/unit.Re;
static const double kMinPosZErrorVariance = 1.0;
static const double kMinGyroXBiasVariance = 1.0*unit.dph;
static const double kMinGyroYBiasVariance = 1.0*unit.dph;
static const double kMinGyroZBiasVariance = 1.0*unit.dph ;
static const double kMinAccXBiasVariance = 0.1*unit.mg;
static const double kMinAccYBiasVariance = 0.1*unit.mg;
static const double kMinAccZBiasVariance = 0.1*unit.mg;


/* System noise covariance matrix (Qt) */
static const double kSystemGyroXNoiseVariance = 1.0*unit.dpsh;
static const double kSystemGyroYNoiseVariance = 1.0*unit.dpsh;
static const double kSystemGyroZNoiseVariance = 1.0*unit.dpsh;
static const double kSystemAccXNoiseVariance = 100.0*unit.ugpsHz;
static const double kSystemAccYNoiseVariance = 100.0*unit.ugpsHz;
static const double kSystemAccZNoiseVariance = 100.0*unit.ugpsHz;
static const double kMarkovGyroXNoiseVariance = 1.0*unit.dpsh;
static const double kMarkovGyroYNoiseVariance = 1.0*unit.dpsh;
static const double kMarkovGyroZNoiseVariance = 1.0*unit.dpsh;
static const double kMarkovAccXNoiseVariance = 100.0*unit.ugpsHz;
static const double kMarkovAccYNoiseVariance = 100.0*unit.ugpsHz;
static const double kMarkovAccZNoiseVariance = 100.0*unit.ugpsHz;

/* Time correlation constants of Partial feedback (FbTau) */
static const double kAttXErrorFeedbackTau = 1.0;
static const double kAttYErrorFeedbackTau = 1.0;
static const double kAttZErrorFeedbackTau = 10.0;
static const double kVelXErrorFeedbackTau = 1.0;
static const double kVelYErrorFeedbackTau = 1.0;
static const double kVelZErrorFeedbackTau = 1.0;
static const double kPosXErrorFeedbackTau = 1.0;
static const double kPosYErrorFeedbackTau = 1.0;
static const double kPosZErrorFeedbackTau = 1.0;
static const double kGyroXBiasFeedbackTau = 10.0;
static const double kGyroYBiasFeedbackTau = 10.0;
static const double kGyroZBiasFeedbackTau = 10.0;
static const double kAccXBiasFeedbackTau = 10.0;
static const double kAccYBiasFeedbackTau = 10.0;
static const double kAccZBiasFeedbackTau = 1.0;

/* Maximum of partial feedback (FbMax) */
static const double kAttXErrorMaxFeedback = kINF;
static const double kAttYErrorMaxFeedback = kINF;
static const double kAttZErrorMaxFeedback = kINF;
static const double kVelXErrorMaxFeedback = kINF;
static const double kVelYErrorMaxFeedback = kINF;
static const double kVelZErrorMaxFeedback = kINF;
static const double kPosXErrorMaxFeedback = kINF;
static const double kPosYErrorMaxFeedback = kINF;
static const double kPosZErrorMaxFeedback = kINF;
static const double kGyroXBiasMaxFeedback = 5000*unit.dph;
static const double kGyroYBiasMaxFeedback = 5000*unit.dph;
static const double kGyroZBiasMaxFeedback = 5000*unit.dph;
static const double kAccXBiasMaxFeedback = 50*unit.mg;
static const double kAccYBiasMaxFeedback = 50*unit.mg;
static const double kAccZBiasMaxFeedback = 50*unit.mg;




} /* namespace imsf */

#endif /* IMSF_FUSION_PARAM_H_ */

/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/


