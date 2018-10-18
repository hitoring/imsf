/*
 *********************************************************************************************************
 * @file    imsf_fusion_core.cpp
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @brief   This file defines some member functions of ErrorStateKalmanFilter class 
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
#include "imsf_fusion_core.h"
#include "imsf_fusion_param.h"
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

ErrorStateKalmanFilter::ErrorStateKalmanFilter(void)
{
	kf_ = KalmanFilter();
	sins_ = SINS();
    output_states_ = OutputStates();

    filter_initialized_ = 0;

	att_err_.SetZero();
	vel_err_.SetZero();
	pos_err_.SetZero();
	gyro_bias_.SetZero();
	acc_bias_.SetZero();

	
	att_err_var_.SetZero();
	vel_err_var_.SetZero();
	pos_err_var_.SetZero();
	gyro_bias_var_.SetZero();
	acc_bias_var_.SetZero();
}

ErrorStateKalmanFilter::~ErrorStateKalmanFilter()
{
}

void ErrorStateKalmanFilter::Init(const Quaterniond& qnb0, const Vector3d& vn0, const Vector3d& pos0)
{
    kf_.Init(kStatesNumber, kSystemNoiseNumber, kFilterUpdatePeriod);
    sins_.Init(qnb0, vn0, pos0, kSubsampleNumber, kFilterUpdatePeriod);

    /* Set Xmax (maximum states) */
	att_err_ = Vector3d(kMaxAttXError, kMaxAttYError, kMaxAttZError);
	vel_err_ = Vector3d(kMaxVelXError, kMaxVelYError, kMaxVelZError);
	pos_err_ = Vector3d(kMaxPosXError, kMaxPosYError, kMaxPosZError);
	gyro_bias_ = Vector3d(kMaxGyroXBias, kMaxGyroYBias, kMaxGyroZBias);
	acc_bias_ = Vector3d(kMaxAccXBias, kMaxAccYBias, kMaxAccZBias);
	kf_.Xmax = StatesElementsToStatesVector();

    /* Set Xmin (minimum states) */
	att_err_ = Vector3d(kMinAttXError, kMinAttYError, kMinAttZError);
	vel_err_ = Vector3d(kMinVelXError, kMinVelYError, kMinVelZError);
	pos_err_ = Vector3d(kMinPosXError, kMinPosYError, kMinPosZError);
	gyro_bias_ = Vector3d(kMinGyroXBias, kMinGyroYBias, kMinGyroZBias);
	acc_bias_ = Vector3d(kMinAccXBias, kMinAccYBias, kMinAccZBias);
	kf_.Xmin = StatesElementsToStatesVector();

    /* Set FbTau (Time correlation constants of Partial feedback) */
    att_err_ = Vector3d(kAttXErrorFeedbackTau, kAttYErrorFeedbackTau, kAttZErrorFeedbackTau);
	vel_err_ = Vector3d(kVelXErrorFeedbackTau, kVelYErrorFeedbackTau, kVelZErrorFeedbackTau);
	pos_err_ = Vector3d(kPosXErrorFeedbackTau, kPosYErrorFeedbackTau, kPosZErrorFeedbackTau);
	gyro_bias_ = Vector3d(kGyroXBiasFeedbackTau, kAttYErrorFeedbackTau, kAttZErrorFeedbackTau);
	acc_bias_ = Vector3d(kAttXErrorFeedbackTau, kAttYErrorFeedbackTau, kAttZErrorFeedbackTau);
	kf_.FbTau = StatesElementsToStatesVector();

    /* Set FbMax (Maximum of partial feedback) */
    att_err_ = Vector3d(kAttXErrorMaxFeedback, kAttYErrorMaxFeedback, kAttZErrorMaxFeedback);
	vel_err_ = Vector3d(kVelXErrorMaxFeedback, kVelYErrorMaxFeedback, kVelZErrorMaxFeedback);
	pos_err_ = Vector3d(kPosXErrorMaxFeedback, kPosYErrorMaxFeedback, kPosZErrorMaxFeedback);
	gyro_bias_ = Vector3d(kGyroXBiasMaxFeedback, kGyroYBiasMaxFeedback, kGyroZBiasMaxFeedback);
	acc_bias_ = Vector3d(kAccXBiasMaxFeedback, kAccYBiasMaxFeedback, kAccZBiasMaxFeedback);
	kf_.FbMax = StatesElementsToStatesVector();

    /* Set X0 (Initial states) */
	att_err_ = Vector3d(kInitialAttXError, kInitialAttYError, kInitialAttZError);
	vel_err_ = Vector3d(kInitialVelXError, kInitialVelYError, kInitialVelZError);
	pos_err_ = Vector3d(kInitialPosXError, kInitialPosYError, kInitialPosZError);
	gyro_bias_ = Vector3d(kInitialGyroXBias, kInitialGyroYBias, kInitialGyroZBias);
	acc_bias_ = Vector3d(kInitialAccXBias, kInitialAccYBias, kInitialAccZBias);
	kf_.Xk = StatesElementsToStatesVector();

    /* Set Pmax (Maximum covariance matrix) */
	att_err_var_ = Vector3d(kMaxAttXErrorVariance, kMaxAttYErrorVariance, kMaxAttZErrorVariance);
	vel_err_var_ = Vector3d(kMaxVelXErrorVariance, kMaxVelYErrorVariance, kMaxVelZErrorVariance);
	pos_err_var_ = Vector3d(kMaxPosXErrorVariance, kMaxPosYErrorVariance, kMaxPosZErrorVariance);
	gyro_bias_var_ = Vector3d(kMaxGyroXBiasVariance, kMaxGyroYBiasVariance, kMaxGyroZBiasVariance);
	acc_bias_var_ = Vector3d(kMaxAccXBiasVariance, kMaxAccYBiasVariance, kMaxAccZBiasVariance);
	kf_.Pmax = VarianceElementsToCovarianceDiagonal();

    /* Set Pmin (Minimum covariance matrix) */
	att_err_var_ = Vector3d(kMinAttXErrorVariance, kMinAttYErrorVariance, kMinAttZErrorVariance);
	vel_err_var_ = Vector3d(kMinVelXErrorVariance, kMinVelYErrorVariance, kMinVelZErrorVariance);
	pos_err_var_ = Vector3d(kMinPosXErrorVariance, kMinPosYErrorVariance, kMinPosZErrorVariance);
	gyro_bias_var_ = Vector3d(kMinGyroXBiasVariance, kMinGyroYBiasVariance, kMinGyroZBiasVariance);
	acc_bias_var_ = Vector3d(kMinAccXBiasVariance, kMinAccYBiasVariance, kMinAccZBiasVariance);
	kf_.Pmin = VarianceElementsToCovarianceDiagonal();

    /* Set P0 (Initial covariance matrix) */
	att_err_var_ = Vector3d(kInitialAttXErrorVariance, kInitialAttYErrorVariance, kInitialAttZErrorVariance);
	vel_err_var_ = Vector3d(kInitialVelXErrorVariance, kInitialVelYErrorVariance, kInitialVelZErrorVariance);
	pos_err_var_ = Vector3d(kInitialPosXErrorVariance, kInitialPosYErrorVariance, kInitialPosZErrorVariance);
	gyro_bias_var_ = Vector3d(kInitialGyroXBiasVariance, kInitialGyroYBiasVariance, kInitialGyroZBiasVariance);
	acc_bias_var_ = Vector3d(kInitialAccXBiasVariance, kInitialAccYBiasVariance, kInitialAccZBiasVariance);
	VarianceElementsToCovarianceMatrix();

    /* Set Qt (System noise covariance matrix) */
    SetQt();

    filter_initialized_ = 1;

}

int ErrorStateKalmanFilter::Update(void)
{	
	if (!filter_initialized_) 
	{
		(!filter_initialized_) = Init();

		if (!(!filter_initialized_)) 
		{
			return -1;
		}
	}

	// Only run the filter if IMU data in the buffer has been updated
	if (sins_.imu_.updated_) 
	{
		// perform state and covariance prediction for the main filter
		sins_.Update();
		SetSystemMatrix();
		kf_.TimeUpdate();

		// control fusion of all measurement data
		ControlFusionModes();
	}

	// the output observer always runs
	CalcOutputStates();

	// check for NaN or inf on attitude states
	if (!ISFINITE(sins_.qnb_.q0) || !ISFINITE(output_states_.qnb_.q0)) 
	{
		return -1;
	}

	return 1;
	
}

void ErrorStateKalmanFilter::StatesVectorToStatesElements(void)
{
	att_err_ = kf_.FbXk.GetVector3d(ERROR_STATE_ATT_X_ERR_IDX);
	vel_err_ = kf_.FbXk.GetVector3d(ERROR_STATE_VEL_X_ERR_IDX);
	pos_err_ = kf_.FbXk.GetVector3d(ERROR_STATE_POS_X_ERR_IDX);
	gyro_bias_ = kf_.FbXk.GetVector3d(ERROR_STATE_GYRO_X_BIAS_IDX);
	acc_bias_ = kf_.FbXk.GetVector3d(ERROR_STATE_ACC_X_BIAS_IDX);
}

void ErrorStateKalmanFilter::CovarianceMatrixToVarianceElements(void)
{
	VectorNd Pk_diag = kf_.Pk.GetDiag();
    
	att_err_var_ = Pk_diag.GetVector3d(ERROR_STATE_ATT_X_ERR_IDX);
	vel_err_var_ = Pk_diag.GetVector3d(ERROR_STATE_VEL_X_ERR_IDX);
	pos_err_var_ = Pk_diag.GetVector3d(ERROR_STATE_POS_X_ERR_IDX);
	gyro_bias_var_ = Pk_diag.GetVector3d(ERROR_STATE_GYRO_X_BIAS_IDX);
	acc_bias_var_ = Pk_diag.GetVector3d(ERROR_STATE_ACC_X_BIAS_IDX);
}

VectorNd ErrorStateKalmanFilter::StatesElementsToStatesVector(void)
{
	VectorNd states(kf_.states_num_, 0.0);
	
	states.SetVector3d(ERROR_STATE_ATT_X_ERR_IDX, att_err_);
	states.SetVector3d(ERROR_STATE_VEL_X_ERR_IDX, vel_err_);
	states.SetVector3d(ERROR_STATE_POS_X_ERR_IDX, pos_err_);
	states.SetVector3d(ERROR_STATE_GYRO_X_BIAS_IDX, gyro_bias_);
	states.SetVector3d(ERROR_STATE_ACC_X_BIAS_IDX, acc_bias_);

	return states;
}

VectorNd ErrorStateKalmanFilter::VarianceElementsToCovarianceDiagonal(void)
{
	VectorNd Pk_diag(kf_.states_num_, 0.0);
	
	Pk_diag.SetVector3d(ERROR_STATE_ATT_X_ERR_IDX, att_err_var_);
	Pk_diag.SetVector3d(ERROR_STATE_VEL_X_ERR_IDX, vel_err_var_);
	Pk_diag.SetVector3d(ERROR_STATE_POS_X_ERR_IDX, pos_err_var_);
	Pk_diag.SetVector3d(ERROR_STATE_GYRO_X_BIAS_IDX, gyro_bias_var_);
	Pk_diag.SetVector3d(ERROR_STATE_ACC_X_BIAS_IDX, acc_bias_var_);
	
	return Pk_diag;
}


void ErrorStateKalmanFilter::VarianceElementsToCovarianceMatrix(void)
{
	VectorNd Pk_diag(kf_.states_num_, 0.0);
	
	Pk_diag.SetVector3d(ERROR_STATE_ATT_X_ERR_IDX, att_err_var_);
	Pk_diag.SetVector3d(ERROR_STATE_VEL_X_ERR_IDX, vel_err_var_);
	Pk_diag.SetVector3d(ERROR_STATE_POS_X_ERR_IDX, pos_err_var_);
	Pk_diag.SetVector3d(ERROR_STATE_GYRO_X_BIAS_IDX, gyro_bias_var_);
	Pk_diag.SetVector3d(ERROR_STATE_ACC_X_BIAS_IDX, acc_bias_var_);
	
	kf_.Pk.SetDiag(Pk_diag);
}

void ErrorStateKalmanFilter::SetQt(void)
{
    Vector3d v_tmp(0.0);
    
    v_tmp = Vector3d(kSystemGyroXNoiseVariance, kSystemGyroYNoiseVariance, kSystemGyroZNoiseVariance);
    kf_.Qt.SetVector3d(0, v_tmp);
    
    v_tmp = Vector3d(kSystemAccXNoiseVariance, kSystemAccYNoiseVariance, kSystemAccZNoiseVariance);
    kf_.Qt.SetVector3d(3, v_tmp);
    
    v_tmp = Vector3d(kMarkovGyroXNoiseVariance, kMarkovGyroYNoiseVariance, kMarkovGyroZNoiseVariance);
    kf_.Qt.SetVector3d(6, v_tmp);
    
    v_tmp = Vector3d(kMarkovAccXNoiseVariance, kMarkovAccYNoiseVariance, kMarkovAccZNoiseVariance);
    kf_.Qt.SetVector3d(9, v_tmp);
}

void ErrorStateKalmanFilter::FuseWithPosition(const double time, const Vector3d& pos, const Vector3d& pos_var)
{
	double dt = sins_.time_ - time;
	Vector3d pos_err = sins_.pos_ - (pos + sins_.Mpv_*sins_.vn_*dt);
	
	MatrixNd Hk_pos(3, kf_.states_num_, 0.0);
	Hk_pos.SetMatrix3d(0, ERROR_STATE_POS_X_ERR_IDX, Matrix3d_::Identity());

	kf_.SequentialMeasurementUpdate(pos_err, pos_var, Hk_pos);
	CorrectNavigationStates();
}

void ErrorStateKalmanFilter::FuseWithVelocity(const double time, const Vector3d& vel, const Vector3d& vel_var)
{
	double dt = sins_.time_ - time;
	Vector3d vel_err = sins_.vn_ - (vel + sins_.an_*dt);
	
	MatrixNd Hk_vel(3, kf_.states_num_, 0.0);
	Hk_vel.SetMatrix3d(0, ERROR_STATE_VEL_X_ERR_IDX, Matrix3d_::Identity());

	kf_.SequentialMeasurementUpdate(vel_err, vel_var, Hk_vel);
	CorrectNavigationStates();
}

void ErrorStateKalmanFilter::FuseWithAttitude(const double time, const Quaterniond& qnb, const Vector3d& att_var)
{
	double dt = sins_.time_ - time;
	Vector3d att_err = sins_.qnb_ - (qnb + sins_.wnb_*dt);
	
	MatrixNd Hk_att(3, kf_.states_num_, 0.0);
	Hk_att.SetMatrix3d(0, ERROR_STATE_ATT_X_ERR_IDX, Matrix3d_::Identity());

	kf_.SequentialMeasurementUpdate(att_err, att_var, Hk_att);
	CorrectNavigationStates();
}

void ErrorStateKalmanFilter::FuseWithAltitude(const double time, const double alt, const double alt_var)
{
	double dt = sins_.time_ - time;
	Vector3d pos_z_err = sins_.pos_.z - (alt + sins_.vn_.z*dt);
	
	VectorNd Hi_alt(kf_.states_num_, 0.0);
	Hi_alt(ERROR_STATE_POS_Z_ERR_IDX) = 1.0;

	kf_.SequentialMeasurementUpdate(pos_z_err, alt_var, Hi_alt);
	CorrectNavigationStates();
}

void ErrorStateKalmanFilter::FuseWithYaw(const double time, const double yaw, const double yaw_var)
{
	double dt = sins_.time_ - time;
	Vector3d qnb_yaw = Vector3d(0.0, 0.0, yaw);
	Vector3d att_err = sins_.qnb_ - (qnb_yaw + sins_.wnb_*dt);
	
	VectorNd Hi_yaw(kf_.states_num_, 0.0);
	Hi_yaw(ERROR_STATE_ATT_Z_ERR_IDX) = 1.0;

	kf_.SequentialMeasurementUpdate(att_err.z, yaw_var, Hi_yaw);
	CorrectNavigationStates();
}

void ErrorStateKalmanFilter::CorrectNavigationStates(void)
{
	/* (1) Kalman filter partial feedback strategy */
	kf_.PartialFeedback(kFilterUpdatePeriod); // Complete feedback
	StatesVectorToStatesElements();
    CovarianceMatrixToVarianceElements();

	/* (2) Feedback error state to SINS */
	sins_.qnb_ -= att_err_; 
	sins_.vn_  -= vel_err_;
	sins_.pos_ -= pos_err_;
	sins_.gyro_bias_ += gyro_bias_;
	sins_.acc_bias_  += acc_bias_;
}

void ErrorStateKalmanFilter::SetSystemMatrix(void)
{
	sins_.CalcErrorTransitionMatrix();
/*	Ft = [ Maa    Mav    Map    -Cnb	                O33 
           Mva    Mvv    Mvp     O33                    Cnb 
           O33    Mpv    Mpp     O33                    O33
           O33    O33    O33     diag(-1./sins.tauG)    O33
           O33    O33    O33     O33                    diag(-1./sins.tauA) ] */
	kf_.Ft.SetMatrix3d(0, 0, sins_.Maa_), 
	kf_.Ft.SetMatrix3d(0, 3, sins_.Mav_), 
	kf_.Ft.SetMatrix3d(0, 6, sins_.Map_), 
	kf_.Ft.SetMatrix3d(0, 9, -sins_.Cnb_); 
	kf_.Ft.SetMatrix3d(3, 0, sins_.Mva_), 
	kf_.Ft.SetMatrix3d(3, 3, sins_.Mvv_), 
	kf_.Ft.SetMatrix3d(3, 6, sins_.Mvp_), 
	kf_.Ft.SetMatrix3d(3, 12, sins_.Cnb_); 
	kf_.Ft.SetMatrix3d(6, 3, sins_.Mpv_), 
	kf_.Ft.SetMatrix3d(6, 6, sins_.Mpp_);

	kf_.Ft(9, 9)   = -(1 / sins_.tau_gyro_.x);
	kf_.Ft(10, 10) = -(1 / sins_.tau_gyro_.y);
	kf_.Ft(11, 11) = -(1 / sins_.tau_gyro_.z);
	kf_.Ft(12, 12) = -(1 / sins_.tau_acc_.x);
	kf_.Ft(13, 13) = -(1 / sins_.tau_acc_.y);   
	kf_.Ft(14, 14) = -(1 / sins_.tau_acc_.z);

/*  Gt = [ -Cnb    O33    O33     O33
            O33    Cnb    O33     O33
            O33    O33    O33     O33
            O33    O33    I33     O33
            O33    O33    O33     I33 ]; */
	kf_.Gt.SetMatrix3d(0, 0, -sins_.Cnb_);
	kf_.Gt.SetMatrix3d(3, 3, sins_.Cnb_);
	kf_.Gt.SetMatrix3d(9, 6, Matrix3d_::Identity());
	kf_.Gt.SetMatrix3d(12, 9, Matrix3d_::Identity());
}

void ErrorStateKalmanFilter::CalcOutputStates(void)
{
    output_states_.qnb_ = sins_.qnb_;
    output_states_.vn_ = sins_.vn_;
    output_states_.pos_ = sins_.pos_;
    output_states_.an_ = sins_.an_;
    output_states_.wnb_ = sins_.wnb_;
}

void ErrorStateKalmanFilter::ControlFusionModes(void)
{
    /* Below all are not real code */
    if (gnss_updated)
    {
        FuseWithPosition();
        FuseWithVelocity();
    }

    if (radar_updated)
    {
        FuseWithAttitude();
        FuseWithPosition();
    }

    if (baro_updated)
    {
        FuseWithAltitude();
    }
    
    if (rtk_yaw_updated)
    {
        FuseWithYaw();
    }

}

OutputStates::OutputStates(void)
{
    qnb_.SetIdentity();
    vn_.SetZero();
    pos_.SetZero();
    an_.SetZero();
    wnb_.SetZero();
}


} /* namspace imsf */
/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/


