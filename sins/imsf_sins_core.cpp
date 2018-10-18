/*
 *********************************************************************************************************
 * @file    imsf_sins_core.cpp
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @note    This file is based on PSINS library written by Prof. Gongmin Yan @ NWPU, 
            great thanks and respect to Prof. Yan!
 * @brief   This file defines some member functions of SINS class
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
#include "imsf_sins_core.h"
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

SINS::SINS(void)
{
    earth_param_ = EarthParam();
    imu_ = IMU(); 
    
    time_ = 0.0;
    nts_ = 0.0；

    qnb_.SetIdentity();
    qnb0_.SetIdentity();
    Cnb_.SetIdentity();
    Cbn_.SetIdentity();
    euler_angle_.SetZero();
    pos_.SetZero();
    vn_.SetZero();
    vb_.SetZero();
    an_.SetZero();
    fn_.SetZero();
    fb_.SetZero();
    wib_.SetZero();
    web_.SetZero();
    wnb_.SetZero();
    
    Kg_.SetIdentity();
    Ka_.SetIdentity();
    gyro_bias_.SetZero();
    acc_bias_.SetZero();
    tau_gyro_.SetInf();
    tau_acc_.SetInf();
    
    Maa_.SetZero();
    Mav_.SetZero();
    Map_.SetZero();
    Mva_.SetZero();
    Mvv_.SetZero();
    Mvp_.SetZero();
    Mpv_.SetZero();
    Mpp_.SetZero();
}

SINS::~SINS()
{
}

void SINS::Init(const Quaterniond& qnb0, const Vector3d& vn0, const Vector3d& pos0, const int subsamples, const double nts)
{ 
    earth_param_.Init();
    nts_ = nts;
    double sub_ts = nts_ / subsamples;
    imu_.Init(subsamples, sub_ts);
    qnb_ = qnb0; 
    qnb0_ = qnb0;
    vn_ = vn0, 
    pos_ = pos0;

    earth_param_.Update(pos0, vn0);
    Cnb_ = QuatToDCM(qnb_); 
    Cbn_ = ~Cnb_; 
    euler_angle_ = DCMToAngle(Cnb_);
    vb_ = Cbn_*vn_;

    CalcErrorTransitionMatrix();
}

void SINS::Update(const Vector3d* delta_angle, const Vector3d* delta_vel)
{
#ifdef USE_LOW_GRADE_MEMS_IMU
    imu_.Update(delta_angle, delta_vel);
    imu_.phim_ = Kg_*imu_.phim_ - gyro_bias_*nts_; 
    imu_.dvbm_ = Ka_*imu_.dvbm_ - acc_bias_*nts_;  // IMU calibration
    earth_param_.Update(pos_, vn_);
    wib_ = imu_.phim_/nts_; 
    fb_ = imu_.dvbm_/nts_;
    web_ = wib_;
    wnb_ = wib_;
    fn_ = qnb_*fb_;
    an_ = fn_ + earth_param_.gcc;
    Vector3d vn_cur = vn_ + an_*nts_;
    //pos_ = pos_ + earth_param_.VelToDeltaPos((vn_+vn_cur)/2, nts_);   
    pos_ = pos_ + Mpv_*(vn_+vn_cur)*nts_/2;
    vn_ = vn_cur;
    
    qnb_ = qnb_*RVToQuat(imu_.phim_);
    Cnb_ = QuatToDCM(qnb_); 
    Cbn_ = ~Cnb_;
    euler_angle_ = DCMToAngle(Cnb_);
    vb_ = Cbn_*vn_;
#else
    imu_.Update(delta_angle, delta_vel);
    imu_.phim_ = Kg_*imu_.phim_ - gyro_bias_*nts_; 
    imu_.dvbm_ = Ka_*imu_.dvbm_ - acc_bias_*nts_;  // IMU calibration
    
    Vector3d vn_mid = vn_ + an_*nts_/2;
    //Vector3d pos_mid = pos_ + earth_param_.VelToDeltaPos(vn_mid, nts_/2);
    Vector3d pos_mid = pos_ + Mpv_*vn_mid*nts_/2;
    
    earth_param_.Update(pos_mid, vn_mid);
    
    wib_ = imu_.phim_/nts_; 
    fb_ = imu_.dvbm_/nts_;
    web_ = wib_ - Cbn_*earth_param_.wnie;
    wnb_ = wib_ - (qnb_*RVToQuat(imu_.phim_/2))*earth_param_.wnin;
    fn_ = qnb_*fb_;
    an_ = RVToQuat(-earth_param_.wnin*nts_/2)*fn_ + earth_param_.gcc;
    Vector3d vn_cur = vn_ + an_*nts_;
    //pos_ = pos_ + earth_param_.VelToDeltaPos((vn_+vn_cur)/2, nts_);   
    pos_ = pos_ + Mpv_*(vn_+vn_cur)*nts_/2;
    vn_ = vn_cur;
    
    qnb_ = RVToQuat(-earth_param_.wnin*nts_)*qnb_*RVToQuat(imu_.phim_);
    Cnb_ = QuatToDCM(qnb_);
    Cbn_ = ~Cnb_;
    euler_angle_ = DCMToAngle(Cnb_);
    vb_ = Cbn_*vn_;
#endif
}

void SINS::CalcErrorTransitionMatrix(void)
{
#ifdef USE_LOW_GRADE_MEMS_IMU
        Mva_ = Matrix3d_::Askew(fn_);
        Mpv_ = Matrix3d(0,earth_param_.f_RMh,0, earth_param_.f_clRNh,0,0, 0,0,1);
#else
        double tl = earth_param_.tl, secl = 1.0/earth_param_.cl, secl2 = secl*secl;
        double scl = earth_param_.sl*earth_param_.cl;
        double wN = earth_param_.wnie.y, wU = earth_param_.wnie.z;
        double vE = vn_.x, vN = vn_.y;
        double f_RMh = earth_param_.f_RMh, f_RMh2 = f_RMh*f_RMh;
        double f_RNh = earth_param_.f_RNh, f_RNh2 = f_RNh*f_RNh;
        double f_clRNh = earth_param_.f_clRNh;
        
        Matrix3d Avn = Matrix3d_::Askew(vn_);
        Matrix3d Mp1(0,0,0, -wU,0,0, wN,0,0);
        Matrix3d Mp2(0,0,vN*f_RMh2, 0,0,-vE*f_RNh2, vE*secl2*f_RNh,0,-vE*tl*f_RNh2);
        
        Maa_ = Matrix3d_::Askew(-earth_param_.wnin);
        Mav_ = Matrix3d(0,-f_RMh,0, f_RNh,0,0, tl*f_RNh,0,0);
        Map_ = Mp1+Mp2;
        Mva_ = Matrix3d_::Askew(fn_);
        Mvv_ = Avn*Mav_ - Matrix3d_::Askew(earth_param_.wnie+earth_param_.wnin);
        Mvp_ = Avn*(Mp1+Map_);
        Mvp_.c.x = Mvp_.c.x - earth_param_.g0*(5.27094e-3*2*scl+2.32718e-5*4*earth_param_.sl2*scl); 
        Mvp_.c.z = Mvp_.c.z + 3.086e-6;
        Mpv_ = Matrix3d(0,f_RMh,0, f_clRNh,0,0, 0,0,1);
        Mpp_ = Matrix3d(0,0,-vN*f_RMh2, vE*tl*f_clRNh,0,-vE*secl*f_RNh2, 0,0,0);
#endif
}


} /* namspace imsf */
/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/


