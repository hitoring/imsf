/*
 *********************************************************************************************************
 * @file    imsf_sins_earth.h
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @note    This file is based on PSINS library written by Prof. Gongmin Yan @ NWPU, 
            great thanks and respect to Prof. Yan!
 * @brief   This file defines a earth parameter class 
            for intelligent multi-sensor fusion library.
 *********************************************************************************************************
 */

/* Define to prevent recursive inclusion ----------------------------------------------------------------*/
#ifndef IMSF_SINS_EARTH_H_
#define IMSF_SINS_EARTH_H_

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

class EarthParam
{
public: 
    /* Member variables */
    double time_; // timestamp
    double ts_; // sample time
    
    double Rm;  // the Earth's mean radius
    double Re;  // the Earth's semi-major radius
    double Rp;  // the Earth's semi-minor radius
    double f;   // flattening
    double e;   // 1st eccentricity
    double e2;  // the square of 1st eccentricity
    double ep;  // 2nd eccentricity
    double ep2; // the square of 2nd eccentricity
    double wie; // the Earth's spin rate
    double g0;  // gravity force
    
    double sl;
    double sl2;
    double sl4;
    double cl;
    double tl;
    double RMh; 
    double RNh; 
    double clRNh;
    double f_RMh;
    double f_RNh;
    double f_clRNh;
    Vector3d gn;
    Vector3d gcc;
    Vector3d wnie;
    Vector3d wnen;
    Vector3d wnin;

    Vector3d pos_;
    Vector3d vn_;

    /* Constructors */
    EarthParam(void);
    
    /* Destructor */
    ~EarthParam();

    /* Member functions */
    void Init(void);
    void Update(const Vector3d& pos, const Vector3d& vn);
    Vector3d VelToDeltaPos(const Vector3d& vn, const double ts) const;
};

extern const double kEarthMeanRadius; 
extern const double kEarthSemiMajorRadius;
extern const double kEarthFlattening;
extern const double kEarthSpinRate;
extern const double kGravityForce;


} /* namespace imsf */

#endif /* IMSF_SINS_EARTH_H_ */

/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/


