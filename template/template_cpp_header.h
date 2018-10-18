/*
 *********************************************************************************************************
 * @file    template_cpp_header.h
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-09
 * @note    This file is based on PSINS library written by Prof. Gongmin Yan @ NWPU, 
            great thanks and respect to Prof. Yan!
 * @brief   This file defines a ClassExample class
            for intelligent multi-sensor fusion library.
 *********************************************************************************************************
 */

/* Define to prevent recursive inclusion ----------------------------------------------------------------*/
#ifndef TEMPLATE_HEADER_H_
#define TEMPLATE_HEADER_H_

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
class ClassExample
{
public:
	/* Member variables */
	
	/* Constructors */
	ClassExample(void);
	
	/* Destructor */
	~ClassExample();
	
	/* Member functions */
	void Init(void);
	void Update(void);
};




} /* namespace imsf */

#endif /* TEMPLATE_HEADER_H_ */

/************************************* (C) COPYRIGHT 2017 PENG ZHANG ******************END OF FILE********/

