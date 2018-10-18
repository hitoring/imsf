/*
 ******************************************************************************
 * @file    math_calib.h
 * @author  Peng Zhang
 * @version V1.0.0
 * @date    2017-06-19
 * @brief   This file provides a complete calibration routine for IMU,
 *          the calibration routine consist of bias, scale and misalignment
 *          calibration.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "math_calib.h"

/* Private function prototypes -----------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static imu_calib_param_t imu_calib_param;
static cube_calib_collect_data_t collect_data;

/* Private functions ---------------------------------------------------------*/

/*
 * @brief  Core algorithm of IMU calibration routine, calculate the bias, scale and misalignment
 *         of IMU using the collected data
 * @param  None
 * @note   Accel sensor model: A = Ma * (Ra - Ba)
 *         Gyro sensor model:  G = Mg * (Rg - Bg)
 * @retval Calculation status
 */
static bool calc_imu_calib_param(void)
{
    vec3f_t data_x, data_y, data_z;
    vec3f_t data_sum;
    mat3f_t mat_tmp;
    bool status;
    
    /* (1) Ma = inv([(Ra(x+)-Ra(x-))/2 (Ra(y+)-Ra(y-))/2 (Ra(z+)-Ra(z-))/2]) */
    data_x = vec3f_sub(collect_data.accel_mean_x_positive, collect_data.accel_mean_x_negative);
    data_x = vec3f_mul_factor(data_x, 0.5f);
    data_y = vec3f_sub(collect_data.accel_mean_y_positive, collect_data.accel_mean_y_negative);
    data_y = vec3f_mul_factor(data_y, 0.5f);
    data_z = vec3f_sub(collect_data.accel_mean_z_positive, collect_data.accel_mean_z_negative);
    data_z = vec3f_mul_factor(data_z, 0.5f);
    mat3f_fill_column(&mat_tmp, data_x, data_y, data_z);
    status = mat3f_inv(&mat_tmp, &(imu_calib_param.Ma));
    if (status == false)
    {
        imu_calib_param.calibed = 0;      // calib failed
        mat3f_eye(&(imu_calib_param.Ma)); // reset Ma to eye(3)
        printf("matrix Ma singularity!!\r\n");
        return false;
    }

    /* (2) Ba = (Ra(x+) + Ra(x-) + Ra(y+) + Ra(y-) + Ra(z+) + Ra(z-)) / 6 */
    data_x = vec3f_add(collect_data.accel_mean_x_positive, collect_data.accel_mean_x_negative);
    data_y = vec3f_add(collect_data.accel_mean_y_positive, collect_data.accel_mean_y_negative);
    data_z = vec3f_add(collect_data.accel_mean_z_positive, collect_data.accel_mean_z_negative);
    data_sum = vec3f_add(data_x, data_y);
    data_sum = vec3f_add(data_sum, data_z);
    imu_calib_param.Ba = vec3f_mul_factor(data_sum, 1.0f/6.0f);

    /* (3) Bg = (Rg(x+) + Rg(x-) + Rg(y+) + Rg(y-) + Rg(z+) + Rg(z-)) / 6 */
    data_x = vec3f_add(collect_data.gyro_mean_x_positive, collect_data.gyro_mean_x_negative);
    data_y = vec3f_add(collect_data.gyro_mean_y_positive, collect_data.gyro_mean_y_negative);
    data_z = vec3f_add(collect_data.gyro_mean_z_positive, collect_data.gyro_mean_z_negative);
    data_sum = vec3f_add(data_x, data_y);
    data_sum = vec3f_add(data_sum, data_z);
    imu_calib_param.Bg = vec3f_mul_factor(data_sum, 1.0f/6.0f);

    /* (4) Mg - eye(3), not calc for now */
    mat3f_eye(&(imu_calib_param.Mg)); // reset Mg to eye(3)

    imu_calib_param.calibed = 1; // calib successfuly
    return true;
}

/*
 * @brief  Init imu_calib_param struct and collect_data struct
 * @param  None
 * @note   None
 * @retval NOne
 */
void cube_calib_init(void)
{
    /* (1) init imu_calib_param struct */
    mat3f_eye(&(imu_calib_param.Ma));
    mat3f_eye(&(imu_calib_param.Mg));
    imu_calib_param.Ba = vec3f_zeros();
    imu_calib_param.Bg = vec3f_zeros();
    imu_calib_param.calibed = 0;
    
    /* (2) init collect_data_ready status */
    collect_data.collect_data_ready = ((uint8_t)0);
}

/*
 * @brief  IMU cube calibration process state machine
 * @param  
 * @note   Replace current character command with a standard USART calibration protocol subsequently!
 * @retval None
 */
void cube_calib_state_machine(uint8_t calib_cmd)
{
    uint16_t i;
    vec3f_t accel_raw, accel_raw_sum;
    vec3f_t gyro_raw, gyro_raw_sum;
    
    switch (calib_cmd)
    {
        case CUBE_X_POSITIVE_UP:
        {
            accel_raw_sum = vec3f_zero();
            gyro_raw_sum = vec3f_zero();
            for (i=0; i<CUBE_CALIB_SAMPLE_COUNT; i++)
            {
                mpu6500_get_accel_raw_data(&accel_raw);
                accel_raw_sum = vec3f_add(accel_raw_sum, accel_raw);
                mpu6500_get_gyro_raw_data(&gyro_raw);
                gyro_raw_sum = vec3f_add(gyro_raw_sum, gyro_raw);

                Delay_Ms(CUBE_CALIB_DELAY_MS);
            }

            collect_data.accel_mean_x_positive = vec3f_mul_factor(accel_raw_sum, 1.0f/CUBE_CALIB_SAMPLE_COUNT);
            collect_data.gyro_mean_x_positive = vec3f_mul_factor(gyro_raw_sum, 1.0f/CUBE_CALIB_SAMPLE_COUNT);
            collect_data.collect_data_ready |= ((uint8_t)(1 << 0));

            printf("the receive byte is %d\r\n", calib_cmd);
            printf("the accel_mean_x_positive is [%.4f, %.4f, %.4f]\r\n", collect_data.accel_mean_x_positive.x, collect_data.accel_mean_x_positive.y, collect_data.accel_mean_x_positive.z);
            printf("the gyro_mean_x_positive  is [%.4f, %.4f, %.4f]\r\n", collect_data.gyro_mean_x_positive.x, collect_data.gyro_mean_x_positive.y, collect_data.gyro_mean_x_positive.z);

            break;
        }
        
        case CUBE_X_NEGATIVE_UP:
        {
            accel_raw_sum = vec3f_zero();
            gyro_raw_sum = vec3f_zero();
            for (i=0; i<CUBE_CALIB_SAMPLE_COUNT; i++)
            {
                mpu6500_get_accel_raw_data(&accel_raw);
                accel_raw_sum = vec3f_add(accel_raw_sum, accel_raw);
                mpu6500_get_gyro_raw_data(&gyro_raw);
                gyro_raw_sum = vec3f_add(gyro_raw_sum, gyro_raw);

                Delay_Ms(CUBE_CALIB_DELAY_MS);
            }
            collect_data.accel_mean_x_negative = vec3f_mul_factor(accel_raw_sum, 1.0f/CUBE_CALIB_SAMPLE_COUNT);
            collect_data.gyro_mean_x_negative = vec3f_mul_factor(gyro_raw_sum, 1.0f/CUBE_CALIB_SAMPLE_COUNT);
            collect_data.collect_data_ready |= ((uint8_t)(1 << 1)); 

            printf("the receive byte is %d\r\n", calib_cmd);
            printf("the accel_mean_x_negative is [%.4f, %.4f, %.4f]\r\n", collect_data.accel_mean_x_negative.x, collect_data.accel_mean_x_negative.y, collect_data.accel_mean_x_negative.z);
            printf("the gyro_mean_x_negative  is [%.4f, %.4f, %.4f]\r\n", collect_data.gyro_mean_x_negative.x, collect_data.gyro_mean_x_negative.y, collect_data.gyro_mean_x_negative.z);

            break;
        }

        case CUBE_Y_POSITIVE_UP:
        {
            accel_raw_sum = vec3f_zero();
            gyro_raw_sum = vec3f_zero();
            for (i=0; i<CUBE_CALIB_SAMPLE_COUNT; i++)
            {
                mpu6500_get_accel_raw_data(&accel_raw);
                accel_raw_sum = vec3f_add(accel_raw_sum, accel_raw);
                mpu6500_get_gyro_raw_data(&gyro_raw);
                gyro_raw_sum = vec3f_add(gyro_raw_sum, gyro_raw);

                Delay_Ms(CUBE_CALIB_DELAY_MS);
            }
            collect_data.accel_mean_y_positive = vec3f_mul_factor(accel_raw_sum, 1.0f/CUBE_CALIB_SAMPLE_COUNT);
            collect_data.gyro_mean_y_positive = vec3f_mul_factor(gyro_raw_sum, 1.0f/CUBE_CALIB_SAMPLE_COUNT);
            collect_data.collect_data_ready |= ((uint8_t)(1 << 2));

            printf("the receive byte is %d\r\n", calib_cmd);
            printf("the accel_mean_y_positive is [%.4f, %.4f, %.4f]\r\n", collect_data.accel_mean_y_positive.x, collect_data.accel_mean_y_positive.y, collect_data.accel_mean_y_positive.z);
            printf("the gyro_mean_y_positive  is [%.4f, %.4f, %.4f]\r\n", collect_data.gyro_mean_y_positive.x, collect_data.gyro_mean_y_positive.y, collect_data.gyro_mean_y_positive.z);

            break;
        }

        case CUBE_Y_NEGATIVE_UP:
        {
            accel_raw_sum = vec3f_zero();
            gyro_raw_sum = vec3f_zero();
            for (i=0; i<CUBE_CALIB_SAMPLE_COUNT; i++)
            {
                mpu6500_get_accel_raw_data(&accel_raw);
                accel_raw_sum = vec3f_add(accel_raw_sum, accel_raw);
                mpu6500_get_gyro_raw_data(&gyro_raw);
                gyro_raw_sum = vec3f_add(gyro_raw_sum, gyro_raw);

                Delay_Ms(CUBE_CALIB_DELAY_MS);
            }
            collect_data.accel_mean_y_negative = vec3f_mul_factor(accel_raw_sum, 1.0f/CUBE_CALIB_SAMPLE_COUNT);
            collect_data.gyro_mean_y_negative = vec3f_mul_factor(gyro_raw_sum, 1.0f/CUBE_CALIB_SAMPLE_COUNT);
            collect_data.collect_data_ready |= ((uint8_t)(1 << 3));

            printf("the receive byte is %d\r\n", calib_cmd);
            printf("the accel_mean_y_negative is [%.4f, %.4f, %.4f]\r\n", collect_data.accel_mean_y_negative.x, collect_data.accel_mean_y_negative.y, collect_data.accel_mean_y_negative.z);
            printf("the gyro_mean_y_negative  is [%.4f, %.4f, %.4f]\r\n", collect_data.gyro_mean_y_negative.x, collect_data.gyro_mean_y_negative.y, collect_data.gyro_mean_y_negative.z);

            break;
        }

        
        case CUBE_Z_POSITIVE_UP:
        {
            accel_raw_sum = vec3f_zero();
            gyro_raw_sum = vec3f_zero();
            for (i=0; i<CUBE_CALIB_SAMPLE_COUNT; i++)
            {
                mpu6500_get_accel_raw_data(&accel_raw);
                accel_raw_sum = vec3f_add(accel_raw_sum, accel_raw);
                mpu6500_get_gyro_raw_data(&gyro_raw);
                gyro_raw_sum = vec3f_add(gyro_raw_sum, gyro_raw);

                Delay_Ms(CUBE_CALIB_DELAY_MS);
            }
            collect_data.accel_mean_z_positive = vec3f_mul_factor(accel_raw_sum, 1.0f/CUBE_CALIB_SAMPLE_COUNT);
            collect_data.gyro_mean_z_positive = vec3f_mul_factor(gyro_raw_sum, 1.0f/CUBE_CALIB_SAMPLE_COUNT);
            collect_data.collect_data_ready |= ((uint8_t)(1 << 4));

            printf("the receive byte is %d\r\n", calib_cmd);
            printf("the accel_mean_z_positive is [%.4f, %.4f, %.4f]\r\n", collect_data.accel_mean_z_positive.x, collect_data.accel_mean_z_positive.y, collect_data.accel_mean_z_positive.z);
            printf("the gyro_mean_z_positive  is [%.4f, %.4f, %.4f]\r\n", collect_data.gyro_mean_z_positive.x, collect_data.gyro_mean_z_positive.y, collect_data.gyro_mean_z_positive.z);

            break;
        }

        case CUBE_Z_NEGATIVE_UP:
        {
            accel_raw_sum = vec3f_zero();
            gyro_raw_sum = vec3f_zero();
            for (i=0; i<CUBE_CALIB_SAMPLE_COUNT; i++)
            {
                mpu6500_get_accel_raw_data(&accel_raw);
                accel_raw_sum = vec3f_add(accel_raw_sum, accel_raw);
                mpu6500_get_gyro_raw_data(&gyro_raw);
                gyro_raw_sum = vec3f_add(gyro_raw_sum, gyro_raw);

                Delay_Ms(CUBE_CALIB_DELAY_MS);
            }
            collect_data.accel_mean_z_negative = vec3f_mul_factor(accel_raw_sum, 1.0f/CUBE_CALIB_SAMPLE_COUNT);
            collect_data.gyro_mean_z_negative = vec3f_mul_factor(gyro_raw_sum, 1.0f/CUBE_CALIB_SAMPLE_COUNT);
            collect_data.collect_data_ready |= ((uint8_t)(1 << 5));

            printf("the receive byte is %d\r\n", calib_cmd);
            printf("the accel_mean_z_negative is [%.4f, %.4f, %.4f]\r\n", collect_data.accel_mean_z_negative.x, collect_data.accel_mean_z_negative.y, collect_data.accel_mean_z_negative.z);
            printf("the gyro_mean_z_negative  is [%.4f, %.4f, %.4f]\r\n", collect_data.gyro_mean_z_negative.x, collect_data.gyro_mean_z_negative.y, collect_data.gyro_mean_z_negative.z);

            break;
        }

        case CALC_CALIB_PARAM:
        {
            if (collect_data.collect_data_ready == 63)  // all data collected
            {
                if(calc_imu_calib_param())
                {
                    printf("(1) the accel calib matrix is [ %.6f, %.6f, %.6f\r\n                                %.6f, %.6f, %.6f\r\n                                %.6f, %.6f, %.6f ]\r\n\r\n", 
                    imu_calib_param.Ma.a.x, imu_calib_param.Ma.b.x, imu_calib_param.Ma.c.x, imu_calib_param.Ma.a.y, imu_calib_param.Ma.b.y, imu_calib_param.Ma.c.y, imu_calib_param.Ma.a.z, imu_calib_param.Ma.b.z, imu_calib_param.Ma.c.z);
                    printf("(2) the accel bias vector is  [ %.5f, %.5f, %.5f]\r\n\r\n", imu_calib_param.Ba.x, imu_calib_param.Ba.y, imu_calib_param.Ba.z);
                    printf("(3) the gyro calib matrix is  [ %.6f, %.6f, %.6f\r\n                                %.6f, %.6f, %.6f\r\n                                %.6f, %.6f, %.6f ]\r\n\r\n", 
                    imu_calib_param.Mg.a.x, imu_calib_param.Mg.b.x, imu_calib_param.Mg.c.x, imu_calib_param.Mg.a.y, imu_calib_param.Mg.b.y, imu_calib_param.Mg.c.y, imu_calib_param.Mg.a.z, imu_calib_param.Mg.b.z, imu_calib_param.Mg.c.z);
                    printf("(4) the gyro bias vector is   [ %.5f, %.5f, %.5f]\r\n\r\n", imu_calib_param.Bg.x, imu_calib_param.Bg.y, imu_calib_param.Bg.z);

                }
                else
                {
                    printf("Can't calculate the calibration parameters, because matrix Ma singularity!!\r\n");
                }
            }
            else
            {
                printf("IMU data not fully collected!!\r\n");
            }

            break;      
        }

        case CALIB_PARAM_COMP:
        {
            mpu6500_get_accel_raw_data(&accel_raw);
            mpu6500_get_gyro_raw_data(&gyro_raw);
            
            if (imu_calib_param.calibed)
            {
                /* (1) Accel calibration parameters compensation: (Ra - Ba) */
                accel_raw = vec3f_sub(accel_raw, imu_calib_param.Ba);
                /* (2) Accel calibration parameters compensation: Ma * (Ra - Ba) */
                accel_raw = mat3f_mul_vec3f(&(imu_calib_param.Ma), accel_raw);
                
                /* (3) Gyro calibration parameters compensation: (Rg - Bg) */
                gyro_raw = vec3f_sub(gyro_raw, imu_calib_param.Bg);
                /* (4) Gyro calibration parameters compensation: Mg * (Rg - Bg) */
                gyro_raw = mat3f_mul_vec3f(&(imu_calib_param.Mg), gyro_raw);

                printf("(1) the calibed accel data is [ %.5f, %.5f, %.5f]\r\n", accel_raw.x, accel_raw.y, accel_raw.z);
                printf("(2) the calibed gyro data is  [ %.5f, %.5f, %.5f]\r\n", gyro_raw.x, gyro_raw.y, gyro_raw.z);
            }
            else
            {
                printf("(1) the raw accel data is [ %.5f, %.5f, %.5f]\r\n", accel_raw.x, accel_raw.y, accel_raw.z);
                printf("(2) the raw gyro data is  [ %.5f, %.5f, %.5f]\r\n", gyro_raw.x, gyro_raw.y, gyro_raw.z);
            }

            Delay_Ms(1000);
            break;
        }

        default:
        {
            printf("invalid calibration command!");
            break;
        }

    }
}


/******************* (C) COPYRIGHT 2017 PENG ZHANG *****END OF FILE********/

