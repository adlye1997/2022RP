/**
  ******************************************************************************
  * @file           : imu.c\h
  * @brief          : 
  * @note           : finish 2022-2-13 19:59:35
  ******************************************************************************
  */

#ifndef __IMU_H
#define __IMU_H

#include "stm32f4xx_hal.h"

typedef struct 
{
	uint16_t dif_speed_fil_times;
	float Kp;
	float kp;
	float gyro_lowpass;
	float acce_lowpass;
}imu_config_t;

typedef struct 
{
	short ggx;							    //������x���ٶ�
	short ggy;							    //������y���ٶ�
	short ggz;							    //������z���ٶ�
	float ggx_K;						    //�����ǵ�ͨ�˲���x���ٶ�
	float ggy_K;						    //�����ǵ�ͨ�˲���y���ٶ�
	float ggz_K;						    //�����ǵ�ͨ�˲���z���ٶ�
	short aax;							    //���ٶȼ�x����ٶ�
	short aay;							    //���ٶȼ�y����ٶ�
	short aaz;							    //���ٶȼ�z����ٶ�
	float aax_K;						    //���ٶȼƵ�ͨ�˲���x����ٶ�
	float aay_K;						    //���ٶȼƵ�ͨ�˲���y����ٶ�
	float aaz_K;						    //���ٶȼƵ�ͨ�˲���z����ٶ�
	float pitch_last_;				  //��������һ��pitch��Ƕ�(�����Ǽ��ٶȼ�)
	float roll_last_;				    //��������һ��roll��Ƕ�(�����Ǽ��ٶȼ�)
	float yaw_last_;					  //��������һ��yaw��Ƕ�(�����Ǽ��ٶȼ�)
	float pitch;						    //������pitch��Ƕ�(+-90d)
	float roll;							    //������roll��Ƕ�(+-180d)
	float yaw;							    //������yaw��Ƕ�(+-180d)
	float pitch_;						    //������pitch��Ƕ�(+-180d)(�����Ǽ��ٶȼ�)
	float roll_;							  //������roll��Ƕ�(+-180d)(�����Ǽ��ٶȼ�)
	float yaw_;							    //������yaw��Ƕ�(+-180d)(�����Ǽ��ٶȼ�)
	float pitch_dif_speed;      //������pitch�����ٶ�
	float roll_dif_speed;       //������roll�����ٶ�
	float yaw_dif_speed;        //������yaw�����ٶ�
	float pitch_dif_speed_ave;  //������pitch�ᾭ����ֵ�˲������ٶ�
	float roll_dif_speed_ave;   //������roll�ᾭ����ֵ�˲������ٶ�
	float yaw_dif_speed_ave;    //������yaw�ᾭ����ֵ�˲������ٶ�
}imu_base_info_t;

/* ��������Ϣ */
typedef struct
{
	uint16_t init_cnt;
}imu_info_t;
 
/* ������ */
typedef struct
{
	imu_config_t *config;
	imu_base_info_t *base_info;
	imu_info_t *info;
}imu_t;

extern imu_t imu;

void imu_init(imu_t *imu);
void imu_info_init(imu_info_t *info);
void imu_update(imu_t *imu);

#endif
