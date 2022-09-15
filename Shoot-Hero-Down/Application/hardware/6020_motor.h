/**
  ******************************************************************************
  * @file           : 6020_motor.c\h
	* @author         : czf
	* @date           : 2022-5-7 22:19:55
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */

/**
  ******************************************************************************
  * 6020���������
	* �ת�٣�    469rpm
	* ���Ƶ�ѹ��Χ��-30000~30000��-20~20A��
  ******************************************************************************
  */

#ifndef __6020_MOTOR_H
#define __6020_MOTOR_H

#include "stm32f4xx_hal.h"
#include "drv_can.h"
#include "pid.h"

typedef struct 
{
	int16_t		angle;						//��е�Ƕȣ�0~8191��
	int16_t 	speed;						//ת�٣�RPM��
	int16_t 	current;					//ʵ��ת�ص���
	int16_t		temperature;			//����¶ȡ�C
	int16_t		angle_add;				//��е�Ƕ���ֵ��-4096~4096��
	int32_t		angle_sum;				//��е�Ƕ���ֵ��-2147683647~2147683647��
}motor_6020_base_info_t;

typedef struct
{
	uint8_t offline_cnt;      //���ʧ������
	uint8_t status;           //���״̬
	int32_t target_angle_sum; //Ŀ���е�Ƕ���ֵ
}motor_6020_info_t;

typedef struct
{
	uint8_t offline_cnt_max; //���ʧ���������ֵ
}motor_6020_config_t;

typedef struct motor_6020_t
{
	motor_6020_base_info_t *base_info; //������Ϣ
	motor_6020_info_t      *info;      //��Ϣ
	motor_6020_config_t    *config;    //����
	pid_t                  *pid_speed; //pid�ٶȻ�
	pid_t                  *pid_angle; //pid�ǶȻ�
	drv_can_t              *can;       //can����
	void                  (*init)(struct motor_6020_t *motor);                   //��ʼ������
	void                  (*update)(struct motor_6020_t *motor, uint8_t *rxBuf); //���º���
}motor_6020_t;

void motor_6020_init(motor_6020_t *motor);
void motor_6020_info_init(motor_6020_t *motor);
void motor_6020_update(motor_6020_t *motor, uint8_t *rxBuf);

#endif
