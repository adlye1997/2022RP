/**
  ******************************************************************************
  * @file           : 3508_motor.c\h
	* @author         : czf
	* @date           : 2022-5-7 22:07:29 finish
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */

/**
  ******************************************************************************
  * 3508�������C620���������
	* ���ٱȣ�      3591/187 = 157312.68/8192
	* �ת�٣�    469rpm
	* ���Ƶ�����Χ��-16384~16384��-20~20A��
  ******************************************************************************
  */

#ifndef __3508_Motor_H
#define __3508_Motor_H

#include "stm32f4xx_hal.h"
#include "pid.h"
#include "drv_can.h"

/* 3508���������Ϣ */
typedef struct
{
	int16_t angle;            //ת�ӻ�е�Ƕȣ�0~8191��
	int16_t speed;            //ת��ת�٣�RPM��
	int16_t current;          //ʵ��ת�ص���
	int16_t temperature;      //����¶ȣ�0~50��C��
	int16_t angle_add;        //ת�ӻ�е�Ƕ���ֵ��-4096~4096��
	int32_t angle_sum;        //ת�ӻ�е�Ƕ���ֵ��-2147683647~2147683647��
}motor_3508_base_info_t;

/* 3508�����Ϣ */
typedef struct 
{
	uint8_t offline_cnt;      //���ʧ������
	uint8_t status;           //���״̬
	int32_t target_angle_sum; //ת��Ŀ���е�Ƕ���ֵ
}motor_3508_info_t;

/* 3508������� */
typedef struct 
{
	uint8_t offline_cnt_max; //���ʧ���������ֵ
}motor_3508_config_t;

/* 3508��� */
typedef struct motor_3508_t
{
	motor_3508_base_info_t *base_info; //������Ϣ
	motor_3508_info_t      *info;      //��Ϣ
	motor_3508_config_t    *config;    //����
	pid_t                  *pid_speed; //pid�ٶȻ�
	pid_t                  *pid_angle; //pid�ǶȻ�
	drv_can_t              *can;       //can����
	void                  (*init)(struct motor_3508_t *self);                   //��ʼ������
	void                  (*update)(struct motor_3508_t *self, uint8_t *rxBuf); //���º���
	void                  (*ctrl)(struct motor_3508_t *self);                   //���ƺ���
}motor_3508_t;

void motor_3508_init(motor_3508_t *motor);
void motor_3508_info_init(motor_3508_t *motor);
void motor_3508_update(motor_3508_t *motor, uint8_t *rxBuf);
void motor_3508_speed_ctrl(motor_3508_t *motor);
void motor_3508_angle_ctrl(motor_3508_t *motor);

#endif
