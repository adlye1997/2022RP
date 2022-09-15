/**
  ******************************************************************************
  * @file           : 3508_motor.c\h
  * @brief          : 
  * @update         : 2022��1��15��13:25:54
  ******************************************************************************
  */

	/**
  ******************************************************************************
  * 3508�������C620���������
	* ʹ�û����¶ȣ�0��C~50��C
	* ���ٱȣ�      3591/187 = 157312.68/8192
  ******************************************************************************
  */


#ifndef __3508_Motor_H
#define __3508_Motor_H

#include "stm32f4xx_hal.h"
#include "pid.h"
#include "drv_can.h"

typedef struct
{
	int16_t   angle;            //����Ƕȣ�0~8191��
	int16_t   speed;            //���ת�٣�RPM����max = 7600��
	int16_t   current;          //ת�ص���-16384~16384��-20~20A��
	int16_t   temperature;      //��C
	int16_t   angle_add;        //-4096~4096
	int32_t   angle_sum;        //-2147683647~2147683647
}motor_3508_base_info_t;

typedef struct 
{
	uint8_t          offline_cnt;
	uint8_t          offline_cnt_max;
	uint8_t          status;
	int32_t          target_angle_sum;
}motor_3508_info_t;

typedef struct motor_3508_t
{
	motor_3508_base_info_t *base_info;
	motor_3508_info_t      *info;
	pid_t                  *pid_speed;
	pid_t                  *pid_angle;
	drv_can_t              *can;
	void                  (*init)(struct motor_3508_t *motor);
	void                  (*update)(struct motor_3508_t *motor, uint8_t *rxBuf);
	void                  (*ctrl)(struct motor_3508_t *motor);
}motor_3508_t;

void motor_3508_init(motor_3508_t *motor);
void motor_3508_info_init(motor_3508_info_t *info);
void motor_3508_update(motor_3508_t *motor, uint8_t *rxBuf);
void motor_3508_speed_ctrl(motor_3508_t *motor);
void motor_3508_angle_ctrl(motor_3508_t *motor);

#endif
