/**
  ******************************************************************************
  * @file           : 2006_motor.c\h
	* @author         : czf
	* @date           : 2022-5-7 16:52:39 finish
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */

/**
  ******************************************************************************
  * 2006电机搭配C610电调参数：
	* 使用环境温度：0°C~55°C
	* 减速比：      36/1
	* 额定转速：    416rpm
  ******************************************************************************
  */
	
#ifndef __2006_Motor_H
#define __2006_Motor_H

#include "stm32f4xx_hal.h"
#include "pid.h"
#include "drv_can.h"

typedef struct 
{
	int16_t		angle;            //0~8191
	int16_t 	speed;            //RPM（max = 14400）
	int16_t 	torque;           //转矩
	int16_t   angle_add;        //-4096~4096
	int32_t   angle_sum;        //-2147683647~2147683647
}motor_2006_base_info_t;

typedef struct
{
	uint8_t offline_cnt;
	uint8_t status;
	int32_t target_angle_sum;
}motor_2006_info_t;

typedef struct 
{
	uint8_t offline_cnt_max;
}motor_2006_config_t;

typedef struct motor_2006_t
{
	motor_2006_base_info_t *base_info;
	motor_2006_info_t      *info;
	motor_2006_config_t    *config;
	pid_t                  *pid_speed;
	pid_t                  *pid_angle;
	drv_can_t              *can;
	void                  (*init)(struct motor_2006_t *self);
	void                  (*update)(struct motor_2006_t *self, uint8_t *rxBuf);
	void                  (*ctrl)(struct motor_2006_t *self);
}motor_2006_t;

void motor_2006_init(motor_2006_t *motor);
void motor_2006_info_init(motor_2006_t *motor);
void motor_2006_update(motor_2006_t *motor, uint8_t *rxBuf);
void motor_2006_speed_ctrl(motor_2006_t *motor);
void motor_2006_angle_ctrl(motor_2006_t *motor);

#endif
