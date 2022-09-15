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
  * 3508电机搭配C620电调参数：
	* 减速比：      3591/187 = 157312.68/8192
	* 额定转速：    469rpm
	* 控制电流范围：-16384~16384（-20~20A）
  ******************************************************************************
  */

#ifndef __3508_Motor_H
#define __3508_Motor_H

#include "stm32f4xx_hal.h"
#include "pid.h"
#include "drv_can.h"

/* 3508电机基本信息 */
typedef struct
{
	int16_t angle;            //转子机械角度（0~8191）
	int16_t speed;            //转子转速（RPM）
	int16_t current;          //实际转矩电流
	int16_t temperature;      //电机温度（0~50°C）
	int16_t angle_add;        //转子机械角度增值（-4096~4096）
	int32_t angle_sum;        //转子机械角度总值（-2147683647~2147683647）
}motor_3508_base_info_t;

/* 3508电机信息 */
typedef struct 
{
	uint8_t offline_cnt;      //电机失联计数
	uint8_t status;           //电机状态
	int32_t target_angle_sum; //转子目标机械角度总值
}motor_3508_info_t;

/* 3508电机配置 */
typedef struct 
{
	uint8_t offline_cnt_max; //电机失联计数最大值
}motor_3508_config_t;

/* 3508电机 */
typedef struct motor_3508_t
{
	motor_3508_base_info_t *base_info; //基本信息
	motor_3508_info_t      *info;      //信息
	motor_3508_config_t    *config;    //配置
	pid_t                  *pid_speed; //pid速度环
	pid_t                  *pid_angle; //pid角度环
	drv_can_t              *can;       //can总线
	void                  (*init)(struct motor_3508_t *self);                   //初始化函数
	void                  (*update)(struct motor_3508_t *self, uint8_t *rxBuf); //更新函数
	void                  (*ctrl)(struct motor_3508_t *self);                   //控制函数
}motor_3508_t;

void motor_3508_init(motor_3508_t *motor);
void motor_3508_info_init(motor_3508_t *motor);
void motor_3508_update(motor_3508_t *motor, uint8_t *rxBuf);
void motor_3508_speed_ctrl(motor_3508_t *motor);
void motor_3508_angle_ctrl(motor_3508_t *motor);

#endif
