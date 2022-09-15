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
  * 6020电机参数：
	* 额定转速：    469rpm
	* 控制电压范围：-30000~30000（-20~20A）
  ******************************************************************************
  */

#ifndef __6020_MOTOR_H
#define __6020_MOTOR_H

#include "stm32f4xx_hal.h"
#include "drv_can.h"
#include "pid.h"

typedef struct 
{
	int16_t		angle;						//机械角度（0~8191）
	int16_t 	speed;						//转速（RPM）
	int16_t 	current;					//实际转矩电流
	int16_t		temperature;			//电机温度°C
	int16_t		angle_add;				//机械角度增值（-4096~4096）
	int32_t		angle_sum;				//机械角度总值（-2147683647~2147683647）
}motor_6020_base_info_t;

typedef struct
{
	uint8_t offline_cnt;      //电机失联计数
	uint8_t status;           //电机状态
	int32_t target_angle_sum; //目标机械角度总值
}motor_6020_info_t;

typedef struct
{
	uint8_t offline_cnt_max; //电机失联计数最大值
}motor_6020_config_t;

typedef struct motor_6020_t
{
	motor_6020_base_info_t *base_info; //基本信息
	motor_6020_info_t      *info;      //信息
	motor_6020_config_t    *config;    //配置
	pid_t                  *pid_speed; //pid速度环
	pid_t                  *pid_angle; //pid角度环
	drv_can_t              *can;       //can总线
	void                  (*init)(struct motor_6020_t *motor);                   //初始化函数
	void                  (*update)(struct motor_6020_t *motor, uint8_t *rxBuf); //更新函数
}motor_6020_t;

void motor_6020_init(motor_6020_t *motor);
void motor_6020_info_init(motor_6020_t *motor);
void motor_6020_update(motor_6020_t *motor, uint8_t *rxBuf);

#endif
