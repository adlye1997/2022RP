/**
  ******************************************************************************
  * @file           : judge.h
  * @brief          : 
  * @update         : finish 2022-2-13 20:10:12
  ******************************************************************************
  */

#ifndef __JUDGE_H
#define __JUDGE_H

#include "stm32f4xx_hal.h"

typedef struct
{
	int16_t chassis_power_buffer;           //底盘缓存功率
	int32_t chassis_out_put_max;            //底盘最大输出
	uint16_t shooter_cooling_limit;					//机器人 42mm 枪口热量上限
	uint16_t shooter_cooling_heat; 					//机器人 42mm 枪口热量
	float   gimbal_yaw_angle;               //枪管yaw轴角度
	uint8_t car_color;                      //2蓝色 1红色
	uint8_t hurt_type;                      //伤害种类
	uint16_t chassis_power_limit;           //底盘功率限制
	uint16_t shooter_id1_42mm_speed_limit;  //射速上限
	uint8_t rfid;
}judge_base_info_t;

typedef struct
{
	uint16_t offline_cnt_max;
	uint8_t status;
	uint16_t offline_cnt;
}judge_info_t;

typedef struct
{
	int16_t buffer_max;
}judge_config_t;

typedef struct 
{
	judge_config_t *config;
	judge_base_info_t *base_info;
	judge_info_t *info;
}judge_t;

extern judge_t judge;

void judge_init(judge_t *judge);
void judge_realtime_task(judge_t *judge);
void judge_send_task(judge_t *judge);

#endif
