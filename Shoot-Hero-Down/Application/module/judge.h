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
#include "judge_potocol.h"

typedef struct
{
//	int16_t chassis_power_buffer;           //底盘缓存功率
	int32_t chassis_out_put_max;            //底盘最大输出
//	uint16_t shooter_cooling_limit;					//机器人 42mm 枪口热量上限
//	uint16_t shooter_cooling_heat; 					//机器人 42mm 枪口热量
//	float   gimbal_yaw_angle;               //枪管yaw轴角度
//	float   bullet_speed;         					//子弹射速 单位 m/s
//	uint16_t chassis_power_limit;
  ext_rfid_status_t rfid_status;
	ext_game_status_t game_status;
	ext_game_robot_status_t game_robot_status;
	ext_power_heat_data_t power_heat_data;
	ext_shoot_data_t shoot_data;
	ext_game_robot_pos_t game_robot_pos;
	ext_robot_hurt_t ext_robot_hurt;
}judge_base_info_t;

typedef struct
{
	uint16_t offline_cnt_max;
	uint8_t status;
	uint16_t offline_cnt;
}judge_info_t;

typedef struct 
{
	judge_base_info_t *base_info;
	judge_info_t *info;
}judge_t;

extern judge_t judge;

void judge_init(judge_t *judge);
void judge_realtime_task(judge_t *judge);
void judge_send_task(judge_t *judge);

void judge_update(uint16_t id, uint8_t *rxBuf);

void cap_send_2F(void);
void cap_send_2E(void);


#endif
