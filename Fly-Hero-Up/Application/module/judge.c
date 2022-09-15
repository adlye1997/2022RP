/**
  ******************************************************************************
  * @file           : judge.c\h
  * @brief          : 
  * @update         : finish 2022-2-13 20:06:34
  ******************************************************************************
  */

#include "judge.h"
#include "config.h"
#include "drv_can.h"
#include "string.h"
#include "gimbal.h"
#include "launcher.h"
#include "chassis.h"
#include "car.h"

extern CAN_HandleTypeDef hcan1;

judge_config_t judge_config = 
{
	.buffer_max = 50,
};
judge_base_info_t judge_base_info;
judge_info_t judge_info = 
{
	.offline_cnt_max = 1000,
};
judge_t judge = 
{
	.config = &judge_config,
	.base_info = &judge_base_info,
	.info = &judge_info,
};

uint8_t judge_tx_info[8];
uint8_t judge_tx_info_2[8];

void judge_init(judge_t *judge)
{
	judge->info->offline_cnt = judge->info->offline_cnt_max;
	judge->info->status = DEV_OFFLINE;
}

void judge_realtime_task(judge_t *judge)
{
	judge->info->offline_cnt++;
	if(judge->info->offline_cnt >= judge->info->offline_cnt_max)
	{
		judge_init(judge);
	}
}

void judge_send_task(judge_t *judge)
{
	if(car.move_mode_status == high_shoot_CAR)
	{
		memcpy(judge_tx_info, &launcher.config->friction_motor_work_speed, 2);
	}
	else 
	{
		memcpy(judge_tx_info, &gimbal.info->yaw_motor_angle, 2);
	}
	memcpy(&judge_tx_info[2], &gimbal.info->pitch_motor_angle, 2);
	memcpy(&judge_tx_info[4], &launcher.work_info->Ready_OR_Not, 1);
	if(chassis.info->cycle_mode == C_C_cycle)
	{
		judge_tx_info_2[5] = 1;
	}
	else 
	{
		judge_tx_info_2[5] = 0;
	}
	if((gimbal.info->pitch_mode == G_P_auto) || (gimbal.info->yaw_mode == G_Y_auto))
	{
		judge_tx_info_2[6] = 1;
	}
	else 
	{
		judge_tx_info_2[6] = 0;
	}
	CAN_SendData(&hcan1,0x300,judge_tx_info);
}
