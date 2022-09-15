/**
  ******************************************************************************
  * @file           : judge.c\h
  * @brief          : 
  * @update         : finish 2022-2-13 20:06:34
  ******************************************************************************
  */

#include "judge.h"
#include "judge_potocol.h"
#include "cap_protocol.h"
#include "config.h"
#include "drv_can.h"
#include "string.h"
#include "gimbal.h"
#include "chassis.h"

extern CAN_HandleTypeDef hcan1;

judge_base_info_t judge_base_info;
judge_info_t judge_info = 
{
	.offline_cnt_max = 1000,
};
judge_t judge = 
{
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

void judge_update(uint16_t id, uint8_t *rxBuf)
{
	judge.info->offline_cnt = 0;
	judge.info->status = DEV_ONLINE;
	switch(id)
	{
		case ID_power_heat_data:
      memcpy(&judge.base_info->power_heat_data, rxBuf, LEN_power_heat_data);
			cap_send_2E();
			break;
		case ID_game_robot_state:
			memcpy(&judge.base_info->game_robot_status, rxBuf, LEN_game_robot_state);
			cap_send_2F();
			break;
		case ID_robot_hurt:
			if(rxBuf[0] & 0x0F == 0x04)
			{
				chassis.config->chassis_output_max -= 2000;
			}
			break;
		default:
			break;
	}
}

uint8_t cap_tx_buf_1[8]; //0x2E
uint8_t cap_tx_buf_2[8]; //0x2F
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void cap_send_2E(void)
{
	cap_tx_buf_1[0] = judge.base_info->power_heat_data.chassis_power_buffer >> 8;
	cap_tx_buf_1[1] = judge.base_info->power_heat_data.chassis_power_buffer;
	cap_tx_buf_1[2] = judge.base_info->power_heat_data.chassis_volt >> 8;
	cap_tx_buf_1[3] = judge.base_info->power_heat_data.chassis_volt;
	cap_tx_buf_1[4] = judge.base_info->power_heat_data.chassis_current >> 8;
	cap_tx_buf_1[5] = judge.base_info->power_heat_data.chassis_current;
	CAN_SendData(&hcan1, 0x2E, cap_tx_buf_1);
}

void cap_send_2F(void)
{
	uint16_t temp;
	cap_tx_buf_2[0] = judge.base_info->game_robot_status.chassis_power_limit >> 8;
	cap_tx_buf_2[1] = judge.base_info->game_robot_status.chassis_power_limit;
	temp = 300;
	cap_tx_buf_2[2] = temp >> 8;
	cap_tx_buf_2[3] = temp;
	temp = 150;
	cap_tx_buf_2[4] = temp >> 8;
	cap_tx_buf_2[5] = temp;
//	if(cap.Y_O_N == 1)
//	{
//		if(cap.record_Y_O_N == 1)
//		{
//			temp = 0x0700;
//		}
//		else 
//		{
//			temp = 0x0300;
//		}
//	}
//	else 
//	{
//		temp = 0x0000;
//	}
  temp = 0x0300;
	memcpy(&cap_tx_buf_2[6], &temp, 2);
	CAN_SendData(&hcan1, 0x2F, cap_tx_buf_2);
}
