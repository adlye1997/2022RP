/**
  ******************************************************************************
  * @file           : up_com_protocol.c\h
	* @author         : czf
	* @date           : 2022-4-22 20:16:32
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */
	
#include "up_com_protocol.h"
#include "judge.h"
#include "string.h"
#include "drv_can.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

uint8_t up_com_tx_buf_1[8]; //power_heat
uint8_t up_com_tx_buf_2[8]; //game_robot_status
uint8_t up_com_tx_buf_3[8]; //shoot_data
uint8_t up_com_tx_buf_4[8]; //game_robot_pos
uint8_t up_com_tx_buf_5[8]; //robot_hurt

void up_send_power_heat(void)
{
	uint8_t rfid = judge.info->rfid_status.rfid_status;
	memcpy(up_com_tx_buf_1, (void*)&judge.info->power_heat_data.chassis_power_buffer, 2);
	memcpy(&up_com_tx_buf_1[2], (void*)&judge.info->power_heat_data.shooter_id1_42mm_cooling_heat, 2);
	if(rfid== 0x02)
	{
		up_com_tx_buf_1[4] = 1;
	}
	else
	{
		up_com_tx_buf_1[4] = 0;
	}
	CAN_SendData(&hcan1, 0x100, up_com_tx_buf_1);
}

void up_send_game_robot_state(void)
{
	memcpy(up_com_tx_buf_2, (void*)&judge.info->game_robot_status.shooter_id1_42mm_cooling_limit, 2);
	if(judge.info->game_robot_status.robot_id == 1)
	{
		up_com_tx_buf_2[2] = 1;
	}
	else if(judge.info->game_robot_status.robot_id == 101)
	{
		up_com_tx_buf_2[2] = 2;
	}
	memcpy(&up_com_tx_buf_2[3], (void*)&judge.info->game_robot_status.chassis_power_limit, 2);
	memcpy(&up_com_tx_buf_2[5], (void*)&judge.info->game_robot_status.shooter_id1_42mm_speed_limit, 2);
	CAN_SendData(&hcan1, 0x101, up_com_tx_buf_2);
}

void up_send_shoot(void)
{
	memcpy(up_com_tx_buf_3, (void*)&judge.info->shoot_data.bullet_speed, 4);
	CAN_SendData(&hcan1, 0x102, up_com_tx_buf_3);
}

void up_send_robot_pos(void)
{
	memcpy(up_com_tx_buf_4, (void*)&judge.info->game_robot_pos.yaw, 4);
	CAN_SendData(&hcan1, 0x103, up_com_tx_buf_4);
}

void up_send_robot_hurt(void)
{
	if(judge.info->ext_robot_hurt.hurt_type == 0x02 || \
		 judge.info->ext_robot_hurt.hurt_type == 0x04)
	{
		up_com_tx_buf_5[0] = judge.info->ext_robot_hurt.hurt_type;
		CAN_SendData(&hcan1, 0x104, up_com_tx_buf_5);
	}
}

