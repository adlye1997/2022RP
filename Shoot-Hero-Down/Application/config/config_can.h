/**
  ******************************************************************************
  * @file           : config_can.c\h
  * @brief          : 
  * @note           : finish 2022-2-12 12:07:05
  ******************************************************************************
  */
	
#ifndef __CONFIG_CAN_H
#define __CONFIG_CAN_H

#include "stm32f4xx_hal.h"

/*
#define example_can_12 hcan1
#define example_can_id 0x000
*/

#define chassis_LF_can_12 hcan1
#define chassis_RF_can_12 hcan1
#define chassis_LB_can_12 hcan1
#define chassis_RB_can_12 hcan1

#define dial_can_12 hcan2
#define pitch_can_12 hcan2
#define push_can_12 hcan2
#define yaw_can_12 hcan2

#define s_yaw_can_12 hcan1
#define s_pitch_can_12 hcan2
#define watch_can_12 hcan2

#define chassis_LF_can_id 0x201
#define chassis_RF_can_id 0x202
#define chassis_LB_can_id 0x203
#define chassis_RB_can_id 0x204

#define dial_can_id 0x201
#define yaw_can_id 0x205
#define pitch_can_id 0x206
#define push_can_id 0x207

#define s_pitch_can_id 0x202
#define s_yaw_can_id 0x206
#define watch_can_id 0x203


void CAN1_Get_Data(uint32_t id, uint8_t *data); //CAN1���պ���
void CAN2_Get_Data(uint32_t id, uint8_t *data); //CAN2���պ���

#endif
