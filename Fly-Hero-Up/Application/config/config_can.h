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
#define example_id 0x000
*/

/* 底盘电机can接收id */
#define chassis_motor_LF_id 0x201
#define chassis_motor_RF_id 0x202
#define chassis_motor_LB_id 0x203
#define chassis_motor_RB_id 0x204

/* 云台电机can接收id */
#define gimbal_motor_pitch_id 0x205
#define gimbal_motor_yaw_id   0x208

/* 发射机构电机can接收id */
#define dial_motor_id           0x207
#define position_motor_id       0x202
#define friction_left_motor_id  0x206
#define friction_right_motor_id 0x207

/* 下主控can接收id */
#define power_heat_data   0x100
#define game_robot_status 0x101
#define shoot_data        0x102
#define game_robot_pos    0x103
#define robot_hurt        0x104

void CAN1_Get_Data(uint32_t id, uint8_t *data); //CAN1接收函数
void CAN2_Get_Data(uint32_t id, uint8_t *data); //CAN2接收函数

#endif
