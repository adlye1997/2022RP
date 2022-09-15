/**
  ******************************************************************************
  * @file           : chassis_motor.c\h
  * @brief          : 
  * @note           : finish 2022-2-13 19:35:23
  ******************************************************************************
  */

#include "chassis_motor.h"
#include "config.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

motor_3508_base_info_t chassis_motor_base_info[4];
motor_3508_info_t chassis_motor_info[4];
extern pid_t chassis_motor_pid[4];
extern drv_can_t chassis_motor_can[4];
motor_3508_t chassis_motor[4] = 
{
	[LF] = 
	{
		.base_info = &chassis_motor_base_info[LF],
		.info = &chassis_motor_info[LF],
		.pid_speed = &chassis_motor_pid[LF],
		.can = &chassis_motor_can[LF],
		.init = motor_3508_init,
		.update = motor_3508_update,
		.ctrl = motor_3508_speed_ctrl,
	},
	[LB] = 
	{
		.base_info = &chassis_motor_base_info[LB],
		.info = &chassis_motor_info[LB],
		.pid_speed = &chassis_motor_pid[LB],
		.can = &chassis_motor_can[LB],
		.init = motor_3508_init,
		.update = motor_3508_update,
		.ctrl = motor_3508_speed_ctrl,
	},
	[RF] = 
	{
		.base_info = &chassis_motor_base_info[RF],
		.info = &chassis_motor_info[RF],
		.pid_speed = &chassis_motor_pid[RF],
		.can = &chassis_motor_can[RF],
		.init = motor_3508_init,
		.update = motor_3508_update,
		.ctrl = motor_3508_speed_ctrl,
	},
	[RB] = 
	{
		.base_info = &chassis_motor_base_info[RB],
		.info = &chassis_motor_info[RB],
		.pid_speed = &chassis_motor_pid[RB],
		.can = &chassis_motor_can[RB],
		.init = motor_3508_init,
		.update = motor_3508_update,
		.ctrl = motor_3508_speed_ctrl,
	},
};
