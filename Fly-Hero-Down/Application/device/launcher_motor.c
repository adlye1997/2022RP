/**
  ******************************************************************************
  * @file           : launcher_motor.c\h
  * @brief          : 
  * @note           : finish 2022-2-13 19:36:06
  ******************************************************************************
  */

#include "launcher_motor.h"

motor_3508_base_info_t dial_motor_base_info;
motor_3508_info_t dial_motor_info;
extern pid_t dial_motor_pid_speed;
extern pid_t dial_motor_pid_angle;
extern drv_can_t dial_motor_can;
motor_3508_t dial_motor = 
{
	.base_info = &dial_motor_base_info,
	.info = &dial_motor_info,
	.pid_speed = &dial_motor_pid_speed,
	.pid_angle = &dial_motor_pid_angle,
	.can = &dial_motor_can,
	.init = motor_3508_init,
	.update = motor_3508_update,
	.ctrl = motor_3508_angle_ctrl,
};

motor_3508_base_info_t friction_left_motor_base_info;
motor_3508_info_t friction_left_motor_info;
extern pid_t friction_left_motor_pid_speed;
extern drv_can_t friction_left_motor_can;
motor_3508_t friction_left_motor = 
{
	.base_info = &friction_left_motor_base_info,
	.info = &friction_left_motor_info,
	.pid_speed = &friction_left_motor_pid_speed,
	.can = &friction_left_motor_can,
	.init = motor_3508_init,
	.update = motor_3508_update,
	.ctrl = motor_3508_speed_ctrl,
};

motor_3508_base_info_t friction_right_motor_base_info;
motor_3508_info_t friction_right_motor_info;
extern pid_t friction_right_motor_pid_speed;
extern drv_can_t friction_right_motor_can;
motor_3508_t friction_right_motor = 
{
	.base_info = &friction_right_motor_base_info,
	.info = &friction_right_motor_info,
	.pid_speed = &friction_right_motor_pid_speed,
	.can = &friction_right_motor_can,
	.init = motor_3508_init,
	.update = motor_3508_update,
	.ctrl = motor_3508_speed_ctrl,
};

motor_2006_base_info_t position_motor_base_info;
motor_2006_info_t position_motor_info;
extern pid_t position_motor_pid_speed;
extern drv_can_t position_motor_can;
motor_2006_t position_motor = 
{
	.base_info = &position_motor_base_info,
	.info = &position_motor_info,
	.pid_speed = &position_motor_pid_speed,
	.can = &position_motor_can,
	.init = motor_2006_init,
	.update = motor_2006_update,
	.ctrl = motor_2006_speed_ctrl,
};
