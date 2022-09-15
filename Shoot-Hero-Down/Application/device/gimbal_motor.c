/**
  ******************************************************************************
  * @file           : gimbal_motor.c\h
	* @author         : czf
	* @date           : 2022-5-8 13:25:06 finish
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */
	
#include "gimbal_motor.h"
#include "config_can.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

motor_6020_base_info_t s_yaw_base_info;
motor_6020_info_t s_yaw_info;
motor_6020_config_t s_yaw_config = 
{
	.offline_cnt_max = 5,
};
extern pid_info_t s_yaw_pid_speed_info;
pid_t s_yaw_pid_speed = 
{
	.info = &s_yaw_pid_speed_info,
};
extern pid_info_t s_yaw_pid_angle_info;
pid_t s_yaw_pid_angle = 
{
	.info = &s_yaw_pid_angle_info,
};
drv_can_t s_yaw_can = 
{
	.hcan = &s_yaw_can_12,
	.rx_id = s_yaw_can_id,
};
motor_6020_t s_yaw = 
{
	.base_info = &s_yaw_base_info,
	.info = &s_yaw_info,
	.config = &s_yaw_config,
	.pid_speed = &s_yaw_pid_speed,
	.pid_angle = &s_yaw_pid_angle,
	.can = &s_yaw_can,
	.init = motor_6020_init,
	.update = motor_6020_update,
};

motor_2006_base_info_t s_pitch_base_info;
motor_2006_info_t s_pitch_info;
motor_2006_config_t s_pitch_config = 
{
	.offline_cnt_max = 5,
};
extern pid_info_t s_pitch_pid_speed_info;
pid_t s_pitch_pid_speed = 
{
	.info = &s_pitch_pid_speed_info,
};
extern pid_info_t s_pitch_pid_angle_info;
pid_t s_pitch_pid_angle = 
{
	.info = &s_pitch_pid_angle_info,
};
drv_can_t s_pitch_can = 
{
	.hcan = &s_pitch_can_12,
	.rx_id = s_pitch_can_id,
};
motor_2006_t s_pitch = 
{
	.base_info = &s_pitch_base_info,
	.info = &s_pitch_info,
	.config = &s_pitch_config,
	.pid_speed = &s_pitch_pid_speed,
	.pid_angle = &s_pitch_pid_angle,
	.can = &s_pitch_can,
	.init = motor_2006_init,
	.update = motor_2006_update,
	.ctrl = motor_2006_angle_ctrl,
};

motor_2006_base_info_t watch_base_info;
motor_2006_info_t watch_info;
motor_2006_config_t watch_config = 
{
	.offline_cnt_max = 5,
};
extern pid_info_t watch_pid_speed_info;
pid_t watch_pid_speed = 
{
	.info = &watch_pid_speed_info,
};
extern pid_info_t watch_pid_angle_info;
pid_t watch_pid_angle = 
{
	.info = &watch_pid_angle_info,
};
drv_can_t watch_can = 
{
	.hcan = &watch_can_12,
	.rx_id = watch_can_id,
};
motor_2006_t watch = 
{
	.base_info = &watch_base_info,
	.info = &watch_info,
	.config = &watch_config,
	.pid_speed = &watch_pid_speed,
	.pid_angle = &watch_pid_angle,
	.can = &watch_can,
	.init = motor_2006_init,
	.update = motor_2006_update,
	.ctrl = motor_2006_angle_ctrl,
};
