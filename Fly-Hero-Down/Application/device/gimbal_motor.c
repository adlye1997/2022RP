/**
  ******************************************************************************
  * @file           : gimbal_motor.c\h
  * @brief          : 
  * @note           : finish 2022-2-13 19:35:34
  ******************************************************************************
  */
	
#include "gimbal_motor.h"

motor_6020_base_info_t gimbal_motor_yaw_base_info;
motor_6020_info_t gimbal_motor_yaw_info;
extern pid_t gimbal_motor_yaw_speed_pid;
extern pid_t gimbal_imu_yaw_speed_pid;
extern pid_t gimbal_motor_yaw_angle_pid;
extern pid_t gimbal_imu_yaw_angle_pid;
extern drv_can_t gimbal_motor_yaw_can;
motor_6020_t gimbal_yaw_motor = 
{
	.base_info = &gimbal_motor_yaw_base_info,
	.info = &gimbal_motor_yaw_info,
	.motor_pid_speed = &gimbal_motor_yaw_speed_pid,
	.imu_pid_speed = &gimbal_imu_yaw_speed_pid,
	.motor_pid_angle = &gimbal_motor_yaw_angle_pid,
	.imu_pid_angle = &gimbal_imu_yaw_angle_pid,
	.can = &gimbal_motor_yaw_can,
	.init = motor_6020_init,
	.update = motor_6020_update,
};

motor_6020_base_info_t gimbal_motor_pitch_base_info;
motor_6020_info_t gimbal_motor_pitch_info;
extern pid_t gimbal_motor_pitch_speed_pid;
extern pid_t gimbal_imu_pitch_speed_pid;
extern pid_t gimbal_motor_pitch_angle_pid;
extern pid_t gimbal_imu_pitch_angle_pid;
extern drv_can_t gimbal_motor_pitch_can;
motor_6020_t gimbal_pitch_motor = 
{
	.base_info = &gimbal_motor_pitch_base_info,
	.info = &gimbal_motor_pitch_info,
	.motor_pid_speed = &gimbal_motor_pitch_speed_pid,
	.imu_pid_speed = &gimbal_imu_pitch_speed_pid,
	.motor_pid_angle = &gimbal_motor_pitch_angle_pid,
	.imu_pid_angle = &gimbal_imu_pitch_angle_pid,
	.can = &gimbal_motor_pitch_can,
	.init = motor_6020_init,
	.update = motor_6020_update,
};
