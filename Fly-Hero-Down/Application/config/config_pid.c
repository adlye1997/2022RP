/**
  ******************************************************************************
  * @file           : config_pid.c\h
  * @brief          : 
  * @note           : 2022��1��15��13:32:03
  ******************************************************************************
  */

#include "config_pid.h"
#include "config.h"

/* 
pid_info_t example =
{
	.kp = 0,
	.ki = 0,
	.kd = 0,
	.integral_bias = 0,
	.integral_max = 0,
	.out_max = 0,
};
*/

/* ���̵��pid��Ϣ */
pid_info_t chassis_motor_pid_info[4] = 
{
	[LF] = 
	{
		.kp = 8,
		.ki = 0.33,
		.out_max = 12500,
		.integral_max = 6000,
	},
	[RF] = 
	{
		.kp = 8,
		.ki = 0.33,
		.out_max = 12500,
		.integral_max = 6000,
	},
	[LB] = 
	{
		.kp = 8,
		.ki = 0.33,
		.out_max = 12500,
		.integral_max = 6000,
	},
	[RB] = 
	{
		.kp = 8,
		.ki = 0.33,
		.out_max = 12500,
		.integral_max = 6000,
	},
};

/* ��̨Yaw���pid�ٶȻ���Ϣ */
pid_info_t gimbal_motor_yaw_pid_speed_info = 
{
	.kp = -500,
	.ki = -7,
	.kd = 0,
	.out_max = 28000,
	.integral_max = 4000,
};
	
/* ��̨Yaw������pid�ٶȻ���Ϣ */
pid_info_t gimbal_imu_yaw_pid_speed_info = 
{
	.kp = -550,
	.ki = -3,
	.kd = 0,
	.out_max = 28000,
	.integral_max = 4000,
};

/* ��̨Yaw���pidλ�û���Ϣ */
pid_info_t gimbal_motor_yaw_pid_angle_info = 
{
	.kp = 0.75,
	.ki = 0,
	.kd = 0,
	.out_max = 700,
	.integral_max = 0,
};

/* ��̨Yaw������pidλ�û���Ϣ */
pid_info_t gimbal_imu_yaw_pid_angle_info = 
{
	.kp = 13,
	.ki = 0,
	.kd = 0,
	.out_max = 500,
	.integral_max = 0,
};

/* ��̨Pitch���pid�ٶȻ���Ϣ */
pid_info_t gimbal_motor_pitch_pid_speed_info = 
{
	.kp = -500,//-500
	.ki = -5,//-5
	.kd = 0,
	.out_max = 28000,
	.integral_bias = -2800,//-2800
	.integral_max = 5000,//3000
};

/* ��̨Pitch������pid�ٶȻ���Ϣ */
pid_info_t gimbal_imu_pitch_pid_speed_info = 
{
	.kp = -450,//
	.ki = -1,//
	.kd = -1500,//
	.out_max = 28000,
	.integral_bias = -3000,//
	.integral_max = 5000,//
};

/* ��̨Pitch���pidλ�û���Ϣ */
pid_info_t gimbal_motor_pitch_pid_angle_info = 
{
	.kp = 1,//16
	.ki = 0,
	.out_max = 500,
	.integral_max = 0,
};

/* ��̨Pitch������pidλ�û���Ϣ */
pid_info_t gimbal_imu_pitch_pid_angle_info = 
{
	.kp = 15,//16
	.ki = 0,
	.out_max = 500,
	.integral_max = 0,
};

/* ���̵��pidλ�û���Ϣ */
pid_info_t dial_motor_pid_angle_info = 
{
	.kp = 0.2,
	.out_max = 3000,  //6000
};

/* ���̵��pid�ٶȻ���Ϣ */
pid_info_t dial_motor_pid_speed_info = 
{
	.kp = 10,
	.ki = 0.4,
	.integral_max = 10000,
	.out_max = 0,
};

/* Ħ��������pid�ٶȻ���Ϣ */
pid_info_t friction_left_motor_pid_speed_info = 
{
	.kp = 8,
	.ki = 0.33,
	.integral_max = 6000,
	.out_max = 10000,
};

/* Ħ�����ҵ��pid�ٶȻ���Ϣ */
pid_info_t friction_right_motor_pid_speed_info = 
{
	.kp = 8,
	.ki = 0.33,
	.integral_max = 6000,
	.out_max = 10000,
};

/* ��λ�ֵ��pid�ٶȻ���Ϣ */
pid_info_t position_motor_pid_speed_info = 
{
	.kp = 15,
	.ki = 0.4,
	.integral_max = 10000,
	.out_max = 9000,
};
