/**
  ******************************************************************************
  * @file           : config_pid.c\h
  * @brief          : 
  * @note           : 2022Äê1ÔÂ15ÈÕ13:32:03
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

pid_info_t chassis_LF_pid_speed_info =
{
	.kp = 8,
	.ki = 0.33,
	.kd = 0,
	.integral_bias = 0,
	.integral_max = 6000,
	.out_max = 16000,
};

pid_info_t chassis_RF_pid_speed_info =
{
	.kp = 8,
	.ki = 0.33,
	.kd = 0,
	.integral_bias = 0,
	.integral_max = 6000,
	.out_max = 16000,
};

pid_info_t chassis_LB_pid_speed_info =
{
	.kp = 8,
	.ki = 0.33,
	.kd = 0,
	.integral_bias = 0,
	.integral_max = 6000,
	.out_max = 16000,
};

pid_info_t chassis_RB_pid_speed_info =
{
	.kp = 8,
	.ki = 0.33,
	.kd = 0,
	.integral_bias = 0,
	.integral_max = 6000,
	.out_max = 16000,
};

pid_info_t pitch_pid_speed_info =
{
	.kp = 8,
	.ki = 0.33,
	.kd = 0,
	.integral_bias = 0,
	.integral_max = 6000,
	.out_max = 12500,
};

pid_info_t pitch_pid_angle_info =
{
	.kp = 0.1,
	.ki = 0,
	.kd = 0,
	.integral_bias = 0,
	.integral_max = 0,
	.out_max = 5000,
};

pid_info_t yaw_pid_speed_info =
{
	.kp = 15,
	.ki = 0.4,
	.kd = 0,
	.integral_bias = 0,
	.integral_max = 10000,
	.out_max = 9000,
};

pid_info_t yaw_pid_angle_info =
{
	.kp = 0.1,
	.ki = 0,
	.kd = 0,
	.integral_bias = 0,
	.integral_max = 0,
	.out_max = 9000,
};

pid_info_t push_pid_speed_info =
{
	.kp = 8,
	.ki = 0.33,
	.kd = 0,
	.integral_bias = 0,
	.integral_max = 6000,
	.out_max = 12500,
};

pid_info_t push_pid_angle_info =
{
	.kp = 0.1,
	.ki = 0,
	.kd = 0,
	.integral_bias = 0,
	.integral_max = 0,
	.blind_err = 100,
	.out_max = 5000,
};

pid_info_t dial_pid_speed_info =
{
	.kp = 8,
	.ki = 0.33,
	.kd = 0,
	.integral_bias = 0,
	.integral_max = 15000,
	.out_max = 10000,
};

pid_info_t dial_pid_angle_info =
{
	.kp = 0.1,
	.ki = 0,
	.kd = 0,
	.integral_bias = 0,
	.integral_max = 0,
	.out_max = 3000,
};

pid_info_t s_yaw_pid_speed_info =
{
	.kp = 150,
	.ki = 2,
	.kd = 0,
	.integral_bias = 0,
	.integral_max = 5000,
	.out_max = 28000,
};

pid_info_t s_yaw_pid_angle_info =
{
	.kp = 0.05,
	.ki = 0,
	.kd = 0,
	.integral_bias = 0,
	.integral_max = 0,
	.out_max = 100,
};

pid_info_t s_pitch_pid_speed_info =
{
	.kp = 15,
	.ki = 0.4,
	.kd = 0,
	.integral_bias = 0,
	.integral_max = 10000,
	.out_max = 9000,
};

pid_info_t s_pitch_pid_angle_info =
{
	.kp = 0.1,
	.ki = 0,
	.kd = 0,
	.integral_bias = 0,
	.integral_max = 0,
	.out_max = 1000,
};

pid_info_t watch_pid_speed_info =
{
	.kp = 15,
	.ki = 0.4,
	.kd = 0,
	.integral_bias = 0,
	.integral_max = 10000,
	.out_max = 9000,
};

pid_info_t watch_pid_angle_info =
{
	.kp = 0.1,
	.ki = 0,
	.kd = 0,
	.integral_bias = 0,
	.integral_max = 0,
	.out_max = 1000,
};
