/**
  ******************************************************************************
  * @file           : pid.c\h
  * @brief          : 
  * @note           : finish 2022-2-13 19:19:47
  ******************************************************************************
  */

#include "pid.h"
#include "math.h"
#include "rp_math.h"
#include "math_support.h"
#include "config.h"
#include "config_pid.h"

/*
pid_t example = 
{
	.info = &example,
	.init = pid_init,
	.cal = single_pid_cal,
};
*/

extern pid_info_t chassis_motor_pid_info[4];
/* ���̵��pid */
pid_t chassis_motor_pid[4] = 
{
	[LF] = 
	{
		.info = &chassis_motor_pid_info[LF],
	},
	[RF] = 
	{
		.info = &chassis_motor_pid_info[RF],
	},
	[LB] = 
	{
		.info = &chassis_motor_pid_info[LB],
	},
	[RB] = 
	{
		.info = &chassis_motor_pid_info[RB],
	},
};

extern pid_info_t gimbal_motor_yaw_pid_speed_info;
/* ��̨Yaw���pid�ٶȻ� */
pid_t gimbal_motor_yaw_speed_pid = 
{
	.info = &gimbal_motor_yaw_pid_speed_info,
};

extern pid_info_t gimbal_imu_yaw_pid_speed_info;
/* ��̨Yaw������pid�ٶȻ� */
pid_t gimbal_imu_yaw_speed_pid = 
{
	.info = &gimbal_imu_yaw_pid_speed_info,
};

extern pid_info_t gimbal_motor_yaw_pid_angle_info;
/* ��̨Yaw���pidλ�û� */
pid_t gimbal_motor_yaw_angle_pid = 
{
	.info = &gimbal_motor_yaw_pid_angle_info,
};

extern pid_info_t gimbal_imu_yaw_pid_angle_info;
/* ��̨Yaw������pidλ�û� */
pid_t gimbal_imu_yaw_angle_pid = 
{
	.info = &gimbal_imu_yaw_pid_angle_info,
};

extern pid_info_t gimbal_motor_pitch_pid_speed_info;
/* ��̨Pitch���pid�ٶȻ� */
pid_t gimbal_motor_pitch_speed_pid = 
{
	.info = &gimbal_motor_pitch_pid_speed_info,
};

extern pid_info_t gimbal_imu_pitch_pid_speed_info;
/* ��̨Pitch���pid�ٶȻ� */
pid_t gimbal_imu_pitch_speed_pid = 
{
	.info = &gimbal_imu_pitch_pid_speed_info,
};

extern pid_info_t gimbal_motor_pitch_pid_angle_info;
/* ��̨Pitch���pidλ�û� */
pid_t gimbal_motor_pitch_angle_pid = 
{
	.info = &gimbal_motor_pitch_pid_angle_info,
};

extern pid_info_t gimbal_imu_pitch_pid_angle_info;
/* ��̨Pitch���pidλ�û� */
pid_t gimbal_imu_pitch_angle_pid = 
{
	.info = &gimbal_imu_pitch_pid_angle_info,
};

extern pid_info_t dial_motor_pid_angle_info;
/* ���̵��pidλ�û� */
pid_t dial_motor_pid_angle = 
{
	.info = &dial_motor_pid_angle_info,
};

extern pid_info_t dial_motor_pid_speed_info;
/* ���̵��pid�ٶȻ� */
pid_t dial_motor_pid_speed = 
{
	.info = &dial_motor_pid_speed_info,
};

extern pid_info_t friction_left_motor_pid_speed_info;
/* Ħ��������pid�ٶȻ� */
pid_t friction_left_motor_pid_speed = 
{
	.info = &friction_left_motor_pid_speed_info,
};

extern pid_info_t friction_right_motor_pid_speed_info;
/* Ħ�����ҵ��pid�ٶȻ� */
pid_t friction_right_motor_pid_speed = 
{
	.info = &friction_right_motor_pid_speed_info,
};

extern pid_info_t position_motor_pid_speed_info;
/* ��λ�ֵ��pid�ٶȻ� */
pid_t position_motor_pid_speed = 
{
	.info = &position_motor_pid_speed_info,
};

void pid_init(pid_t *pid)
{
	pid->info->target = 0;
	pid->info->measure = 0;
	pid->info->err = 0;
	pid->info->last_err = 0;
	pid->info->pout = 0;
	pid->info->iout = 0;
	pid->info->dout = 0;
	pid->info->out = 0;
	pid->info->integral = 0;
}

void single_pid_cal(pid_info_t *pid)
{
	pid->err = pid->target - pid->measure;
	if(abs(pid->err)<=(pid->blind_err))
		pid->err = 0;
	// ����
	pid->integral += pid->err;
	pid->integral = constrain(pid->integral, -pid->integral_max+pid->integral_bias, pid->integral_max+pid->integral_bias);
	// p i d ��������
	pid->pout = pid->kp * pid->err;
	pid->iout = pid->ki * pid->integral;
	pid->dout = pid->kd * (pid->err - pid->last_err);
	// �ۼ�pid���ֵ
	pid->out = pid->pout + pid->iout + pid->dout;
	pid->out = constrain(pid->out, -pid->out_max, pid->out_max);
	// ��¼�ϴ����ֵ
	pid->last_err = pid->err;
}

void nonlinear_pid_cal(pid_info_t *pid)
{
	pid->err = pid->target - pid->measure;
	if(abs(pid->err)<=(pid->blind_err))
		pid->err = 0;
	// ����
	pid->integral += pid->err;
	pid->integral = constrain(pid->integral, -pid->integral_max+pid->integral_bias, pid->integral_max+pid->integral_bias);
	// p i d ��������
	pid->pout = - pid->kp * fal(pid->err, 0.5, 50);
	pid->iout = - pid->ki * fal(pid->integral, 0.1, 50);
	pid->dout = pid->kd * (pid->err - pid->last_err);
	// �ۼ�pid���ֵ
	pid->out = pid->pout + pid->iout + pid->dout;
	pid->out = constrain(pid->out, -pid->out_max, pid->out_max);
	// ��¼�ϴ����ֵ
	pid->last_err = pid->err;
}

float fal(float err, float a, float d)
{
	if(abs(err) > d)
	{
		return (pow(abs(err),a) * sgn(err));
	}
	else 
	{
		return(err/pow(d,1-a));
	}
}
