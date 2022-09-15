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
