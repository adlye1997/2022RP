/**
  ******************************************************************************
  * @file           : pid_feedback_system.c/h
  * @brief          : 
  * @note           : 
  ******************************************************************************
  */
	
#ifndef __PID_FEEDBACK_SYSTEM_H
#define __PID_FEEDBACK_SYSTEM_H

#include "stm32f4xx_hal.h"

typedef enum
{
	up_P,
	down_P,
}pid_feedback_system_base_status_e;  //pid����ϵͳ����״̬ö��

typedef enum
{
	init_P,
	work_P,
	finish_P,
}pid_feedback_system_status_e;  //pid����ϵͳ״̬ö��

typedef struct
{
	int16_t    target;
	int16_t    measure;
}pid_feedback_system_config_t;

typedef struct
{
	uint16_t   cnt;
	float      value_last;
	float      value_new;
	float      value_status_last;
	uint8_t    status_last;
	uint8_t    status_new;
	uint8_t    finish_time;
}pid_feedback_system_base_info_t;

typedef struct
{
	uint8_t    status;
	uint16_t   jump_times;    //����ʱ�𵴴���
	uint16_t   cnt_sum;       //����ʱ��
	uint16_t   max;           //��󳬵���
	uint16_t   max_per;       //��󳬵������ٷֱȣ�
}pid_feedback_system_info_t;

typedef struct
{
	pid_feedback_system_config_t *config;
	pid_feedback_system_base_info_t *base_info;
	pid_feedback_system_info_t *info;
}pid_feedback_system_t;

void pid_feedback_system_init(pid_feedback_system_t *sys);
void pid_feedback_system_base_info_init(pid_feedback_system_base_info_t *info);

void pid_feedback_system_ready(pid_feedback_system_t *sys,int16_t target,int16_t measure);
void pid_feedback_system_update(pid_feedback_system_t *sys, float value);
void pid_feedback_system_status_update(pid_feedback_system_t *sys);
void pid_feedback_system_jump_update(pid_feedback_system_t *sys);

#endif
