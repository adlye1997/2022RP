/**
  ******************************************************************************
  * @file           : launcher.c/h
  * @brief          : 
  * @note           : finish 2022-3-3 16:25:29
  ******************************************************************************
  */

#include "stm32f4xx_hal.h"
#include "3508_motor.h"
#include "2006_motor.h"
#include "stdbool.h"

typedef enum 
{
	Reload_D,  //����
	F_lock_D,  //���򿨵�
	Unload_D,  //�˵�
	B_lock_D,  //���򿨵�
	Keep_D,    //ά��
}dial_work_status_e;

typedef struct 
{
	uint8_t commond;
	uint8_t status;
	uint16_t work_times;
	uint16_t lock_times;
	uint16_t cnt;
	int16_t angle_temp;
}dial_work_info_t;

typedef enum 
{
	Stop_L,     //ֹͣ
	Ready_L,    //׼�����
	Shoot_L,    //���
	Relax_L,    //�ȴ�ָ��
	Done_L,     //ָ��ִ�����
}launcher_work_status_e;

typedef enum
{
  L_work,
  L_stop,
}launcher_mode_e;

typedef struct
{
	int16_t friction_motor_work_speed;
	int16_t friction_16ms_work_speed;
	int16_t friction_10ms_work_speed;
	int16_t position_motor_work_speed;
	int16_t position_motor_work_cnt;
	int16_t position_motor_work_cnt_add;
	int16_t dial_motor_work_delay_cnt;
	int16_t dial_front_output_max;
	int16_t dial_back_output_max;
	int16_t dial_keep_output_max;
	int16_t auto_shoot_T;
	int16_t Reload_Times;
	int16_t F_lock_Times;
	int16_t Unload_Times;
	int16_t B_lock_Times;
	int16_t Reload_angle_check;
	int32_t Reload_angle;
	int16_t Unload_angle_check;
	int32_t Unload_angle;
	int16_t F_lock_angle_check;
	int16_t F_lock_angle;
	int16_t B_lock_angle_check;
	int16_t B_lock_angle;
	int16_t Reload_cnt;
	int16_t F_lock_cnt;
	int16_t B_lock_cnt;
	int16_t Unload_cnt;
}launcher_config_t;

typedef struct 
{
	dial_work_info_t *dial_work_info;
	uint8_t commond;
	uint8_t status;
	uint8_t Ready_OR_Not;
	int16_t cnt;
	int16_t shoot_cnt;
}launcher_work_info_t;

typedef struct 
{
	uint8_t mode;
	bool auto_shoot_Y_O_N;
}launcher_info_t;

typedef struct 
{
	motor_3508_t  *dial;
	motor_3508_t  *friction_left;   //��ת�����
	motor_3508_t  *friction_right;  //��ת�����
	motor_2006_t  *position;
	
	launcher_config_t *config;
	launcher_info_t *info;
	launcher_work_info_t *work_info;
}launcher_t;

extern launcher_t launcher;

void launcher_init(launcher_t *launcher);
void launcher_commond_init(launcher_t *launcher);
void launcher_work_info_init(launcher_work_info_t *info);
void dial_work_init(dial_work_info_t *info);

void launcher_ctrl_task(launcher_t *launcher);
void launcher_mode_update(launcher_t *launcher);
void launcher_commond_respond(launcher_t *launcher);
void launcher_work(launcher_t *launcher);

void dial_work(dial_work_info_t *info);
void dial_work_to_keep(dial_work_info_t *info);

void judge_limit(launcher_t *launcher);

void launcher_motor_speed_change(uint16_t speed);

void launcher_tick_task(launcher_t *launcher);
