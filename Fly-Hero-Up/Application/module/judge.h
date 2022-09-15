/**
  ******************************************************************************
  * @file           : judge.h
  * @brief          : 
  * @update         : finish 2022-2-13 20:10:12
  ******************************************************************************
  */

#ifndef __JUDGE_H
#define __JUDGE_H

#include "stm32f4xx_hal.h"

typedef struct
{
	int16_t chassis_power_buffer;           //���̻��湦��
	int32_t chassis_out_put_max;            //����������
	uint16_t shooter_cooling_limit;					//������ 42mm ǹ����������
	uint16_t shooter_cooling_heat; 					//������ 42mm ǹ������
	float   gimbal_yaw_angle;               //ǹ��yaw��Ƕ�
	uint8_t car_color;                      //2��ɫ 1��ɫ
	uint8_t hurt_type;                      //�˺�����
	uint16_t chassis_power_limit;           //���̹�������
	uint16_t shooter_id1_42mm_speed_limit;  //��������
	uint8_t rfid;
}judge_base_info_t;

typedef struct
{
	uint16_t offline_cnt_max;
	uint8_t status;
	uint16_t offline_cnt;
}judge_info_t;

typedef struct
{
	int16_t buffer_max;
}judge_config_t;

typedef struct 
{
	judge_config_t *config;
	judge_base_info_t *base_info;
	judge_info_t *info;
}judge_t;

extern judge_t judge;

void judge_init(judge_t *judge);
void judge_realtime_task(judge_t *judge);
void judge_send_task(judge_t *judge);

#endif
