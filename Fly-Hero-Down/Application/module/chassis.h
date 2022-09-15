/**
  ******************************************************************************
  * @file           : chassis.c/h
  * @brief          : 
  * @note           : finish 2022-2-11 12:33:21
	*                   2022-3-17 ���ӵ������ýṹ�壬����С����ģʽ��ת��ģʽ��
	*                             �������ýṹ�壬���ӵ����ٶ�target�仯ģʽ��
  ******************************************************************************
  */

#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "3508_motor.h"

/* �����ƶ�ģʽö�� */
typedef enum 
{
	C_M_offline, //ʧ��ģʽ
	C_M_stop,    //ֹͣģʽ
	C_M_normal,  //����ģʽ���Ե��̳���Ϊ������
	C_M_special, //����ģʽ������̨����Ϊ������
}chassis_move_mode_e;

/* ������תģʽö�� */
typedef enum 
{
	C_C_offline, //ʧ��ģʽ
	C_C_stop,    //ֹͣģʽ
	C_C_normal,  //����ģʽ���ɿ��ƣ�
	C_C_follow,  //����ģʽ����ǰ��ߵ���
	C_C_back,    //��λģʽ������ǰ��ߵ���
	C_C_cycle,   //С����ģʽ
}chassis_cycle_mode_e;

typedef struct
{
	uint8_t chassis_cycle_speed_mode; //С����ģʽ��ת��ģʽ //0����ת�� //1�ȼ���ת�� //2���Ǻ���ʽ
	uint8_t chassis_target_speed_mode; //����target�ٶȱ仯ģʽ //0���ݰ����������α仯 //1��ֵ��΢�ָ�����
}chassis_config_t;

/* ������Ϣ */
typedef struct 
{
	int16_t target_front_speed;  //Ŀ��ǰ���ٶ�
	int16_t target_right_speed;  //Ŀ�������ٶ�
	int16_t target_cycle_speed;  //Ŀ����ת�ٶ�
	uint8_t move_mode;           //�ƶ�ģʽ
	uint8_t cycle_mode;          //��תģʽ
	int16_t back_cnt;            //��λ����
	int16_t speed_cnt;           //�ٶȺ�������
}chassis_info_t;

/* ���� */
typedef struct 
{
	motor_3508_t *motor;
	chassis_config_t *config;
	chassis_info_t *info;
}chassis_t;

/* �ⲿ���� */
extern chassis_t chassis;

/* ��ʼ�� */
void chassis_init(chassis_t *chassis);
void chassis_info_init(chassis_info_t *info);
void chassis_commond_init(chassis_t *chassis);

/* �������� */
void chassis_ctrl_task(chassis_t *chassis);
void chassis_mode_update(chassis_t *chassis);
void chassis_commond_respond(chassis_t *chassis);
void chassis_work(chassis_t *chassis);
void chassis_judge_limit(chassis_t *chassis);

void motor_chassis_update(chassis_t *chassis);
void motor_speed_limit(chassis_t *chassis, int16_t *front, int16_t *right, int16_t *round);

#endif
