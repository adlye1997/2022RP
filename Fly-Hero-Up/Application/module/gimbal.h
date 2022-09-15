/**
  ******************************************************************************
  * @file           : gimbal.c/h
  * @brief          : 
  * @note           : 
	*                   2020.3.11 �޸�gimbal_t���ָ����
  ******************************************************************************
  */

#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "stm32F4xx_hal.h"
#include "6020_motor.h"

/* ��̨yaw��ģʽö�� */
typedef enum 
{
	G_Y_offline,  //ʧ��
	G_Y_follow,   //�������
	G_Y_gyro,     //������
	G_Y_machine,  //��е
	G_Y_auto,     //����
	G_Y_shoot,    //����
	G_Y_keep,     //����
}gimbal_yaw_mode_e;

/* ��̨pitch��ģʽö�� */
typedef enum 
{
	G_P_offline,  //ʧ��
	G_P_gyro,     //������
	G_P_machine,  //��е
	G_P_auto,     //����
	G_P_shoot,    //����
	G_P_keep,     //����
}gimbal_pitch_mode_e;

typedef struct 
{
	uint16_t yaw_angle_change_shoot;    //����ģʽyaw��Ƕȿ��Ʊ仯��
	uint16_t pitch_angle_change_shoot;  //����ģʽpitch��Ƕȿ��Ʊ仯��
	uint16_t pitch_motor_angle_middle;  //pitch������ֵ
	uint16_t yaw_motor_angle_middle;    //yaw������ֵ
}gimbal_config_t;

typedef struct
{
	float    yaw_imu_angle;             //yaw�� ��������ϵ �Ƕ�(-180��~180��)      (˳ʱ��Ϊ��)
	int16_t  yaw_motor_angle;           //yaw�� ��Ե���   �Ƕ�(-4096~4096)      (˳ʱ��Ϊ��)
	float    yaw_imu_speed;             //yaw�� ��������ϵ �ٶ�(dps)             (˳ʱ��Ϊ��)
	int16_t  yaw_motor_speed;           //yaw�� ��Ե���   �ٶ�(rpm)             (˳ʱ��Ϊ��)
	float    pitch_imu_angle;           //pitch�� ��������ϵ �Ƕ�(-90��~90��)      (����Ϊ��)
	int16_t  pitch_motor_angle;         //pitch�� ��Ե���   �Ƕ�(-2048~2048)    (����Ϊ��)
	float    pitch_imu_speed;           //pitch�� ��������ϵ �ٶ�(dps)           (����Ϊ��)
	float    pitch_motor_speed;         //pitch�� ��Ե���   �ٶ�(rpm)           (����Ϊ��)
	float    judge_yaw_angle;           //����ϵͳǹ�ܽǶ�(d)
	float    yaw_imu_angle_target;      //yaw�� ��������ϵ Ŀ��Ƕ�(-180��~180��)  (˳ʱ��Ϊ��)
	int16_t  yaw_motor_angle_target;    //yaw�� ��Ե���   Ŀ��Ƕ�(-4096~4096)  (˳ʱ��Ϊ��)
	float    pitch_imu_angle_target;    //pitch�� ��������ϵ Ŀ��Ƕ�(-90��~90��)  (����Ϊ��)
	float    pitch_motor_angle_target;  //pitch�� ��Ե���   Ŀ��Ƕ�(-2048~2048)(����Ϊ��)
	uint8_t  yaw_mode;                  //yaw��ģʽ
	uint8_t  pitch_mode;                //pitch��ģʽ
}gimbal_info_t;

typedef struct 
{
	motor_6020_t    *yaw;
	motor_6020_t    *pitch;
	gimbal_config_t *config;
	gimbal_info_t   *info;
}gimbal_t;

extern gimbal_t gimbal;

void gimbal_init(gimbal_t *gimbal);
void gimbal_info_init(gimbal_info_t *info);
void gimbal_commond_init(gimbal_t *gimbal);

void gimbal_ctrl_task(gimbal_t *gimbal);
void gimbal_mode_update(gimbal_t *gimbal);
void gimbal_commond_respond(gimbal_t *gimbal);
void gimbal_work(gimbal_t *gimbal);

void gimbal_yaw_can_update(gimbal_t *gimbal);             //��̨yaw��can����
void gimbal_pitch_can_update(gimbal_t *gimbal);           //��̨pitch��can����
void gimbal_imu_update(gimbal_t *gimbal);                 //��̨�����Ǹ���

void gimbal_yaw_angle_check(gimbal_t *gimbal);            //��̨yaw��Ƕȼ��
void gimbal_pitch_angle_check(gimbal_t *gimbal);          //��̨pitch��Ƕȼ��

void gimbal_yaw_motor_pid_speed_ctrl(gimbal_t *gimbal);   //��̨yaw�����ٶȻ�
void gimbal_yaw_imu_pid_speed_ctrl(gimbal_t *gimbal);     //��̨yaw���������ٶȻ�
void gimbal_yaw_motor_pid_angle_ctrl(gimbal_t *gimbal);   //��̨yaw�����ǶȻ�
void gimbal_yaw_imu_pid_angle_ctrl(gimbal_t *gimbal);     //��̨yaw�������ǽǶȻ�
void gimbal_pitch_motor_pid_speed_ctrl(gimbal_t *gimbal); //��̨pitch�����ٶȻ�
void gimbal_pitch_imu_pid_speed_ctrl(gimbal_t *gimbal);   //��̨pitch���������ٶȻ�
void gimbal_pitch_motor_pid_angle_ctrl(gimbal_t *gimbal); //��̨pitch�����ǶȻ�
void gimbal_pitch_imu_pid_angle_ctrl(gimbal_t *gimbal);   //��̨pitch�������ǽǶȻ�

#endif
