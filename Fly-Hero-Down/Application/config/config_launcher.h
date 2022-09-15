/**
  ******************************************************************************
  * @file           : config_launcher.c\h
  * @brief          : 
  * @note           : 
  ******************************************************************************
  */

#ifndef __CONFIG_LAUNCHER_H
#define __CONFIG_LAUNCHER_H

#define friction_motor_work_speed_init 7000
#define position_motor_work_speed_init 5400
#define position_motor_work_cnt_init   200
#define dial_motor_work_delay_cnt_init 100   //������ʱ����
#define auto_shoot_T_init              3000
#define dial_front_output_max_init     6000  //����������
#define dial_back_output_max_init      5000  //�˵�������
#define dial_keep_output_max_init      3000  //���־�ֹʱ������

#define Reload_Times_init 10         //������������
#define F_lock_Times_init 0          //����������������//3
#define Unload_Times_init 4          //�˵���������
#define B_lock_Times_init 2          //�˵�������������
#define Reload_angle_init 33480      //����ת���Ƕ�  //31462.5
#define Reload_angle_check_init 500  //��������Ƿ�ɹ��Ƕ�
#define Unload_angle_init 33480      //�˵�ת���Ƕ�
#define Unload_angle_check_init 500  //�˵�����Ƿ�ɹ��Ƕ�
#define F_lock_angle_init 3000       //���򿨵��˺�Ƕ�
#define F_lock_angle_check_init 100  //���򿨵��˺�����Ƿ�ɹ��Ƕ�
#define B_lock_angle_init 1000       //���򿨵��˺�Ƕ�
#define B_lock_angle_check_init 100  //���򿨵��˺�����Ƿ�ɹ��Ƕ�
#define Reload_cnt_init 50           //������ʱ
#define F_lock_cnt_init 50           //���򿨵���ʱ
#define B_lock_cnt_init 50           //�˵���ʱ
#define Unload_cnt_init 50           //���򿨵���ʱ

//#define LAUNCHER_PART_OFF     //�رշ����������
//#define DIAL_PART_OFF         //�رղ��̲���

#endif
