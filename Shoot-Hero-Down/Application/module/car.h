/**
  ******************************************************************************
  * @file           : car.c/h
  * @brief          : 
  * @note           : 
  ******************************************************************************
  */

#ifndef __CAR_H
#define __CAR_H

#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "remote.h"

/* ���ж�ģʽö�� */
typedef enum 
{
  offline_CAR,        //����ģʽ
  init_CAR,           //��ʼ��ģʽ
  machine_CAR,        //��еģʽ
  high_shoot_CAR,     //����ģʽ
}car_move_mode_e;

/* ������ģʽö�� */
typedef enum 
{
  RC_CAR,             //ң��������
  KEY_CAR,            //���̿���
}car_ctrl_mode_e;

/* ������ö�� */
typedef struct 
{
  uint8_t reserve;
}car_config_t;


typedef struct 
{
	car_config_t *config;  //��������
	
  uint8_t move_mode_commond;  //�ƶ�ģʽ����
  uint8_t move_mode_status;   //�ƶ�ģʽ״̬
  uint8_t ctrl_mode;          //����ģʽ
}car_t;

/* �ⲿ���� */
extern bool car_mode_change;
extern bool chassis_cycle_off;
extern bool chassis_cycle_on;
extern bool chassis_cycle_change;
extern bool gimbal_to_check;           //��̨�л�checkģʽ
extern bool gimbal_out_check;          //��̨�˳�checkģʽ
extern bool gimbal_l_90;
extern bool launcher_l;
extern bool gimbal_r_90;
extern bool launcher_r;
extern bool gimbal_r_180;
extern bool launcher_u;
extern bool launcher_d;
extern bool launcher_u_s;
extern bool launcher_d_s;
extern bool launcher_l_s;
extern bool launcher_r_s;
extern bool Launcher_Shoot;
extern bool Launcher_Relax;
extern bool Launcher_Ready;
extern bool Launcher_Return;
extern bool dial_reload;
extern bool dial_unload;
extern bool auto_on;
extern bool auto_off;
extern bool judge_close;
extern bool launcher_auto;
extern bool Launcher_Init;

extern car_t car;

/* ��ʼ�� */
void car_init(car_t *car);
void car_commond_init(car_t *car);

/* �������� */
void car_ctrl(car_t *car);
void car_init_judge(void);
void car_mode_commond_update(car_t *car);
void car_mode_status_update(car_t *car);

/* ң������λɨ�� */
void RC_status_scan(car_t *car);
/* ң������λ״̬���� */
void RC_s1_status_check(car_t *car);
void RC_s2_status_check(car_t *car);
void RC_thumbwheel_status_check(car_t *car);

/* ���̰���ɨ�� */
void KEY_status_scan(car_t *car);
/* ���̰���״̬���� */
void KEY_mouse_l_status_check(car_t *car);
void KEY_mouse_r_status_check(car_t *car);
void KEY_Q_status_check(car_t *car);
void KEY_W_status_check(car_t *car);
void KEY_E_status_check(car_t *car);
void KEY_R_status_check(car_t *car);
void KEY_A_status_check(car_t *car);
void KEY_S_status_check(car_t *car);
void KEY_D_status_check(car_t *car);
void KEY_F_status_check(car_t *car);
void KEY_G_status_check(car_t *car);
void KEY_Z_status_check(car_t *car);
void KEY_X_status_check(car_t *car);
void KEY_C_status_check(car_t *car);
void KEY_V_status_check(car_t *car);
void KEY_B_status_check(car_t *car);
void KEY_Shift_status_check(car_t *car);
void KEY_Ctrl_status_check(car_t *car);

/* ����ģʽ */
void debug_special_scan(car_t *car);

#endif
