/**
  ******************************************************************************
  * @file           : remote.c/h
  * @brief          : 
  * @note           : ����ʱ�䣺2022��1��11��17:24:36
	*                   ����ٶ��˲�˵����
	*                   �ڸ����ж϶�����ٶȳ�ʼ���ݽ��о�ֵ�˲�
	*                   �ڿ��������ж�����ٶȽ��е�ͨ�˲�
  ******************************************************************************
  */

#ifndef __REMOTE_H
#define __REMOTE_H

#include "stm32f4xx_hal.h"
#include "config_remote.h"

/* ����״̬ö�� */
typedef enum
{
  relax_K,        //����
  down_K,         //����
  up_K,           //̧��
  short_press_K,  //�̰�
  long_press_K,   //����
}key_board_status_e;

/* ң����������ť״̬ö�� */
typedef enum 
{
  keep_R,         //����
  up_R,           //���ϲ�
  mid_R,          //���в�
  down_R,         //���²�
}remote_status_e;

/* ������Ϣ */
typedef struct
{
  uint8_t value;    //ֵ
  uint8_t status;   //״̬
  int16_t cnt;      //��ǰ����
  int16_t cnt_max;  //��������
}key_board_info_t;

/* ������Ϣ */
typedef struct
{
  uint8_t value_last;  //��һ��ֵ
  uint8_t value;       //��ֵ
  uint8_t status;      //״̬
}remote_switch_info_t;

/* ��ť��Ϣ */
typedef struct
{
  int16_t value_last;  //��һ��ֵ
  int16_t value;       //��ֵ
  uint8_t status;      //״̬
}remote_wheel_info_t;

/* ң��ԭʼ��Ϣ */
typedef struct 
{
  /* ң���� */
  int16_t                 ch0;                  //�ҵ�����
  int16_t                 ch1;                  //�ҵ�ǰ��
  int16_t                 ch2;                  //�������
  int16_t                 ch3;                  //���ǰ��
  remote_switch_info_t    s1;                   //�󲦸�
  remote_switch_info_t    s2;                   //�Ҳ���
  remote_wheel_info_t     thumbwheel;           //����Ť
  /* ���� */
  int16_t                 mouse_vx;             //���x���ٶ�
  int16_t                 mouse_vy;             //���y���ٶ�
  int16_t                 mouse_vz;             //���z���ٶ�
  key_board_info_t        mouse_btn_l;          //������
  key_board_info_t        mouse_btn_r;          //����Ҽ�
  key_board_info_t        Q;                    //����Q
  key_board_info_t        W;                    //����W
  key_board_info_t        E;                    //����E
  key_board_info_t        R;                    //����R
  key_board_info_t        A;                    //����A
  key_board_info_t        S;                    //����S
  key_board_info_t        D;                    //����D
  key_board_info_t        F;                    //����F
  key_board_info_t        G;                    //����G
  key_board_info_t        Z;                    //����Z
  key_board_info_t        X;                    //����X
  key_board_info_t        C;                    //����C
  key_board_info_t        V;                    //����V
  key_board_info_t        B;                    //����B
  key_board_info_t        Shift;                //����Shift
  key_board_info_t        Ctrl;                 //����Ctrl
}rc_base_info_t;

/* ң����Ϣ */
typedef struct 
{
  int16_t             offline_cnt;  //ʧ������
	uint8_t             status;       //״̬
	float					  		mouse_x;      //���x���ٶ�
	float  							mouse_y;      //���y���ٶ�
	float               mouse_x_K;    //���x���˲����ٶ�
	float  							mouse_y_K;    //���y���˲����ٶ�
}rc_info_t;

/* ң�� */
typedef struct
{
  rc_base_info_t     *base_info;
  rc_info_t          *info;
}rc_t;

/* �ⲿ���� */
extern rc_t rc;

/* ��ʼ�� */
void rc_init(rc_t *rc);
void rc_base_info_init(rc_base_info_t *info);
void key_board_cnt_max_set(rc_base_info_t *info);
void rc_info_init(rc_info_t *info);
void rc_interrupt_init(rc_base_info_t *info);

/* �ж� */
void rc_interrupt_update(rc_t *rc);

/* �δ����� */
void rc_tick_task(rc_t *rc);

/* �������� */
void rc_ctrl(rc_t *rc);

/* ״̬���� */
void key_board_status_update(key_board_info_t *key);
void all_key_board_status_update(rc_base_info_t *info);

/* �ж�״̬���� */
void rc_switch_status_interrupt_update(rc_base_info_t *info);
void rc_wheel_status_interrupt_update(rc_base_info_t *info);
void key_board_status_interrupt_update(key_board_info_t *key);
void all_key_board_status_interrupt_update(rc_base_info_t *info);
void remote_soft_reset_check(rc_t *rc);

#endif
