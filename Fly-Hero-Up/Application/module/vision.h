/**
  ******************************************************************************
  * @file           : vision.c\h
  * @brief          : 
  * @update         : 
  ******************************************************************************
  */
  
#ifndef __VISION_H
#define __VISION_H

#include "stm32f4xx_hal.h"
#include "vision_potocol.h"
#include "timer.h"

typedef struct
{
	//�Ӿ���������
	float yaw_err;  //��ǰʱ��yaw��ǶȲ�
	float yaw_err_last;  //��һʱ��yaw��ǶȲ�
	float pitch_err;  //pitch��ǶȲ�
	float distance;  //����
	uint8_t find_Y_O_N;  //�Ƿ�ʶ�𵽱�־
	uint8_t target_car_num;  //Ŀ�공����
	
	//�Ӿ��������ݵ���Ϣ
	uint16_t find_times;  //�Ӿ�ʶ��װ�װ����
	uint16_t lost_times;  //�Ӿ�ûʶ��װ�װ����
	uint16_t real_find_times;  //��ʵʶ��װ�װ����
	uint16_t real_lost_times;  //��ʵûʶ��װ�װ����
	float target_yaw_angle;  //Ŀ��yaw��Ƕ�
	float target_yaw_angle_last;  //��һ֡Ŀ��yaw��Ƕ�
	float camera_yaw_speed;  //������yaw���ٶ�
	float target_yaw_speed;  //Ŀ��yaw���ٶ�
	float target_pitch_angle;  //Ŀ��pitch��Ƕ�
	float target_yaw_angle_begin;  //��һ֡Ŀ��yaw��Ƕ�
	float target_yaw_angle_end;  //���һ֡Ŀ��yaw��Ƕ�
	float target_yaw_angle_total;  //��һ֡�����һ֡Ŀ��yaw��ǶȲ�
}vision_base_info_t;

typedef struct
{
	uint16_t offline_cnt_max;
	uint16_t shoot_delay_cnt_max;  //�����ʱ��������
	
	uint16_t find_times_max;  //ʶ��װ�װ��������
	uint16_t lost_times_max;  //ûʶ��װ�װ��������
	uint16_t find_times_eli;  //ʶ��ǰ��֡�޳�
	uint16_t real_find_times_max;  //��ʵʶ��װ�װ��������
	uint16_t real_lost_times_max;  //��ʵûʶ��װ�װ��������
	float target_yaw_angle_dif_max;  //Ŀ��ǶȲ�ֵ���ֵ
	uint16_t offline_error_cnt_max;  //��������ʧ��������ֵ
	uint16_t target_yaw_speed_max;  //Ŀ��yaw����ٶ�����
	float yaw_shoot_bias;  //yaw�Ჹ��
	float pitch_shoot_bias;  //pitch�Ჹ��
	float predict_con;  //����Ԥ��ϵ��
	float predict_lp_con;  //����Ԥ������ͨ�˲�ϵ��
	float spin_target_yaw_err;  //������ʶ��ǶȲ�
	float predict_con_F;  //������Ԥ��ϵ��
	float predict_con_F_bias;  //������Ԥ��ϵ��ƫ��
	
	uint8_t shoot_permit;  //��������־
}vision_config_t;

typedef struct
{
	uint16_t offline_cnt;  //ʧ������
	uint16_t offline_cnt_last;  //��һ��ʧ������
	uint8_t status;  //״̬
	uint8_t interrupt;  //�жϱ�־
	
	uint16_t offline_error_cnt;  //ʧ���������ݼ���
	
	float yaw_predict;  //yaw������Ԥ����
	float yaw_predict_K;  //yaw������Ԥ��������ͨ�˲���
	uint8_t spin_Y_O_N;  //�����ݱ�־
	
	float target_yaw_angle_ave;  //Ŀ��yaw��ǶȾ�ֵ
	float target_yaw_angle_ave_last;  //��һ��Ŀ��yaw��ǶȾ�ֵ
	float target_yaw_speed_ave;  //Ŀ��yaw���ٶȾ�ֵ
	float target_pitch_angle_F;  //������ʱĿ��pitch��Ƕ�
	float target_pitch_angle_F_last;  //��һ��������ʱĿ��pitch��Ƕ�
	float target_pitch_angle_F_yaw_err;  //������ʱĿ��pitch��Ƕ�ȡֵʱ��yaw_err
	float target_distance_F;  //������ʱĿ�����
	float target_distance_F_last;  //������ʱĿ�����
	float target_distance_F_yaw_err;  //������ʱĿ�����ȡֵʱ��yaw_err
	float yaw_predict_F;  //������yaw��Ԥ����
	float yaw_predict_F_shoot;  //������yaw�����Ԥ����
	
	uint16_t shoot_delay_cnt;  //�����ʱ����
	uint8_t shoot_Y_O_N;  //�Ƿ���������־
}vision_info_t;

typedef struct
{
	float yaw_move;
	float pitch_move;
}vision_output_t;

typedef struct 
{
	vision_base_info_t *base_info;
	vision_config_t *config;
	vision_info_t *info;
	vision_output_t *output;
	vision_tx_info_t *tx_info;
	vision_rx_info_t *rx_info;
}vision_t;

extern vision_t vision;
extern timer_t      vision_task_timer;

void vision_init(vision_t *vision);
void vision_base_info_init(vision_t *vision);
void vision_info_init(vision_t *vision);
void vision_output_init(vision_t *vision);

void vision_ctrl_task(vision_t *vision);

void vision_tick_task(vision_t *vision);

bool vision_send( uint8_t  datau8_1,\
                  float    dataf_1, \
                  float    dataf_2, \
                  uint8_t  datau8_2);
bool vision_recieve(uint8_t  *rxBuf);

#endif
