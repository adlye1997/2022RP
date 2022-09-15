/**
  ******************************************************************************
  * @file           : config_chassis.c\h
  * @brief          : 
  * @note           : 
  ******************************************************************************
  */

#ifndef __CONFIG_CHASSIS_H
#define __CONFIG_CHASSIS_H

#define chassis_cycle_T 1000    //����С��������
#define chassis_back_time 2000  //������ģʽ->��еģʽ�ĵ�������λʱ��

#define chassis_cycle_speed_mode_init 0 //С����ģʽ��ת��ģʽ //0����ת�� //1�ȼ���ת�� //2���Ǻ���ʽ
#define chassis_target_speed_mode_init 0 //����target�ٶȱ仯ģʽ //0���ݰ����������α仯 //1��ֵ��΢�ָ�����

#define chassis_cycle_mode_0_speed_init 3350 //����С����ģʽ0�ٶ�
#define chassis_cycle_mode_1_speed_init 2050 //����С����ģʽ1�ٶ�
#define chassis_cycle_mode_2_speed_init 2350 //����С����ģʽ2�ٶ�

#define chassis_output_max_init 50000// 40000 //����������
#define chassis_speed_max_init 8000 //��������ٶ�

#define straight_rectify_coefficient_init 0.1  //ֱ��У��ϵ����У��������뵱ǰ����ı�ֵ������Χ0~1��

#endif
