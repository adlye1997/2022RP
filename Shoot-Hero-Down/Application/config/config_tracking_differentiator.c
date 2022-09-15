/**
  ******************************************************************************
  * @file           : config_tracking_differentiator.c\h
  * @brief          : 
  * @note           : 
  ******************************************************************************
  */

#include "config_tracking_differentiator.h"
#include "tracking_differentiator.h"

/*
tracking_differentiator_info_t example_track_tor_info =
{
	.r = , //�Ӽ��ٶ�
	.h0 = , //�˲�ϵ��
	.output_differential_value_max = , //���ٶ����ֵ
};
*/

tracking_differentiator_info_t chassis_front_speed_track_tor_info =
{
	.r = 0.1, //�Ӽ��ٶ�
	.h0 = 1, //�˲�ϵ��
	.output_differential_value_max = 20, //���ٶ����ֵ
};

tracking_differentiator_info_t chassis_right_speed_track_tor_info =
{
	.r = 0.1, //�Ӽ��ٶ�
	.h0 = 1, //�˲�ϵ��
	.output_differential_value_max = 20, //���ٶ����ֵ
};

tracking_differentiator_info_t pitch_speed_track_tor_info =
{
	.r = 1, //�Ӽ��ٶ�
	.h0 = 1, //�˲�ϵ��
	.output_differential_value_max = -1, //���ٶ����ֵ
};
