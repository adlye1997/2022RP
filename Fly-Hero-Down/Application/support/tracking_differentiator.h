/**
  ******************************************************************************
  * @file           : tracking_differentiator.c\h
  * @brief          : 
  * @note           : 2022-2-7 22:53:42
  ******************************************************************************
  */

#ifndef __TRACKING_DIFFERENTIATOR
#define __TRACKING_DIFFERENTIATOR

typedef struct
{
	float output_value;
	float output_differential_value;       //һ��΢��
	float output_differential_value_max;   //һ��΢�����ֵ������Ϊ-1�������ƣ�
	float r;                               //�����ٶȣ�����΢�����ֵ��
	float h0;                              //�˲�ϵ����һ��ȡ1��
}tracking_differentiator_info_t;

typedef struct
{
	tracking_differentiator_info_t *info;
}tracking_differentiator_t;

void tracking_differentiator_init(tracking_differentiator_t *tor);
void tracking_differentiator_update(tracking_differentiator_t *tor, float v);
float fhan(float x1, float x2, float r, float h);
float fsg(float x, float d);
float tracking_differentiator_cal(tracking_differentiator_t *tor, float x1, float x2, float v);
	
#endif
