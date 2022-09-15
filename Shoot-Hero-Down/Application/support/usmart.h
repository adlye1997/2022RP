/**
  ******************************************************************************
  * @file           : usmart.c\h
  * @brief          : 
  * @note           : 2022-3-3 14:21:12
  ******************************************************************************
  */
	
#ifndef __USMART_H
#define __USMART_H

#include "stm32f4xx_hal.h"

typedef enum
{
	usmart_ok,
	usmart_name_err,
	usmart_para_err,
}usmart_error_e;

typedef struct
{
	uint8_t interrupt;
}usmart_info_t;

typedef struct
{
	void* func;  //����ָ��
	const uint8_t* name;  //������(���Ҵ�)
}usmart_cmd_list_t;

typedef struct
{
	usmart_info_t *info;
	usmart_cmd_list_t *list;  //�����б�
	uint16_t cmd_list_sum;  //�����б����
	uint8_t *fname;  //������
	float *fpara;  //��������
	uint16_t fpara_sum;  //������������
}usmart_t;

extern usmart_t usmart;

void usmart_init(usmart_t *usmart);
void usmart_info_init(usmart_info_t *info);
void usmart_update(usmart_t *usmart, uint8_t *rxbuf);

void usmart_scan_task(usmart_t *usmart);

uint8_t usmart_cmd_res(usmart_t *usmart);
uint8_t usmart_get_fname(uint8_t *str, uint8_t *name);
uint8_t usmart_get_fpara(uint8_t *str, float *para, uint16_t *sum);

float usmart_get_num(uint8_t *str);

void usmart_send_data(uint8_t *str, uint16_t len);

#endif

