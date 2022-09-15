/**
  ******************************************************************************
  * @file           : drv_can.c\h
  * @brief          : 
  * @note           : finish 2022-2-12 12:08:23
  ******************************************************************************
  */

#ifndef __DEV_CAN_H
#define __DEV_CAN_H

#include "stm32f4xx_hal.h"

typedef struct 
{
  CAN_RxHeaderTypeDef Header;
  uint8_t Data[8];
}can_rx_info_t;

typedef struct 
{
	CAN_HandleTypeDef *hcan;
	uint32_t rx_id;
}drv_can_t;

void CAN_send_all(void); //CAN���巢�ͺ���

void CAN1_Init(void); //CAN1��ʼ��
void CAN2_Init(void); //CAN2��ʼ��
void CAN_Filter_ParamsInit(CAN_FilterTypeDef *sFilterConfig); //����CAN��ʶ���˲���
uint8_t CAN_SendData(CAN_HandleTypeDef *hcan, uint32_t stdId, uint8_t *dat); //CAN���ͺ���

#endif
