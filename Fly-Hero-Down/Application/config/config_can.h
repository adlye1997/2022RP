/**
  ******************************************************************************
  * @file           : config_can.c\h
  * @brief          : 
  * @note           : finish 2022-2-12 12:07:05
  ******************************************************************************
  */
	
#ifndef __CONFIG_CAN_H
#define __CONFIG_CAN_H

#include "stm32f4xx_hal.h"

/*
#define example_id 0x000
*/

void CAN1_Get_Data(uint32_t id, uint8_t *data); //CAN1���պ���
void CAN2_Get_Data(uint32_t id, uint8_t *data); //CAN2���պ���

#endif
