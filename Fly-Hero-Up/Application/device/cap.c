/**
  ******************************************************************************
  * @file           : cap.c\h
	* @author         : czf
	* @date           : 2022.4.28
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */
	
#include "cap.h"
#include "drv_can.h"

extern CAN_HandleTypeDef hcan1;

uint8_t txBuf[8];

void cap_send(void)
{
	txBuf[0] = 1;
	CAN_SendData(&hcan1, 0x0FF, txBuf);
}
