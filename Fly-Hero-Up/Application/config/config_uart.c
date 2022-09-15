/**
  ******************************************************************************
  * @file           : config_uart.c\h
  * @brief          : 
  * @note           : finish 2022-2-12 12:24:28
  ******************************************************************************
  */
	
#include "config_uart.h"
#include "remote.h"
#include "remote_potocol.h"
#include "vision.h"
#include "usmart.h"

extern usmart_t usmart;

void USART1_rxDataHandler(uint8_t *rxBuf)
{
//	usmart_update(&usmart,rxBuf);
}

void USART2_rxDataHandler(uint8_t *rxBuf)
{
	/* 遥控器信息更新 */
	rc_base_info_update(rc.base_info,rxBuf);
	rc_base_info_check(rc.base_info);
	
	/* 遥控器更新 */
	rc_interrupt_update(&rc);
}

void USART3_rxDataHandler(uint8_t *rxBuf)
{
	vision_recieve(rxBuf);
}

void USART4_rxDataHandler(uint8_t *rxBuf)
{
}

void USART5_rxDataHandler(uint8_t *rxBuf)
{
}
