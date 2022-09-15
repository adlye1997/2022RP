/**
  ******************************************************************************
  * @file           : vision_potocol.c\h
  * @brief          : 
  * @update         : finish 2022-2-13 19:16:56
  ******************************************************************************
  */

#ifndef __VISION_POTOCOL_H
#define __VISION_POTOCOL_H

#include "stm32f4xx_hal.h"
#include "stdbool.h"

typedef __packed struct 
{
	uint8_t  SOF;
	uint8_t  datau8_1;
	uint8_t  CRC8;
	float    dataf_1;  //pitch_angle
	float    dataf_2;  //yaw_angle
 	uint8_t  datau8_2;  //±¾·½ÑÕÉ«
	uint16_t CRC16;
}vision_tx_info_t;

typedef __packed struct 
{
	uint8_t  SOF;
	uint8_t  datau8_1;
	uint8_t  CRC8;
	float    dataf_1;  //pitch
	float    dataf_2;  //yaw
	float    dataf_3;  //distance
	uint8_t  datau8_2;  //is_find_target
	uint8_t  datau8_3;  //car_number
	uint16_t CRC16;
}vision_rx_info_t;

extern vision_tx_info_t vision_tx_info;
extern vision_rx_info_t vision_rx_info;

bool vision_send_data(void);
bool vision_recieve_data(uint8_t *rxBuf);

#endif
