/**
  ******************************************************************************
  * @file           : vision.c\h
  * @brief          : 
  * @update         : finish 2022-2-13 19:34:19
  ******************************************************************************
  */
  
#ifndef __VISION_H
#define __VISION_H

#include "stm32f4xx_hal.h"
#include "vision_potocol.h"
#include "timer.h"

typedef struct 
{
	float yaw_err;
	float pitch_err;
	vision_tx_info_t *tx_info;
	vision_rx_info_t *rx_info;
	uint16_t offline_cnt;
	uint8_t status;
}vision_t;

extern vision_t vision;
extern timer_t      vision_task_timer;

bool vision_send( uint8_t  datau8_1,\
                  uint8_t  datau8_2,\
                  uint8_t  datau8_3,\
                  float    dataf_1, \
                  float    dataf_2, \
									float    dataf_3, \
                  uint8_t  datau8_4);
bool vision_recieve(uint8_t  *rxBuf);
void vision_init(vision_t *vision);
									
void vision_tick_task(vision_t *vision);

#endif
