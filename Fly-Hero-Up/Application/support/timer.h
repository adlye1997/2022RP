/**
  ******************************************************************************
  * @file           : timer.c/h
  * @brief          : 
  * @note           : 2022-1-17 22:23:13
  ******************************************************************************
  */

#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f4xx_hal.h"

typedef struct 
{
	uint32_t start_time_us;  //��ʼʱ��(us)
	uint32_t start_time_ms;  //��ʼʱ��(ms)
	uint32_t end_time_us;    //����ʱ��(us)
	uint32_t end_time_ms;    //����ʱ��(ms)
	uint32_t duration_us;    //����ʱ��(us)
	uint32_t duration_ms;    //����ʱ��(ms)
}timer_info_t;

typedef struct 
{
	timer_info_t *info;
}timer_t;

void timer_init(timer_t *timer);
void timer_start(timer_t *timer);
void timer_end(timer_t *timer);
void timer_cycle(timer_t *timer);

#endif
