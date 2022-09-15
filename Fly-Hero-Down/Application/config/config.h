#ifndef __CONFIG_H
#define __CONFIG_H

#include "stm32f4xx_hal.h"

/******************************ȫ��ʹ��ö��******************************/

typedef enum DEV_WORK_STATE
{
	DEV_OFFLINE,
	DEV_ONLINE,
}Dev_Work_State;

/* ״̬��ö�� */
typedef enum LED_NUM
{
	LED_ORANGE,            //
	LED_Blue,              //ģʽ״̬��6ʧ��5��ʼ��4����3����2������1��е
	LED_RED,
	LED_GREEN,
}LED_Num;

typedef enum
{
	Light_t = 150,//250
	Interval_t = 200,//300
	Wait_t = 500,
}LED_time;

typedef enum
{
	LF,
	RF,
	LB,
	RB,
}motor_num;

#endif
