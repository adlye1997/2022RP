/**
  ******************************************************************************
  * @file           : math_support.c\h
  * @brief          : 
  * @note           : 2022-3-3 15:29:25
	*                   增加str_to_num函数
  ******************************************************************************
  */
	
#ifndef __MATH_SUPPORT_H
#define __MATH_SUPPORT_H

#include "stm32f4xx_hal.h"

#define abs(x) ((x)>0? (x):(-(x)))
#define sgn(x) (((x)>0)?1:((x)<0?-1:0))
#define constrain(x, min, max)	((x>max)?max:(x<min?min:x))

float lowpass(float X_last, float X_new, float K);//低通滤波

int16_t str_to_num(uint8_t *str, uint16_t len);  //字符串转数值
uint8_t num_to_str(int16_t num, uint8_t *str, uint16_t *len);  //字符串转数值
void num_to_str_2(uint16_t num, uint8_t *str, uint16_t len);
void num_to_str_3(int16_t num, uint8_t *str, uint16_t len);

#endif
