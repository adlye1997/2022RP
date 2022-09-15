/**
  ******************************************************************************
  * @file           : vector.c\h
	* @author         : czf
	* @date           : 2022-5-18 20:47:36
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */
	
#ifndef __VECTOR_H
#define __VECTOR_H

#include "stm32f4xx_hal.h"

typedef struct 
{
	float data[50];
	uint16_t index;
	uint16_t size;
}vector_t;

void vector_init(vector_t *vector, int16_t size);
void vector_update(vector_t *vector, float num);
float vector_get(vector_t *vector);

#endif
