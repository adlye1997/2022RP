/**
  ******************************************************************************
  * @file           : vector.c\h
	* @author         : czf
	* @date           : 2022-5-18 20:47:36
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */

#include "vector.h"
#include "string.h"

void vector_init(vector_t *vector, int16_t size)
{
	if(size > 50)
	{
		size = 50;
	}
	vector->size = size;
	vector->index = 0;
	memset(vector->data, 0, size);
}

void vector_update(vector_t *vector, float num)
{
	vector->data[vector->index] = num;
	vector->index ++;
	vector->index %= vector->size;
}

float vector_get(vector_t *vector)
{
	return vector->data[vector->index];
}
