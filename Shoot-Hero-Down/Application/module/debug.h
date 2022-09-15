/**
  ******************************************************************************
  * @file           : debug.c\h
  * @brief          : 
  * @note           : 
  ******************************************************************************
  */
	
#ifndef __DEBUG_H
#define __DEBUG_H

#include "stm32f4xx_hal.h"
#include "6020_motor.h"

typedef struct
{
	uint8_t pitch_motor_debug_mode;
}pitch_debug_t;

typedef struct
{
	uint8_t yaw_motor_debug_mode;
	float imu_left_angle;
	float imu_right_angle;
}yaw_debug_t;

void pitch_motor_speed_pid_debug_task(void);
void pitch_motor_angle_pid_debug_task(int16_t up_angle, int16_t down_angle);
void pitch_imu_angle_pid_debug_task(int16_t up_angle, int16_t down_angle);

void yaw_motor_angle_pid_debug_task(int16_t left_angle, int16_t right_angle);
void yaw_imu_angle_pid_debug_task(int16_t left_angle, int16_t right_angle);


void chassis_cycle_debug(void);
void chassis_measure_speed_send(void);

void debug_vision_send(void);

#endif
