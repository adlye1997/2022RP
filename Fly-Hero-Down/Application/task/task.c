/**
  ******************************************************************************
  * @file           : task.c\h
  * @brief          : 
  * @note           : 2022-1-21 22:08:46
  ******************************************************************************
  */

#include "task.h"
#include "cmsis_os.h"
#include "config_car.h"
#include "imu.h"
#include "remote.h"
#include "car.h"
#include "chassis.h"
#include "launcher.h"
#include "vision.h"
#include "judge.h"
#include "led.h"
#include "debug.h"
#include "usmart.h"
#include "my_judge.h"

extern IWDG_HandleTypeDef hiwdg;

/**
  * @brief  控制任务
  * @param  
  * @retval 
  */
void Control_Task(void const * argument)
{
	timer_init(&control_task_normal_timer);
	timer_init(&control_task_cycle_timer);
	for(;;)
	{
		tick_task(1);
		osDelay(1);
	}
}

/**
  * @brief  实时任务
  * @param  
  * @retval 
  */
void Realtime_Task(void const * argument)
{
	for(;;)
	{
		HAL_IWDG_Refresh(&hiwdg);
		osDelay(1);
	}
}

/**
  * @brief  usmart 交互专用任务
  * @param  
  * @retval 
  */
void Interaction_Task(void const * argument)
{
  for(;;)
  {
    osDelay(1000);
  }
}

