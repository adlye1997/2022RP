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
#include "gimbal.h"
#include "launcher.h"
#include "vision.h"
#include "judge.h"
#include "led.h"
#include "debug.h"
#include "usmart.h"
#include "cap.h"

extern IWDG_HandleTypeDef hiwdg;

/**
  * @brief  控制任务
  * @param  
  * @retval 
  */
void Control_Task(void const * argument)
{
//	timer_init(&control_task_normal_timer);
//	timer_init(&control_task_cycle_timer);
	for(;;)
	{
//		timer_start(&control_task_normal_timer);
//		timer_cycle(&control_task_cycle_timer);
		
		/* code start */
		rc_ctrl(&rc);
		car_ctrl(&car);
//		vision_ctrl_task(&vision);
		chassis_ctrl_task(&chassis);
		gimbal_ctrl_task(&gimbal);
		launcher_ctrl_task(&launcher);
		CAN_send_all();
		judge_send_task(&judge);
		led_work();
		/* code end */
//		timer_end(&control_task_normal_timer);
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
	static int16_t cnt = 0;
//	timer_init(&realtime_task_normal_timer);
//	timer_init(&realtime_task_cycle_timer);
	for(;;)
	{
//		timer_start(&realtime_task_normal_timer);
//		timer_cycle(&realtime_task_cycle_timer);
		/* code start */
		HAL_IWDG_Refresh(&hiwdg);
		imu_update(&imu);
		gimbal_imu_update(&gimbal);
		launcher_tick_task(&launcher);
		rc_tick_task(&rc);
		vision_tick_task(&vision);
		#ifndef VISION_OFF
		if(cnt >= 3)
		{
			cnt = 0;
			vision_send(0, gimbal.info->yaw_imu_angle, gimbal.info->pitch_imu_angle, judge.base_info->car_color);
		}
		else 
		{
			cnt++;
		}
		#endif
		judge_realtime_task(&judge);
		/* code end */
//		timer_end(&realtime_task_normal_timer);
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
//		usmart_scan_task(&usmart);
    osDelay(1000);
  }
}

