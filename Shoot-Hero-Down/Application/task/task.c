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
#include "config.h"
#include "my_judge.h"

void motor_tick_task(void);

int64_t see;

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
		timer_start(&control_task_normal_timer);
		timer_cycle(&control_task_cycle_timer);
		/* code start */
		rc_ctrl(&rc);
		car_ctrl(&car);

		chassis_ctrl_task(&chassis);
		
		gimbal_ctrl_task(&gimbal);
		
		launcher_ctrl_task(&launcher);
		
#ifndef ALL_MOTOR_CLOSE
		CAN_send_all();
#endif
		
		/* code end */
		timer_end(&control_task_normal_timer);
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
	timer_init(&realtime_task_normal_timer);
	timer_init(&realtime_task_cycle_timer);
	for(;;)
	{
		timer_start(&realtime_task_normal_timer);
		timer_cycle(&realtime_task_cycle_timer);
		/* code start */
		rc_tick_task(&rc);
		judge_realtime_task(&judge);
		motor_tick_task();
    tick_task(1);
		/* code end */
		timer_end(&realtime_task_normal_timer);
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

void motor_tick_task(void)
{
	chassis.motor_LB->info->offline_cnt++;
	chassis.motor_LF->info->offline_cnt++;
	chassis.motor_RB->info->offline_cnt++;
	chassis.motor_RF->info->offline_cnt++;
	
	gimbal.yaw->info->offline_cnt++;
	gimbal.pitch->info->offline_cnt++;
	gimbal.watch->info->offline_cnt++;
	
	launcher.pitch->info->offline_cnt++;
	launcher.yaw->info->offline_cnt++;
	launcher.push->info->offline_cnt++;
	launcher.dial->info->offline_cnt++;
	
	if(chassis.motor_LB->info->offline_cnt > chassis.motor_LB->config->offline_cnt_max)
	{
		chassis.motor_LB->info->offline_cnt = chassis.motor_LB->config->offline_cnt_max + 1;
		chassis.motor_LB->info->status = DEV_OFFLINE;
	}
	
	if(chassis.motor_LF->info->offline_cnt > chassis.motor_LF->config->offline_cnt_max)
	{
		chassis.motor_LF->info->offline_cnt = chassis.motor_LF->config->offline_cnt_max + 1;
		chassis.motor_LF->info->status = DEV_OFFLINE;
	}
	
	if(chassis.motor_RB->info->offline_cnt > chassis.motor_RB->config->offline_cnt_max)
	{
		chassis.motor_RB->info->offline_cnt = chassis.motor_RB->config->offline_cnt_max + 1;
		chassis.motor_RB->info->status = DEV_OFFLINE;
	}
	
	if(chassis.motor_RF->info->offline_cnt > chassis.motor_RF->config->offline_cnt_max)
	{
		chassis.motor_RF->info->offline_cnt = chassis.motor_RF->config->offline_cnt_max + 1;
		chassis.motor_RF->info->status = DEV_OFFLINE;
	}
	
	if(gimbal.yaw->info->offline_cnt > gimbal.yaw->config->offline_cnt_max)
	{
		gimbal.yaw->info->offline_cnt = gimbal.yaw->config->offline_cnt_max + 1;
		gimbal.yaw->info->status = DEV_OFFLINE;
	}
	
	if(gimbal.pitch->info->offline_cnt > gimbal.pitch->config->offline_cnt_max)
	{
		gimbal.pitch->info->offline_cnt = gimbal.pitch->config->offline_cnt_max + 1;
		gimbal.pitch->info->status = DEV_OFFLINE;
	}
	
	if(gimbal.watch->info->offline_cnt > gimbal.watch->config->offline_cnt_max)
	{
		gimbal.watch->info->offline_cnt = gimbal.watch->config->offline_cnt_max + 1;
		gimbal.watch->info->status = DEV_OFFLINE;
	}
	
	if(launcher.pitch->info->offline_cnt > launcher.pitch->config->offline_cnt_max)
	{
		launcher.pitch->info->offline_cnt = launcher.pitch->config->offline_cnt_max + 1;
		launcher.pitch->info->status = DEV_OFFLINE;
	}
	
	if(launcher.yaw->info->offline_cnt > launcher.yaw->config->offline_cnt_max)
	{
		launcher.yaw->info->offline_cnt = launcher.yaw->config->offline_cnt_max + 1;
		launcher.yaw->info->status = DEV_OFFLINE;
	}
	
	if(launcher.push->info->offline_cnt > launcher.push->config->offline_cnt_max)
	{
		launcher.push->info->offline_cnt = launcher.push->config->offline_cnt_max + 1;
		launcher.push->info->status = DEV_OFFLINE;
	}
	
	if(launcher.dial->info->offline_cnt > launcher.dial->config->offline_cnt_max)
	{
		launcher.dial->info->offline_cnt = launcher.dial->config->offline_cnt_max + 1;
		launcher.dial->info->status = DEV_OFFLINE;
	}
}

