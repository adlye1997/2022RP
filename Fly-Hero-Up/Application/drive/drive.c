/**
  ******************************************************************************
  * @file           : drive.c/h
  * @brief          : 
  * @note           : finish 2022-2-12 12:17:38
  ******************************************************************************
  */

#include "drive.h"
#include "bmi2_defs.h"
#include "bmi.h"
#include "drv_can.h"
#include "drv_uart.h"
#include "imu.h"
#include "remote.h"
#include "car.h"
#include "chassis.h"
#include "gimbal.h"
#include "launcher.h"
#include "vision.h"
#include "judge.h"
#include "tracking_differentiator.h"
#include "pid_feedback_system.h"
#include "debug.h"
#include "led.h"
#include "cap.h"

void drive_init(void)
{
  /* 外设初始化 */
  CAN1_Init();
  CAN2_Init();
  USART1_Init();
  USART2_Init();
  USART3_Init();
  USART4_Init();
  USART5_Init();
	
	cap_send();
}

void module_init(void)
{
  /* 陀螺仪初始化 */
  imu_init(&imu);
	
  /* 遥控器初始化 */
  rc_init(&rc);
	
  /* 整车初始化 */
  car_init(&car);
  
  /* 底盘初始化 */
  chassis_init(&chassis);
    
  /* 云台初始化 */
  gimbal_init(&gimbal);
    
  /* 发射机构初始化 */
 launcher_init(&launcher);

	/* 视觉初始化 */
	vision_init(&vision);
	
	/* 裁判系统初始化 */
	judge_init(&judge);
}
