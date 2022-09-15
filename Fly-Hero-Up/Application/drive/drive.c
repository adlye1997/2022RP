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
  /* �����ʼ�� */
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
  /* �����ǳ�ʼ�� */
  imu_init(&imu);
	
  /* ң������ʼ�� */
  rc_init(&rc);
	
  /* ������ʼ�� */
  car_init(&car);
  
  /* ���̳�ʼ�� */
  chassis_init(&chassis);
    
  /* ��̨��ʼ�� */
  gimbal_init(&gimbal);
    
  /* ���������ʼ�� */
 launcher_init(&launcher);

	/* �Ӿ���ʼ�� */
	vision_init(&vision);
	
	/* ����ϵͳ��ʼ�� */
	judge_init(&judge);
}
