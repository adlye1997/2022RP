/**
  ******************************************************************************
  * @file           : chassis_motor.c\h
	* @author         : czf
	* @date           : 2022-5-7 21:00:07 finish
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */

#include "chassis_motor.h"
#include "config_can.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* ×óÇ°ÂÖ */
motor_3508_base_info_t chassis_LF_base_info;
motor_3508_info_t chassis_LF_info;
motor_3508_config_t chassis_LF_config = 
{
	.offline_cnt_max = 5,
};
extern pid_info_t chassis_LF_pid_speed_info;
pid_t chassis_LF_pid_speed = 
{
	.info = &chassis_LF_pid_speed_info,
};
drv_can_t chassis_LF_can = 
{
	.hcan = &chassis_LF_can_12,
	.rx_id = chassis_LF_can_id,
};
motor_3508_t chassis_LF = 
{
	.base_info = &chassis_LF_base_info,
	.info = &chassis_LF_info,
	.config = &chassis_LF_config,
	.pid_speed = &chassis_LF_pid_speed,
	.can = &chassis_LF_can,
	.init = motor_3508_init,
	.update = motor_3508_update,
	.ctrl = motor_3508_speed_ctrl,
};

/* ÓÒÇ°ÂÖ */
motor_3508_base_info_t chassis_RF_base_info;
motor_3508_info_t chassis_RF_info;
motor_3508_config_t chassis_RF_config = 
{
	.offline_cnt_max = 5,
};
extern pid_info_t chassis_RF_pid_speed_info;
pid_t chassis_RF_pid_speed = 
{
	.info = &chassis_RF_pid_speed_info,
};
drv_can_t chassis_RF_can = 
{
	.hcan = &chassis_RF_can_12,
	.rx_id = chassis_RF_can_id,
};
motor_3508_t chassis_RF = 
{
	.base_info = &chassis_RF_base_info,
	.info = &chassis_RF_info,
	.config = &chassis_RF_config,
	.pid_speed = &chassis_RF_pid_speed,
	.can = &chassis_RF_can,
	.init = motor_3508_init,
	.update = motor_3508_update,
	.ctrl = motor_3508_speed_ctrl,
};

/* ×óºóÂÖ */
motor_3508_base_info_t chassis_LB_base_info;
motor_3508_info_t chassis_LB_info;
motor_3508_config_t chassis_LB_config = 
{
	.offline_cnt_max = 5,
};
extern pid_info_t chassis_LB_pid_speed_info;
pid_t chassis_LB_pid_speed = 
{
	.info = &chassis_LB_pid_speed_info,
};
drv_can_t chassis_LB_can = 
{
	.hcan = &chassis_LB_can_12,
	.rx_id = chassis_LB_can_id,
};
motor_3508_t chassis_LB = 
{
	.base_info = &chassis_LB_base_info,
	.info = &chassis_LB_info,
	.config = &chassis_LB_config,
	.pid_speed = &chassis_LB_pid_speed,
	.can = &chassis_LB_can,
	.init = motor_3508_init,
	.update = motor_3508_update,
	.ctrl = motor_3508_speed_ctrl,
};

/* ÓÒºóÂÖ */
motor_3508_base_info_t chassis_RB_base_info;
motor_3508_info_t chassis_RB_info;
motor_3508_config_t chassis_RB_config = 
{
	.offline_cnt_max = 5,
};
extern pid_info_t chassis_RB_pid_speed_info;
pid_t chassis_RB_pid_speed = 
{
	.info = &chassis_RB_pid_speed_info,
};
drv_can_t chassis_RB_can = 
{
	.hcan = &chassis_RB_can_12,
	.rx_id = chassis_RB_can_id,
};
motor_3508_t chassis_RB = 
{
	.base_info = &chassis_RB_base_info,
	.info = &chassis_RB_info,
	.config = &chassis_RB_config,
	.pid_speed = &chassis_RB_pid_speed,
	.can = &chassis_RB_can,
	.init = motor_3508_init,
	.update = motor_3508_update,
	.ctrl = motor_3508_speed_ctrl,
};
