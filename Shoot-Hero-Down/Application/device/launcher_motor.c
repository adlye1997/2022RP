/**
  ******************************************************************************
  * @file           : launcher_motor.c\h
	* @author         : czf
	* @date           : 2022-5-7 17:12:34 finish
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */

#include "launcher_motor.h"
#include "config_can.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* yaw轴电机 */
motor_2006_base_info_t yaw_base_info;
motor_2006_info_t yaw_info;
motor_2006_config_t yaw_config = 
{
	.offline_cnt_max = 5,
}; 
extern pid_info_t yaw_pid_speed_info;
pid_t yaw_pid_speed = 
{
	.info = &yaw_pid_speed_info,
};
extern pid_info_t yaw_pid_angle_info;
pid_t yaw_pid_angle = 
{
	.info = &yaw_pid_angle_info,
};
drv_can_t yaw_can = 
{
	.hcan = &yaw_can_12,
	.rx_id = yaw_can_id,
};
motor_2006_t yaw = 
{
	.base_info = &yaw_base_info,
	.info = &yaw_info,
	.config = &yaw_config,
	.pid_speed = &yaw_pid_speed,
	.pid_angle = &yaw_pid_angle,
	.can = &yaw_can,
	.init = motor_2006_init,
	.update = motor_2006_update,
	.ctrl = motor_2006_angle_ctrl,
};

/* pitch轴电机 */
motor_3508_base_info_t pitch_base_info;
motor_3508_info_t pitch_info;
motor_3508_config_t pitch_config = 
{
	.offline_cnt_max = 5,
};
extern pid_info_t pitch_pid_speed_info;
pid_t pitch_pid_speed = 
{
	.info = &pitch_pid_speed_info,
};
extern pid_info_t pitch_pid_angle_info;
pid_t pitch_pid_angle = 
{
	.info = &pitch_pid_angle_info,
};
drv_can_t pitch_can = 
{
	.hcan = &pitch_can_12,
	.rx_id = pitch_can_id,
};
motor_3508_t pitch = 
{
	.base_info = &pitch_base_info,
	.info = &pitch_info,
	.config = &pitch_config,
	.pid_speed = &pitch_pid_speed,
	.pid_angle = &pitch_pid_angle,
	.can = &pitch_can,
	.init = motor_3508_init,
	.update = motor_3508_update,
	.ctrl = motor_3508_angle_ctrl,
};

/* 推杆电机 */
motor_3508_base_info_t push_base_info;
motor_3508_info_t push_info;
motor_3508_config_t push_config = 
{
	.offline_cnt_max = 5,
};
extern pid_info_t push_pid_speed_info;
pid_t push_pid_speed = 
{
	.info = &push_pid_speed_info,
};
extern pid_info_t push_pid_angle_info;
pid_t push_pid_angle = 
{
	.info = &push_pid_angle_info,
};
drv_can_t push_can = 
{
	.hcan = &push_can_12,
	.rx_id = push_can_id,
};
motor_3508_t push = 
{
	.base_info = &push_base_info,
	.info = &push_info,
	.config = &push_config,
	.pid_speed = &push_pid_speed,
	.pid_angle = &push_pid_angle,
	.can = &push_can,
	.init = motor_3508_init,
	.update = motor_3508_update,
	.ctrl = motor_3508_angle_ctrl,
};

/* 拨盘电机 */
motor_3508_base_info_t dial_base_info;
motor_3508_info_t dial_info;
motor_3508_config_t dial_config = 
{
	.offline_cnt_max = 5,
};
extern pid_info_t dial_pid_speed_info;
pid_t dial_pid_speed = 
{
	.info = &dial_pid_speed_info,
};
extern pid_info_t dial_pid_angle_info;
pid_t dial_pid_angle = 
{
	.info = &dial_pid_angle_info,
};
drv_can_t dial_can = 
{
	.hcan = &dial_can_12,
	.rx_id = dial_can_id,
};
motor_3508_t dial = 
{
	.base_info = &dial_base_info,
	.info = &dial_info,
	.config = &dial_config,
	.pid_speed = &dial_pid_speed,
	.pid_angle = &dial_pid_angle,
	.can = &dial_can,
	.init = motor_3508_init,
	.update = motor_3508_update,
	.ctrl = motor_3508_angle_ctrl,
};
