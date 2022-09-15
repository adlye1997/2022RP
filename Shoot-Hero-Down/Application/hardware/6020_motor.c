/**
  ******************************************************************************
  * @file           : 6020_motor.c\h
	* @author         : czf
	* @date           : 2022-5-7 22:21:08
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */

#include "6020_motor.h"
#include "math_support.h"
#include "string.h"
#include "config.h"

/*
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
motor_6020_base_info_t example_base_info;
motor_6020_info_t example_info;
motor_6020_config_t example_config = 
{
	.offline_cnt_max = 5,
};
extern pid_info_t example_pid_speed_info;
pid_t example_pid_speed = 
{
	.info = &example_pid_speed_info,
};
extern pid_info_t example_pid_angle_info;
pid_t example_pid_angle = 
{
	.info = &example_pid_angle_info,
};
drv_can_t example_can = 
{
	.hcan = &example_can_12,
	.rx_id = example_can_id,
};
motor_6020_t example = 
{
	.base_info = &example_base_info,
	.info = &example_info,
	.config = &example_config,
	.pid_speed = &example_pid_speed,
	.pid_angle = &example_pid_angle,
	.can = &example_can,
	.init = motor_6020_init,
	.update = motor_6020_update,
};
*/

extern uint8_t can1_tx_buf[16];
extern uint8_t can2_tx_buf[16];
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void motor_6020_init(motor_6020_t *motor)
{
	memset(motor->base_info, 0, sizeof(motor_6020_base_info_t));
	motor_6020_info_init(motor);
	if(motor->pid_speed != NULL)
	{
		pid_init(motor->pid_speed);
	}
	if(motor->pid_angle != NULL)
	{
		pid_init(motor->pid_angle);
	}
}

void motor_6020_info_init(motor_6020_t *motor)
{
	motor_6020_info_t *info = motor->info;
	
	info->offline_cnt = motor->config->offline_cnt_max + 1;
	info->status = DEV_OFFLINE;
	info->target_angle_sum = 0;
}

void motor_6020_update(motor_6020_t *motor, uint8_t *rxBuf)
{
	motor_6020_base_info_t *base_info = motor->base_info;
	uint16_t angle_last = base_info->angle;
	
	base_info->angle = rxBuf[0];
	base_info->angle <<= 8;
	base_info->angle |= rxBuf[1];
	base_info->speed = rxBuf[2];
	base_info->speed <<= 8;
	base_info->speed |= rxBuf[3];
	base_info->current = rxBuf[4];
	base_info->current <<= 8;
	base_info->current |= rxBuf[5];
	base_info->temperature = rxBuf[6];
	
	/* angle_add */
	base_info->angle_add = base_info->angle - angle_last;
	if(abs(base_info->angle_add) > 4096)
	{
		base_info->angle_add -= 8192 * sgn(base_info->angle_add);
	}
	
	/* angle_sum and target_angle_sum */
	base_info->angle_sum += base_info->angle_add;
	if(abs(base_info->angle_sum) > 0x0FFF)
	{
		base_info->angle_sum -= 0x0FFF * sgn(base_info->angle_sum);
		motor->info->target_angle_sum -= 0x0FFF * sgn(base_info->angle_sum);
	}
	
	/* offline_flag */
	motor->info->offline_cnt = 0;
	motor->info->status = DEV_ONLINE;
}
