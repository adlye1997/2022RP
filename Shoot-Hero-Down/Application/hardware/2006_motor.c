/**
  ******************************************************************************
  * @file           : 2006_motor.c\h
	* @author         : czf
	* @date           : 2022-5-7 16:52:39 finish
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */
	
#include "2006_motor.h"
#include "math_support.h"
#include "config.h"
#include "string.h"

/*
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
motor_2006_base_info_t example_base_info;
motor_2006_info_t example_info;
motor_2006_config_t example_config = 
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
	.hcan = &hcan1 or hcan2,
	.rx_id = example_can_id,
};
motor_2006_t example = 
{
	.base_info = &example_base_info,
	.info = &example_info,
	.config = &example_config,
	.pid_speed = &example_pid_speed,
	.pid_angle = &example_pid_angle,
	.can = &example_can,
	.init = motor_2006_init,
	.update = motor_2006_update,
	.ctrl = motor_2006_angle_ctrl or motor_2006_speed_ctrl,
};
*/

extern uint8_t can1_tx_buf[16];
extern uint8_t can2_tx_buf[16];
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**
  * @brief  
  * @param  
  * @retval 
  */
void motor_2006_init(motor_2006_t *motor)
{
	memset(motor->base_info,0,sizeof(motor_2006_base_info_t));
	motor_2006_info_init(motor);
	if(motor->pid_speed != NULL)
	{
		pid_init(motor->pid_speed);
	}
	if(motor->pid_angle != NULL)
	{
		pid_init(motor->pid_angle);
	}
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void motor_2006_info_init(motor_2006_t *motor)
{
	motor_2006_info_t *info = motor->info;
	
	info->offline_cnt = motor->config->offline_cnt_max + 1;
	info->status = DEV_OFFLINE;
	info->target_angle_sum = 0;
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void motor_2006_update(motor_2006_t *motor, uint8_t *rxBuf)
{
	motor_2006_base_info_t *base_info = motor->base_info;
	int16_t last_angle = base_info->angle;
	
	/* 电调返回信息 */
	base_info->angle = rxBuf[0];
	base_info->angle <<= 8;
	base_info->angle |= rxBuf[1];
	base_info->speed = rxBuf[2];
	base_info->speed <<= 8;
	base_info->speed |= rxBuf[3];
	base_info->torque = rxBuf[4];
	base_info->torque <<= 8;
	base_info->torque |= rxBuf[5];
	
	/* angle_add */
	base_info->angle_add = base_info->angle - last_angle;
	if(abs(base_info->angle_add) > 4096)
	{
		base_info->angle_add -= 8192 * sgn(base_info->angle_add);
	}
	
	/* angle_sum and target_angle_sum */
	base_info->angle_sum += base_info->angle_add;
	if(abs(base_info->angle_sum) > 0x0FFFFFFF)
	{
		base_info->angle_sum -= 0x0FFFFFFF * sgn(base_info->angle_sum);
		motor->info->target_angle_sum -= 0x0FFFFFFF * sgn(base_info->angle_sum);
	}

	/* flag */
	motor->info->offline_cnt = 0;
	motor->info->status = DEV_ONLINE;
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void motor_2006_speed_ctrl(motor_2006_t *motor)
{
	int16_t output;
	motor->pid_speed->info->measure = motor->base_info->speed;
	single_pid_cal(motor->pid_speed->info);
	output = motor->pid_speed->info->out;
	if(motor->can->hcan == &hcan1){
		can1_tx_buf[(motor->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
		can1_tx_buf[(motor->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
	}
	else if(motor->can->hcan == &hcan2){
		can2_tx_buf[(motor->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
		can2_tx_buf[(motor->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
	}
}

/**
  * @brief  
  * @param  
  * @retval 
  */
void motor_2006_angle_ctrl(motor_2006_t *motor)
{
	motor->pid_angle->info->target = motor->info->target_angle_sum;
	motor->pid_angle->info->measure = motor->base_info->angle_sum;
	single_pid_cal(motor->pid_angle->info);
	motor->pid_speed->info->target = motor->pid_angle->info->out;
	motor_2006_speed_ctrl(motor);
}
