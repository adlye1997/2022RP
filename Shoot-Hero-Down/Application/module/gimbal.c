/**
  ******************************************************************************
  * @file           : gimbal.c\h
	* @author         : czf
	* @date           : 
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */

#include "gimbal.h"
#include "string.h"
#include "gimbal_motor.h"
#include "car.h"
#include "main.h"
#include "rp_math.h"
#include "config_gimbal.h"
#include "math_support.h"
#include "imu.h"
#include "tracking_differentiator.h"
#include "ave_filter.h"
#include "vision.h"
#include "config.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern uint8_t can1_tx_buf[16];
extern uint8_t can2_tx_buf[16];

bool gimbal_init_Y_O_N = false;
bool gimbal_machine_debug_on = false;
bool gimbal_machine_debug_off = false;

gimbal_work_info_t gimbal_work_info;
s_pitch_work_info_t s_pitch_work_info;
s_yaw_work_info_t s_yaw_work_info;
watch_work_info_t watch_work_info;
gimbal_config_t gimbal_config = 
{
	.watch_init_out_max = 2000,
	.watch_normal_out_max = 9000,
	.watch_init_speed = 2000,
	.watch_up_angle = 145000,
	.pitch_init_speed = 500,
	.pitch_up_angle = 26000,
	.yaw_check_angle = -3122,
	.pitch_check_angle = -7255,
	.yaw_middle_angle = 4724,
	.lock_cnt_max = 100,
	.pitch_angle_permit_max = 23000,
	.pitch_angle_permit_min = -8400,
};
gimbal_info_t gimbal_info;
gimbal_t gimbal = 
{
	.pitch = &s_pitch,
	.yaw = &s_yaw,
	.watch = &watch,
	
	.work = &gimbal_work_info,
	.pitch_work = &s_pitch_work_info,
	.yaw_work = &s_yaw_work_info,
	.watch_work = &watch_work_info,
	
	.config = &gimbal_config,
	.info = &gimbal_info,
};

/**
  * @brief  云台初始化
  * @param  
  * @retval 
  */
void gimbal_init(gimbal_t *gimbal)
{
	/* 电机初始化 */
	gimbal->yaw->init(gimbal->yaw);
	gimbal->pitch->init(gimbal->pitch);
	gimbal->watch->init(gimbal->watch);
	
	/* 云台信息初始化 */
	gimbal_info_init(gimbal->info);
	
	/* 云台命令初始化 */
	gimbal_commond_init(gimbal);
}

/**
  * @brief  云台信息初始化
  * @param  
  * @retval 
  */
void gimbal_info_init(gimbal_info_t *info)
{
	memset(info, 0, sizeof(gimbal_info_t));
}

/**
  * @brief  云台命令初始化
  * @param  
  * @retval 
  */
void gimbal_commond_init(gimbal_t *gimbal)
{
	gimbal_to_check = false;
	gimbal_out_check = false;
}

/**
  * @brief  云台控制任务
  * @param  
  * @retval 
  */
void gimbal_ctrl_task(gimbal_t *gimbal)
{
	/* 云台模式更新 */
  gimbal_mode_update(gimbal);
	
	/* 云台命令响应 */
  gimbal_commond_respond(gimbal);
	
	/* 云台工作 */
  gimbal_work(gimbal);
	
	/* 云台命令初始化 */
	gimbal_commond_init(gimbal);
}

/**
  * @brief  云台模式更新
  * @param  
  * @retval 
  */
void gimbal_mode_update(gimbal_t *gimbal)
{
	static uint8_t status_last;
	gimbal_work_info_t *work = gimbal->work;
	
	if(car.move_mode_status != status_last)
	{
		work->work_status = 0;
	}
	
	switch (car.move_mode_status)
	{
		case offline_CAR:
			work->work_status = 0;
			work->status = G_G_offline;
			break;
		case init_CAR:
			work->status = G_G_init;
			if(work->init_Y_O_N == 1)
			{
				gimbal_init_Y_O_N = true;
			}
			break;
		case machine_CAR:
			work->status = G_G_follow;
			break;
		case high_shoot_CAR:
			if(work->status != G_G_check)
			{
				work->status = G_G_shoot;
			}
			break;
		default :
			break;
	}
	
	status_last = car.move_mode_status;
}

/**
  * @brief  云台命令响应
  * @param  
  * @retval 
  */
void gimbal_commond_respond(gimbal_t *gimbal)
{
	gimbal_work_info_t *work = gimbal->work;
	
	if(gimbal_to_check == true)
	{
		if(work->status == G_G_shoot)
		{
			work->work_status = 0;
			work->status = G_G_check;
		}
	}
	if(gimbal_out_check == true)
	{
		if(work->status == G_G_check)
		{
			work->work_status = 0;
			work->status = G_G_shoot;
		}
	}
}

void gimbal_work(gimbal_t *gimbal)
{
	gimbal_work_info_t *work = gimbal->work;
	gimbal_config_t *config = gimbal->config;
	gimbal_info_t *info = gimbal->info;
	s_pitch_work_info_t *pitch_work = gimbal->pitch_work;
	s_yaw_work_info_t *yaw_work = gimbal->yaw_work;
	watch_work_info_t *watch_work = gimbal->watch_work;
	
	static uint8_t G_W_flag;
	
	switch(work->status)
	{
		case G_G_offline:
			pitch_work->status = G_P_offline;
			yaw_work->status = G_Y_offline;
			watch_work->status = G_W_offline;
			work->work_status = 0;
			work->init_Y_O_N = 0;
			break;
		case G_G_init:
			switch(work->work_status)
			{
				case 0:
					if(  gimbal->pitch->info->status == DEV_ONLINE
						&& gimbal->yaw->info->status == DEV_ONLINE
					  && gimbal->watch->info->status == DEV_ONLINE)
					{
						work->work_status = 1;
					}
					break;
				case 1:
					pitch_work->status = G_P_stop;
					yaw_work->status = G_Y_stop;
					gimbal->watch->pid_speed->info->out_max = config->watch_init_out_max;
					info->target_watch_speed = -config->watch_init_speed;
					watch_work->status = G_W_speed;
					work->work_status = 2;
					break;
				case 2:
					if(watch_work->status == G_W_lock)
					{
						gimbal->watch->pid_speed->info->out_max = config->watch_normal_out_max;
						watch_work->status = G_W_stop;
						info->target_yaw_angle = 0;
						yaw_work->status = G_Y_angle;
						work->work_status = 3;
					}
					break;
				case 3:
					if(yaw_work->status == G_Y_done)
					{
						yaw_work->status = G_Y_stop;
						info->target_watch_speed = config->watch_init_speed;
						watch_work->status = G_W_speed;
						work->work_status = 4;
					}
					break;
				case 4:
					if(watch_work->status == G_W_lock)
					{
						info->watch_angle = config->watch_up_angle;
						gimbal->watch->pid_speed->info->out_max = config->watch_normal_out_max;
						info->target_pitch_speed = config->pitch_init_speed;
						pitch_work->status = G_P_speed;
						work->work_status = 5;
					}
					break;
				case 5:
					if(pitch_work->status == G_P_lock)
					{
						info->pitch_angle = config->pitch_up_angle;
						info->target_pitch_angle = 0;
						pitch_work->status = G_P_angle;
						work->work_status = 6;
					}
					break;
				case 6:
					if(pitch_work->status == G_P_done)
					{
						work->init_Y_O_N = 1;
						work->work_status = 7;
					}
					break;
				case 7:
					info->target_pitch_angle = 0;
					pitch_work->status = G_P_angle;
					info->target_yaw_angle = 0;
					yaw_work->status = G_Y_angle;
					watch_work->status = G_W_retract;
					break;
				default:
					work->work_status = 0;
					break;
			}
			break;
		case G_G_follow:
			pitch_work->status = G_P_normal;
			info->target_yaw_angle = 0;
			yaw_work->status = G_Y_angle;
			watch_work->status = G_W_retract;
			break;
		case G_G_shoot:
			switch(work->work_status)
			{
				case 0:
					pitch_work->status = G_P_stop;
					info->target_yaw_angle = info->return_yaw_angle;
					yaw_work->status = G_Y_angle;
					watch_work->status = G_W_retract;
					work->work_status = 1;
					break;
				case 1:
					if(yaw_work->status == G_Y_done)
					{
						work->work_status = 2;
						if(info->shoot_watch_flag == 0)
						{
							watch_work->status = G_W_open;
						}
						else 
						{
							watch_work->status = G_W_retract;
						}
					}
					break;
				case 2:
					if(watch_work->status == G_W_lock)
					{
						work->work_status = 3;
						info->target_pitch_angle = info->return_pitch_angle;
						pitch_work->status = G_P_angle;
						yaw_work->status = G_Y_slow;
					}
					else if(watch_work->status == G_W_retract)
					{
						work->work_status = 3;
						info->target_pitch_angle = info->return_pitch_angle;
						pitch_work->status = G_P_angle;
						yaw_work->status = G_Y_slow;
					}
					break;
				case 3:
					if(pitch_work->status == G_P_done)
					{
						work->work_status = 4;
					}
					break;
				case 4:
					pitch_work->status = G_P_slow;
					yaw_work->status = G_Y_slow;
					if(G_W_flag != info->shoot_watch_flag)
					{
						if(info->shoot_watch_flag == 0)
						{
							watch_work->status = G_W_open;
						}
						else 
						{
							watch_work->status = G_W_retract;
						}
					}
					G_W_flag = info->shoot_watch_flag;
					break;
			}
			break;
		case G_G_check:
			switch(work->work_status)
			{
				case 0:
					info->return_pitch_angle = info->pitch_angle;
					info->return_yaw_angle = info->yaw_angle;
					pitch_work->status = G_P_stop;
					yaw_work->status = G_Y_stop;
					gimbal->watch->pid_speed->info->out_max = config->watch_init_out_max;
					info->target_watch_speed = -config->watch_init_speed;
					watch_work->status = G_W_speed;
					work->work_status = 1;
					break;
				case 1:
					if(watch_work->status == G_W_lock)
					{
						watch_work->status = G_W_retract;
						gimbal->watch->pid_speed->info->out_max = config->watch_normal_out_max;
						info->target_yaw_angle = config->yaw_check_angle;
						yaw_work->status = G_Y_angle;
						work->work_status = 2;
					}
					break;
				case 2:
					if(yaw_work->status == G_Y_done)
					{
						info->target_pitch_angle = config->pitch_check_angle;
						pitch_work->status = G_P_angle;
						work->work_status = 3;
					}
					break;
				case 3:
					if(pitch_work->status == G_P_done)
					{
						work->work_status = 4;
					}
					break;
				case 4:
//					info->target_pitch_angle = config->pitch_check_angle;
					pitch_work->status = G_P_slow;
//					info->target_yaw_angle = config->yaw_check_angle;
					yaw_work->status = G_Y_slow;
					watch_work->status = G_W_retract;
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}
	gimbal_pitch_work(gimbal);
	gimbal_yaw_work(gimbal);
	gimbal_watch_work(gimbal);
}

void gimbal_pitch_work(gimbal_t *gimbal)
{
	gimbal_config_t *config = gimbal->config;
	gimbal_info_t *info = gimbal->info;
	s_pitch_work_info_t *work = gimbal->pitch_work;
	
	switch(work->status)
	{
		case G_P_offline:
			break;
		case G_P_stop:
			info->target_pitch_speed = 0;
			gimbal_pitch_speed_ctrl(gimbal);
			break;
		case G_P_speed:
			gimbal_pitch_speed_ctrl(gimbal);
			if(info->pitch_speed == 0)
			{
				work->cnt ++;
				if(work->cnt >= config->lock_cnt_max)
				{
					work->cnt = 0;
					work->status = G_P_lock;
				}
			}
			else 
			{
				work->cnt = 0;
			}
			break;
		case G_P_lock:
			info->target_pitch_speed = 0;
			gimbal_pitch_speed_ctrl(gimbal);
			break;
		case G_P_angle:
			gimbal_pitch_angle_ctrl(gimbal);
			if(info->pitch_speed == 0)
			{
				work->cnt ++;
				if(work->cnt >= config->lock_cnt_max)
				{
					work->cnt = 0;
					work->status = G_P_done;
				}
			}
			else 
			{
				work->cnt = 0;
			}
			break;
		case G_P_done:
			info->target_pitch_speed = 0;
			gimbal_pitch_speed_ctrl(gimbal);
			break;
		case G_P_normal:
			switch(car.ctrl_mode)
			{
				case RC_CAR:
					info->target_pitch_angle += rc.base_info->ch1 / 12.f;
					gimbal_pitch_angle_check(gimbal);
					gimbal_pitch_angle_ctrl(gimbal);
					break;
				case KEY_CAR:
					info->target_pitch_angle -= rc.info->mouse_y_K / 10.f;
					gimbal_pitch_angle_check(gimbal);
					gimbal_pitch_angle_ctrl(gimbal);
					break;
				default:
					break;
			}
			break;
		case G_P_slow:
			switch(car.ctrl_mode)
			{
				case RC_CAR:
					info->target_pitch_angle += rc.base_info->ch1 / 36.f;
					gimbal_pitch_angle_check(gimbal);
					gimbal_pitch_angle_ctrl(gimbal);
					break;
				case KEY_CAR:
					info->target_pitch_angle -= rc.info->mouse_y_K / 3.f;
					gimbal_pitch_angle_check(gimbal);
					gimbal_pitch_angle_ctrl(gimbal);
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}
}

void gimbal_yaw_work(gimbal_t *gimbal)
{
	s_yaw_work_info_t *work = gimbal->yaw_work;
	gimbal_config_t *config = gimbal->config;
	gimbal_info_t *info = gimbal->info;
	
	switch(work->status)
	{
		case G_Y_offline:
			break;
		case G_Y_stop:
			info->target_yaw_speed = 0;
			gimbal_yaw_speed_ctrl(gimbal);
			break;
		case G_Y_angle:
			gimbal_yaw_angle_ctrl(gimbal);
			if(info->yaw_speed == 0)
			{
				work->cnt ++;
				if(work->cnt >= config->lock_cnt_max)
				{
					work->cnt = 0;
					work->status = G_Y_done;
				}
			}
			else 
			{
				work->cnt = 0;
			}
			break;
		case G_Y_done:
			info->target_yaw_speed = 0;
			gimbal_yaw_speed_ctrl(gimbal);
			break;
		case G_Y_normal:
			switch(car.ctrl_mode)
			{
				case RC_CAR:
					info->target_yaw_angle += rc.base_info->ch0 / 20.f;
					gimbal_yaw_angle_check(gimbal);
					gimbal_yaw_angle_ctrl(gimbal);
					break;
				case KEY_CAR:
					info->target_yaw_angle += rc.info->mouse_x_K / 20.f;
					gimbal_yaw_angle_check(gimbal);
					gimbal_yaw_angle_ctrl(gimbal);
					break;
			}
			break;
		case G_Y_slow:
			switch(car.ctrl_mode)
			{
				case RC_CAR:
					info->target_yaw_angle += rc.base_info->ch0 / 60.f;
					gimbal_yaw_angle_check(gimbal);
					gimbal_yaw_angle_ctrl(gimbal);
					break;
				case KEY_CAR:
					info->target_yaw_angle += rc.info->mouse_x_K / 20.f;
					gimbal_yaw_angle_check(gimbal);
					gimbal_yaw_angle_ctrl(gimbal);
					break;
			}
			break;
		default:
			break;
	}
}

void gimbal_watch_work(gimbal_t *gimbal)
{
	watch_work_info_t *work = gimbal->watch_work;
	gimbal_config_t *config = gimbal->config;
	gimbal_info_t *info = gimbal->info;
	
	switch(work->status)
	{
		case G_W_offline:
			break;
		case G_W_stop:
			info->target_watch_speed = 0;
			gimbal_watch_speed_ctrl(gimbal);
			break;
		case G_W_speed:
			gimbal_watch_speed_ctrl(gimbal);
			if(info->watch_speed == 0)
			{
				work->cnt ++;
				if(work->cnt >= config->lock_cnt_max)
				{
					work->cnt = 0;
					work->status = G_W_lock;
				}
			}
			else 
			{
				work->cnt = 0;
			}
			break;
		case G_W_lock:
			info->target_watch_speed = 0;
			gimbal_watch_speed_ctrl(gimbal);
			break;
		case G_W_angle:
			gimbal_watch_angle_ctrl(gimbal);
			if(info->watch_speed == 0)
			{
				work->cnt ++;
				if(work->cnt >= config->lock_cnt_max)
				{
					work->cnt = 0;
					work->status = G_W_done;
				}
			}
			else 
			{
				work->cnt = 0;
			}
			break;
		case G_W_done:
			info->target_watch_speed = 0;
			gimbal_watch_speed_ctrl(gimbal);
			break;
		case G_W_retract:
			info->target_watch_angle = -1.421 * info->pitch_angle + 40000;// config->watch_follow_angle;//(float)(info->pitch_angle - 26000) * -config->kp;//1.2;//.255;
			gimbal_watch_angle_ctrl(gimbal);
			break;
		case G_W_open:
			info->target_watch_speed = config->watch_init_speed;
			work->status = G_W_speed;
			break;
		default:
			break;
	}
}

void gimbal_pitch_can_update(gimbal_t *gimbal)
{
	gimbal_info_t *info = gimbal->info;
	motor_2006_base_info_t *base_info = gimbal->pitch->base_info;
	
	info->pitch_speed = - base_info->speed;
	info->pitch_angle -= base_info->angle_add;
}

void gimbal_yaw_can_update(gimbal_t *gimbal)
{
	gimbal_info_t *info = gimbal->info;
	gimbal_config_t *config = gimbal->config;
	motor_6020_base_info_t *base_info = gimbal->yaw->base_info;
	
	info->yaw_speed = - base_info->speed;
	info->yaw_angle = config->yaw_middle_angle - base_info->angle;
	gimbal_yaw_angle_check(gimbal);
}

void gimbal_watch_can_update(gimbal_t *gimbal)
{
	gimbal_info_t *info = gimbal->info;
	motor_2006_base_info_t *base_info = gimbal->watch->base_info;
	
	info->watch_speed = - base_info->speed;
	info->watch_angle -= base_info->angle_add;
}

void gimbal_pitch_angle_check(gimbal_t *gimbal)
{
	gimbal_info_t *info = gimbal->info;
	gimbal_config_t *config = gimbal->config;
	
	if(info->target_pitch_angle >= config->pitch_angle_permit_max)
	{
		info->target_pitch_angle = config->pitch_angle_permit_max;
	}
	if(info->target_pitch_angle <= config->pitch_angle_permit_min)
	{
		info->target_pitch_angle = config->pitch_angle_permit_min;
	}
}

void gimbal_yaw_angle_check(gimbal_t *gimbal)
{
	gimbal_info_t *info = gimbal->info;
	
	if(abs(info->yaw_angle) > 4096)
	{
		info->yaw_angle -= 8192 * sgn(info->yaw_angle);
	}
	
	if(abs(info->target_yaw_angle) > 4096)
	{
		info->target_yaw_angle -= 8192 * sgn(info->target_yaw_angle);
	}
}

void gimbal_pitch_speed_ctrl(gimbal_t *gimbal)
{
	pid_info_t *info = gimbal->pitch->pid_speed->info;
	motor_2006_t *motor = gimbal->pitch;
	
	int16_t output;
	
	info->target = gimbal->info->target_pitch_speed;
	info->measure = gimbal->info->pitch_speed;
	single_pid_cal(info);
	
	output = - info->out;
	if(motor->can->hcan == &hcan1)
	{
		can1_tx_buf[(motor->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
		can1_tx_buf[(motor->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
	}
	else if(motor->can->hcan == &hcan2)
	{
		can2_tx_buf[(motor->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
		can2_tx_buf[(motor->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
	}
}

void gimbal_pitch_angle_ctrl(gimbal_t *gimbal)
{
	pid_info_t *info = gimbal->pitch->pid_angle->info;
	
	info->target = gimbal->info->target_pitch_angle;
	info->measure = gimbal->info->pitch_angle;
	single_pid_cal(info);
	
	gimbal->info->target_pitch_speed = info->out;
	gimbal_pitch_speed_ctrl(gimbal);
}

void gimbal_yaw_speed_ctrl(gimbal_t *gimbal)
{
	pid_info_t *info = gimbal->yaw->pid_speed->info;
	motor_6020_t *motor = gimbal->yaw;
	
	int16_t output;
	
	info->target = gimbal->info->target_yaw_speed;
	info->measure = gimbal->info->yaw_speed;
	single_pid_cal(info);
	
	output = - info->out;
	if(motor->can->hcan == &hcan1)
	{
		can1_tx_buf[(motor->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
		can1_tx_buf[(motor->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
	}
	else if(motor->can->hcan == &hcan2)
	{
		can2_tx_buf[(motor->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
		can2_tx_buf[(motor->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
	}
}

void gimbal_yaw_angle_ctrl(gimbal_t *gimbal)
{
	pid_info_t *info = gimbal->yaw->pid_angle->info;
	
	int16_t err;
	
	info->target = gimbal->info->target_yaw_angle;
	info->measure = gimbal->info->yaw_angle;
	
	err = info->target - info->measure;
	if(abs(err) >= 4096)
	{
		info->target -= sgn(err) * 8192;
	}
	single_pid_cal(info);
	
	gimbal->info->target_yaw_speed = info->out;
	gimbal_yaw_speed_ctrl(gimbal);
}

void gimbal_watch_speed_ctrl(gimbal_t *gimbal)
{
	pid_info_t *info = gimbal->watch->pid_speed->info;
	motor_2006_t *motor = gimbal->watch;
	
	int16_t output;
	
	info->target = gimbal->info->target_watch_speed;
	info->measure = gimbal->info->watch_speed;
	single_pid_cal(info);
	
	output = - info->out;
	if(motor->can->hcan == &hcan1)
	{
		can1_tx_buf[(motor->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
		can1_tx_buf[(motor->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
	}
	else if(motor->can->hcan == &hcan2)
	{
		can2_tx_buf[(motor->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
		can2_tx_buf[(motor->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
	}
}

void gimbal_watch_angle_ctrl(gimbal_t *gimbal)
{
	pid_info_t *info = gimbal->watch->pid_angle->info;
	
	info->target = gimbal->info->target_watch_angle;
	info->measure = gimbal->info->watch_angle;
	single_pid_cal(info);
	
	gimbal->info->target_watch_speed = info->out;
	gimbal_watch_speed_ctrl(gimbal);
}

///**
//  * @brief  云台yaw轴can中断更新
//  * @param  
//  * @retval 
//  */
//void gimbal_yaw_can_update(gimbal_t *gimbal)
//{
//	gimbal_info_t *info = gimbal->info;
//	gimbal_config_t *config = gimbal->config;
//	motor_6020_base_info_t *yaw = gimbal->yaw->base_info;
//	
//	int16_t angle_temp = info->yaw_motor_angle;
//	
//	info->yaw_motor_angle = config->yaw_motor_angle_middle - yaw->angle;
//	info->yaw_motor_speed = - yaw->speed;
//	info->yaw_motor_dif_speed = (angle_temp - info->yaw_motor_angle) * 60000.f / 8192.f;
//	info->yaw_motor_dif_speed_ave = ave_fil_update(&yaw_dif_speed_filter, info->yaw_motor_dif_speed, 5);
//	
//	gimbal_yaw_angle_check(gimbal);
//}

///**
//  * @brief  云台pitch轴can中断更新
//  * @param  
//  * @retval 
//  */
//void gimbal_pitch_can_update(gimbal_t *gimbal)
//{
//	gimbal_info_t *info = gimbal->info;
//	gimbal_config_t *config = gimbal->config;
//	motor_6020_base_info_t *pitch = gimbal->pitch->base_info;
//	
//	int16_t angle_temp = info->pitch_motor_angle;
//	
//	info->pitch_motor_angle = config->pitch_motor_angle_middle - pitch->angle;
//	info->pitch_motor_speed = - pitch->speed;
//	info->pitch_motor_dif_speed = (info->pitch_motor_angle - angle_temp) * 60000.f / 8192.f;
//	info->pitch_motor_dif_speed_ave = ave_fil_update(&pitch_dif_speed_filter, info->pitch_motor_dif_speed, 10);
//	
//	gimbal_pitch_angle_check(gimbal);
//}

///**
//  * @brief  云台yaw轴角度检查
//  * @param  
//  * @retval 
//  */
//void gimbal_yaw_angle_check(gimbal_t *gimbal)
//{
//	gimbal_info_t *info = gimbal->info;
//	
//	float angle;
//	
//	angle = info->yaw_imu_angle;//-180°~180°
//	while (abs(angle) > 180)
//	{
//		angle -= 360 * sgn(angle);
//	}
//	info->yaw_imu_angle = angle;
//	
//	angle = info->yaw_imu_angle_target;//-180°~180°
//	while (abs(angle) > 180)
//	{
//		angle -= 360 * sgn(angle);
//	}
//	info->yaw_imu_angle_target = angle;
//	
//	angle = info->yaw_motor_angle;//-4096~4096
//	while (abs(angle) > 4096)
//	{
//		angle -= 8192 * sgn(angle);
//	}
//	info->yaw_motor_angle = angle;
//	
//	angle = info->yaw_motor_angle_target;//-4096~4096
//	while (abs(angle) > 4096)
//	{
//		angle -= 8192 * sgn(angle);
//	}
//	info->yaw_motor_angle_target = angle;
//}

///**
//  * @brief  云台pitch轴角度检查
//  * @param  
//  * @retval 
//  */
//void gimbal_pitch_angle_check(gimbal_t *gimbal)
//{
//	gimbal_info_t *info = gimbal->info;
//	
//	float angle;
//	
//	angle = info->pitch_motor_angle_target;
//	if(angle > 900)
//	{
//		angle = 900;
//	}
//	if(angle < -550)
//	{
//		angle = -550;
//	}
//	info->pitch_motor_angle_target = angle;
//	
//	angle = gimbal->info->pitch_imu_angle_target;
//	if(angle > 40)
//	{
//		angle = 40;
//	}
//	if(angle < -25)
//	{
//		angle = -25;
//	}
//	gimbal->info->pitch_imu_angle_target = angle;
//}

///**
//  * @brief  云台工作
//  * @param  
//  * @retval 
//  */
//void gimbal_work(gimbal_t *gimbal)
//{
//	switch(gimbal->info->yaw_mode)
//	{
//		case G_Y_offline:
//			break;
//		case G_Y_follow:
//			gimbal->info->yaw_motor_angle_target = 0;
//			gimbal_yaw_motor_pid_angle_ctrl(gimbal);
//			break;
//		case G_Y_gyro:
//			switch(car.ctrl_mode)
//			{
//				case RC_CAR:
//					gimbal->info->yaw_imu_angle_target += rc.base_info->ch0 / 2000.f;//最快增量330dps
//					gimbal_yaw_angle_check(gimbal);
//					gimbal_yaw_imu_pid_angle_ctrl(gimbal);
//					break;
//				case KEY_CAR:
//					gimbal->info->yaw_imu_angle_target += rc.info->mouse_x_K / 1500.f;
//					gimbal_yaw_angle_check(gimbal);
//					gimbal_yaw_imu_pid_angle_ctrl(gimbal);	
//					break;
//				default:
//					break;
//			}
//			break;
//		case G_Y_machine:
//			gimbal_yaw_motor_pid_angle_ctrl(gimbal);
//			break;
//		case G_Y_auto:
//			if(vision.yaw_err != 0)
//			{
//				gimbal->info->yaw_imu_angle_target = gimbal->info->yaw_imu_angle + vision.yaw_err;
//				vision.yaw_err = 0;
//			}
//			gimbal_yaw_imu_pid_angle_ctrl(gimbal);
//			break;
//	}
//	switch(gimbal->info->pitch_mode)
//	{
//		case G_P_offline:
//			break;
//		case G_P_gyro:
//			switch(car.ctrl_mode)
//			{
//				case RC_CAR:
//					gimbal->info->pitch_imu_angle_target += rc.base_info->ch1 / 10000.f;
//					gimbal_pitch_angle_check(gimbal);
//					gimbal_pitch_imu_pid_angle_ctrl(gimbal);
//					break;
//				case KEY_CAR:
//					gimbal->info->pitch_imu_angle_target += rc.info->mouse_y_K / 3000.f;
//					gimbal_pitch_angle_check(gimbal);
//					gimbal_pitch_imu_pid_angle_ctrl(gimbal);
//					break;
//			}
//			break;
//		case G_P_machine:
//			switch(car.ctrl_mode)
//			{
//				case RC_CAR:
//					gimbal->info->pitch_motor_angle_target += rc.base_info->ch1 / 10000.f * 22.f;
//					gimbal_pitch_angle_check(gimbal);
//					gimbal_pitch_motor_pid_angle_ctrl(gimbal);
//					break;
//				case KEY_CAR:
//					gimbal->info->pitch_motor_angle_target += rc.info->mouse_y_K / 3000.f;
//					gimbal_pitch_angle_check(gimbal);
//					gimbal_pitch_motor_pid_angle_ctrl(gimbal);
//					break;
//			}
//			break;
//		case G_P_auto:
//			if(vision.pitch_err != 0)
//			{
//				gimbal->info->pitch_imu_angle_target = gimbal->info->pitch_imu_angle + vision.pitch_err;
//				vision.pitch_err = 0;
//			}
//			gimbal_pitch_imu_pid_angle_ctrl(gimbal);
//			break;
//	}
//}

///**
//  * @brief  云台yaw轴电机速度环
//  * @param  
//  * @retval 
//  */
//void gimbal_yaw_motor_pid_speed_ctrl(gimbal_t *gimbal)
//{
//	pid_info_t *pid = gimbal->yaw->motor_pid_speed->info;
//	
//	int16_t output;
//	
//	pid->measure = gimbal->info->yaw_imu_speed;
//	single_pid_cal(pid);
//	output = pid->out;
//	if(gimbal->yaw->can->hcan == &hcan1)
//	{
//		can1_tx_buf[(gimbal->yaw->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
//		can1_tx_buf[(gimbal->yaw->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
//	}
//	else if(gimbal->yaw->can->hcan == &hcan2)
//	{
//		can2_tx_buf[(gimbal->yaw->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
//		can2_tx_buf[(gimbal->yaw->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
//	}
//}

///**
//  * @brief  云台yaw轴陀螺仪速度环
//  * @param  
//  * @retval 
//  */
//void gimbal_yaw_imu_pid_speed_ctrl(gimbal_t *gimbal)
//{
//	pid_info_t *pid = gimbal->yaw->imu_pid_speed->info;
//	
//	int16_t output;
//	
//	pid->measure = gimbal->info->yaw_imu_speed;
//	single_pid_cal(pid);
//	output = pid->out;
//	if(gimbal->yaw->can->hcan == &hcan1)
//	{
//		can1_tx_buf[(gimbal->yaw->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
//		can1_tx_buf[(gimbal->yaw->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
//	}
//	else if(gimbal->yaw->can->hcan == &hcan2)
//	{
//		can2_tx_buf[(gimbal->yaw->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
//		can2_tx_buf[(gimbal->yaw->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
//	}
//}

///**
//  * @brief  云台yaw轴电机角度环
//  * @param  
//  * @retval 
//  */
//void gimbal_yaw_motor_pid_angle_ctrl(gimbal_t *gimbal)
//{
//	pid_info_t *pid = gimbal->yaw->motor_pid_angle->info;
//	gimbal_info_t *info = gimbal->info;
//	
//	float target = info->yaw_motor_angle_target;
//	float measure = info->yaw_motor_angle;
//	float err = target - measure;
//	
//	//+-180°跳变检测
//	if(abs(err) > 4096)
//	{
//		measure -= 4096 * one(measure);
//	}
//	
//	pid->target = target;
//	pid->measure = measure;
//	
//	/* 电机精度不够，到位以后变相变为速度环，误差较大后还原 */
//	if(pid->measure == pid->target)
//	{
//		pid->blind_err = 3;
//	}
//	else if(abs(pid->measure - pid->target) >= 3)
//	{
//		pid->blind_err = 0;
//	}
//	
//	single_pid_cal(pid);
//	
//	gimbal->yaw->motor_pid_speed->info->target = pid->out;
//	gimbal_yaw_motor_pid_speed_ctrl(gimbal);
//}

///**
//  * @brief  云台yaw轴陀螺仪角度环
//  * @param  
//  * @retval 
//  */
//void gimbal_yaw_imu_pid_angle_ctrl(gimbal_t *gimbal)
//{
//	pid_info_t *pid = gimbal->yaw->imu_pid_angle->info;
//	gimbal_info_t *info = gimbal->info;
//	
//	float target = info->yaw_imu_angle_target;
//	float measure = info->yaw_imu_angle;
//	float err = target - measure;
//	
//	//+-180°跳变检测
//	if(abs(err) > 180)
//	{
//		measure -= 360.f * sgn(measure);
//	}
//	
//	pid->target = target;
//	pid->measure = measure;

//	single_pid_cal(pid);
//	
//	gimbal->yaw->imu_pid_speed->info->target = pid->out;
//	gimbal_yaw_imu_pid_speed_ctrl(gimbal);
//}

///**
//  * @brief  云台pitch轴电机速度环
//  * @param  
//  * @retval 
//  */
//void gimbal_pitch_motor_pid_speed_ctrl(gimbal_t *gimbal)
//{
//	pid_info_t *pid = gimbal->pitch->motor_pid_speed->info;
//	
//	int16_t output;
//	
//	pid->measure = gimbal->info->pitch_motor_speed;
////	pid->measure = gimbal->info->pitch_motor_dif_speed_ave * 0.3 +\
////	               gimbal->info->pitch_motor_speed * 0.7;
//	single_pid_cal(pid);
//	output = pid->out;
//	if(gimbal->pitch->can->hcan == &hcan1)
//	{
//		can1_tx_buf[(gimbal->pitch->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
//		can1_tx_buf[(gimbal->pitch->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
//	}
//	else if(gimbal->pitch->can->hcan == &hcan2)
//	{
//		can2_tx_buf[(gimbal->pitch->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
//		can2_tx_buf[(gimbal->pitch->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
//	}
//}

///**
//  * @brief  云台pitch轴陀螺仪速度环
//  * @param  
//  * @retval 
//  */
//void gimbal_pitch_imu_pid_speed_ctrl(gimbal_t *gimbal)
//{
//	pid_info_t *pid = gimbal->pitch->imu_pid_speed->info;
//	
//	int16_t output;
//	
//	pid->measure = gimbal->info->pitch_imu_speed;
//	single_pid_cal(pid);
//	output = pid->out;
//	if(gimbal->pitch->can->hcan == &hcan1)
//	{
//		can1_tx_buf[(gimbal->pitch->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
//		can1_tx_buf[(gimbal->pitch->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
//	}
//	else if(gimbal->pitch->can->hcan == &hcan2)
//	{
//		can2_tx_buf[(gimbal->pitch->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
//		can2_tx_buf[(gimbal->pitch->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
//	}
//}

///**
//  * @brief  云台pitch轴陀螺仪角度环
//  * @param  
//  * @retval 
//  */
//void gimbal_pitch_imu_pid_angle_ctrl(gimbal_t *gimbal)
//{
//	gimbal_info_t *info = gimbal->info;
//	pid_info_t *pid = gimbal->pitch->imu_pid_angle->info;
//	
//	pid->target = info->pitch_imu_angle_target;
//	pid->measure = info->pitch_imu_angle;
//	single_pid_cal(pid);
//	
//	gimbal->pitch->imu_pid_speed->info->target = pid->out;
//	gimbal_pitch_imu_pid_speed_ctrl(gimbal);
//}

///**
//  * @brief  云台pitch轴电机角度环
//  * @param  
//  * @retval 2022-2-28 15:01:00 简化并增加操作1
//  */
//void gimbal_pitch_motor_pid_angle_ctrl(gimbal_t *gimbal)
//{
//	gimbal_info_t *info = gimbal->info;
//	
//	float target = gimbal->info->pitch_motor_angle_target;
//	float measure = gimbal->info->pitch_motor_angle;
//	float err = target - measure;
//	info->pitch_imu_angle_target = lowpass(info->pitch_imu_angle_target, \
//	                                       info->pitch_imu_angle + err / 4096.f * 180.f, \
//	                                       0.05);
//	gimbal_pitch_imu_pid_angle_ctrl(gimbal);
//}

///**
//  * @brief  云台陀螺仪更新
//  * @param  
//  * @retval 
//  */
//void gimbal_imu_update(gimbal_t *gimbal)
//{
//	gimbal_info_t *info = gimbal->info;
//	imu_base_info_t *base_info = imu.base_info;
//	
//	info->yaw_imu_angle = base_info->yaw;
//	info->yaw_imu_speed = base_info->yaw_dif_speed_ave;
//	info->pitch_imu_angle = base_info->pitch;
//	info->pitch_imu_speed = base_info->pitch_dif_speed_ave;
//}
