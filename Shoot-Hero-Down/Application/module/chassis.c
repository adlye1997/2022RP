/**
  ******************************************************************************
  * @file           : chassis.c\h
	* @author         : czf
	* @date           : 
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */

#include "chassis.h"
#include "string.h"
#include "chassis_motor.h"
#include "config.h"
#include "config_chassis.h"
#include "gimbal.h"
#include "math.h"
#include "car.h"
#include "judge.h"
#include "rp_math.h"
#include "math_support.h"
#include "tracking_differentiator.h"
#include "usmart.h"
#include "debug.h"
#include "math_support.h"

chassis_config_t chassis_config = 
{
	.chassis_cycle_speed_mode = chassis_cycle_speed_mode_init,
	.chassis_target_speed_mode = chassis_target_speed_mode_init,
	.chassis_cycle_mode_0_speed = chassis_cycle_mode_0_speed_init,
	.chassis_cycle_mode_1_speed = chassis_cycle_mode_1_speed_init,
	.chassis_cycle_mode_2_speed = chassis_cycle_mode_2_speed_init,
	.chassis_speed_max = 8000,
	.chassis_output_max = 50000,
};
chassis_info_t chassis_info;
chassis_t chassis = 
{
	.motor_LF = &chassis_LF,
	.motor_RF = &chassis_RF,
	.motor_LB = &chassis_LB,
	.motor_RB = &chassis_RB,
	.config = &chassis_config,
	.info = &chassis_info,
};

extern tracking_differentiator_info_t chassis_front_speed_track_tor_info;
tracking_differentiator_t chassis_front_speed_track_tor = 
{
	.info = &chassis_front_speed_track_tor_info,
};
extern tracking_differentiator_info_t chassis_right_speed_track_tor_info;
tracking_differentiator_t chassis_right_speed_track_tor = 
{
	.info = &chassis_right_speed_track_tor_info,
};

/**
  * @brief  底盘初始化
  * @param  
  * @retval 
  */
void chassis_init(chassis_t *chassis)
{
	/* 初始化四个电机 */
	chassis->motor_LF->init(chassis->motor_LF);
	chassis->motor_RF->init(chassis->motor_RF);
	chassis->motor_LB->init(chassis->motor_LB);
	chassis->motor_RB->init(chassis->motor_RB);
	
	/* 底盘命令初始化 */
	chassis_commond_init(chassis);
	
	/* 底盘信息初始化 */
	chassis_info_init(chassis->info);
}

/**
  * @brief  底盘命令初始化
  * @param  
  * @retval 
  */
void chassis_commond_init(chassis_t *chassis)
{
  chassis_cycle_off = false;
  chassis_cycle_on = false;
  chassis_cycle_change = false;
}

/**
  * @brief  底盘信息初始化
  * @param  
  * @retval 
  */
void chassis_info_init(chassis_info_t *info)
{
	info->target_front_speed = 0;
	info->target_right_speed = 0;
	info->target_cycle_speed = 0;
	info->move_mode = C_M_offline;
	info->cycle_mode = C_C_offline;
	info->back_cnt = 0;
	info->speed_cnt = 0;
}

/**
  * @brief  底盘控制任务
  * @param  
  * @retval 
  */
void chassis_ctrl_task(chassis_t *chassis)
{
	/* 底盘模式更新 */
  chassis_mode_update(chassis);
	
	/* 底盘当前速度更新 */
	chassis_measure_speed_update(chassis);
	
	/* 底盘命令响应 */
  chassis_commond_respond(chassis);
	
	/* 底盘工作 */
  chassis_work(chassis);
	
	/* 底盘裁判系统限制 */
	chassis_judge_limit(chassis);
	
	/* 命令初始化 */
	chassis_commond_init(chassis);
}

/**
  * @brief  底盘模式更新
  * @param  
  * @retval 
  */
void chassis_mode_update(chassis_t *chassis)
{
  switch (car.move_mode_status)
  {
    case offline_CAR:
      chassis->info->move_mode = C_M_offline;
      chassis->info->cycle_mode = C_C_offline;
      break;
    case init_CAR:
      chassis->info->move_mode = C_M_stop;
      chassis->info->cycle_mode = C_C_stop;
      break;
    case high_shoot_CAR:
      chassis->info->move_mode = C_M_stop;
      chassis->info->cycle_mode = C_C_stop;
      break;
    case machine_CAR:
      chassis->info->back_cnt = 0;
      chassis->info->move_mode = C_M_normal;
      chassis->info->cycle_mode = C_C_normal;
      break;
    default:
      break;
  }
}

/**
  * @brief  底盘当前速度更新
  * @param  
  * @retval 
  */
void chassis_measure_speed_update(chassis_t *chassis)
{
	int16_t motor_speed_LF = chassis->motor_LF->base_info->speed;
	int16_t motor_speed_RF = chassis->motor_RF->base_info->speed;
	int16_t motor_speed_LB = chassis->motor_LB->base_info->speed;
	int16_t motor_speed_RB = chassis->motor_RB->base_info->speed;
	
	int16_t front_speed = chassis->info->measure_front_speed;
	int16_t right_speed = chassis->info->measure_right_speed;
	
	chassis->info->measure_front_speed = (motor_speed_LF - motor_speed_RF + motor_speed_LB - motor_speed_RB) / 4;
	chassis->info->measure_right_speed = (motor_speed_LF + motor_speed_RF - motor_speed_LB - motor_speed_RB) / 4;
	chassis->info->measure_cycle_speed = (motor_speed_LF + motor_speed_RF + motor_speed_LB + motor_speed_RB) / 4;
	
	chassis->info->measure_front_speed_dif = chassis->info->measure_front_speed - front_speed;
	chassis->info->measure_right_speed_dif = chassis->info->measure_right_speed - right_speed;
	
//	chassis_measure_speed_send();
}

/**
  * @brief  底盘命令响应
  * @param  
  * @retval 
  */
void chassis_commond_respond(chassis_t *chassis)
{
}

/**
  * @brief  底盘工作
  * @param  
  * @retval 
  */
void chassis_work(chassis_t *chassis)
{
	chassis_config_t *config = chassis->config;
	int16_t front, right;
	
  switch(chassis->info->move_mode)
  {
    case C_M_offline:
      break;
    case C_M_stop:
      chassis->info->target_front_speed = 0;
      chassis->info->target_right_speed = 0;
      break;
    case C_M_normal:
      switch(car.ctrl_mode)
      {
        case RC_CAR:
          chassis->info->target_front_speed = (float)rc.base_info->ch3 * (float)config->chassis_speed_max / 660.f;
          chassis->info->target_right_speed = (float)rc.base_info->ch2 * (float)config->chassis_speed_max / 660.f;
          break;
        case KEY_CAR:
					/* 根据按键计数梯形变化 */
					front = 0;
					right = 0;
					if(chassis->config->chassis_target_speed_mode == 0)
					{
						front += (float)rc.base_info->W.cnt / (float)KEY_W_CNT_MAX * (float)config->chassis_speed_max;
						front -= (float)rc.base_info->S.cnt / (float)KEY_S_CNT_MAX * (float)config->chassis_speed_max;
						right += (float)rc.base_info->D.cnt / (float)KEY_D_CNT_MAX * (float)config->chassis_speed_max;
						right -= (float)rc.base_info->A.cnt / (float)KEY_A_CNT_MAX * (float)config->chassis_speed_max;
					}
					/* 单值过微分跟踪器 */
					else if(chassis->config->chassis_target_speed_mode == 1)
					{
						/* 前后速度 */
						front = (rc.base_info->W.value - rc.base_info->S.value) * config->chassis_speed_max;
						if(front * chassis->info->measure_front_speed < 0)
						{
							front = 0;
							tracking_differentiator_reset(&chassis_front_speed_track_tor, \
							                              chassis->info->measure_front_speed, \
							                              chassis->info->measure_front_speed_dif);
						}
						else if(abs(chassis->info->measure_front_speed) > abs(front))
						{
							tracking_differentiator_reset(&chassis_front_speed_track_tor, \
							                              chassis->info->measure_front_speed, \
							                              chassis->info->measure_front_speed_dif);
						}
						else 
						{
							tracking_differentiator_update(&chassis_front_speed_track_tor,front);
							front = chassis_front_speed_track_tor.info->output_value;
						}
						
						/* 左右速度 */
						right = (rc.base_info->D.value - rc.base_info->A.value) * config->chassis_speed_max;
						if(right * chassis->info->measure_right_speed < 0)
						{
							right = 0;
							tracking_differentiator_reset(&chassis_right_speed_track_tor, \
							                              chassis->info->measure_right_speed, \
							                              chassis->info->measure_right_speed_dif);
						}
						else if(abs(chassis->info->measure_right_speed) > abs(right))
						{
							tracking_differentiator_reset(&chassis_right_speed_track_tor, \
							                              chassis->info->measure_right_speed, \
							                              chassis->info->measure_right_speed_dif);
						}
						else 
						{
							tracking_differentiator_update(&chassis_right_speed_track_tor,right);
							right = chassis_right_speed_track_tor.info->output_value;
						}
					}
					else 
					{
					}
					/* 赋值 */
          chassis->info->target_front_speed = front;
					chassis->info->target_right_speed = right;
          break;
        default:
          break;
      }
      break;
		}
	
	switch(chassis->info->cycle_mode)
	{
		case C_C_stop:
			chassis->info->target_cycle_speed = 0;
			break;
		case C_C_normal:
			switch(car.ctrl_mode)
			{
				case RC_CAR:
					chassis->info->target_cycle_speed = (float)rc.base_info->ch0 / 660.f * config->chassis_speed_max;
					break;
				case KEY_CAR:
					chassis->info->target_cycle_speed = (float)rc.info->mouse_x_K * 10.f;
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}
	
	/* 底盘电机更新from底盘 */
	motor_chassis_update(chassis);
	
	/* 电机工作 */
	chassis->motor_LF->ctrl(chassis->motor_LF);
	chassis->motor_RF->ctrl(chassis->motor_RF);
	chassis->motor_LB->ctrl(chassis->motor_LB);
	chassis->motor_RB->ctrl(chassis->motor_RB);
}

//void chassis_cycle_auto_check(void)
//{
//	int16_t *speed;
//	
//	static uint16_t cnt = 0;
//	uint8_t flag;
//	int16_t config_speed_change = 100;
//	int16_t config_time = 200;
//	
//	/* 周期任务 */
//	if((cnt == 0)&&(judge.info->status == DEV_ONLINE))
//	{
//		/* 判断是否超功率 */
//		if(judge.base_info->chassis_power_buffer == 60)
//		{
//			flag = 1; //未超功率
//		}
//		else 
//		{
//			flag = 0; //超功率
//		}
//		/* 简化后续代码 */
//		switch(chassis.config->chassis_cycle_speed_mode)
//		{
//			case 0:
//				speed = &chassis.config->chassis_cycle_mode_0_speed;
//				break;
//			case 1:
//				speed = &chassis.config->chassis_cycle_mode_1_speed;
//				break;
//			case 2:
//				speed = &chassis.config->chassis_cycle_mode_2_speed;
//				break;
//		}
////		/* 反馈信息 */
////		uint8_t txBuf[40] = "cycle_mode= ,speed=    ,buffer=  ";
////		num_to_str_2(chassis.config->chassis_cycle_speed_mode, &txBuf[11], 1);
////		num_to_str_2(abs(*speed), &txBuf[19], 4);
////		num_to_str_2(abs(judge.base_info->chassis_power_buffer), &txBuf[31], 2);
////		usmart_send_data(txBuf, 33);
//		/* 自适应 */
//		if(flag == 0)
//		{
//			(*speed) -= config_speed_change;
//			if((*speed) < 0)
//			{
//				(*speed) = 0;
//			}
//		}
//		else if(flag == 1)
//		{
//			(*speed) += config_speed_change;
//		}
//	}
//	cnt++;
//	cnt	%= config_time;
//}

/**
  * @brief  底盘裁判系统限制
  * @param  
  * @retval 
  */
int32_t out_LF_1,out_RF_1,out_LB_1,out_RB_1;
int32_t out_LF,out_RF,out_LB,out_RB;
int32_t out_LF_err,out_RF_err,out_LB_err,out_RB_err;
int16_t front_err;
int16_t right_err;
int16_t cycle_err;
void chassis_judge_limit(chassis_t *chassis)
{
	extern uint8_t can1_tx_buf[16];
	int32_t sum;
	float Real_PowerBuffer, Limit_k;
	sum = 0;
	out_LF = chassis->motor_LF->pid_speed->info->out;
	out_RF = chassis->motor_RF->pid_speed->info->out;
	out_LB = chassis->motor_LB->pid_speed->info->out;
	out_RB = chassis->motor_RB->pid_speed->info->out;
	sum = abs(out_LF) + abs(out_RF) + abs(out_LB) + abs(out_RB);
	
	if(judge.info->status == DEV_ONLINE)
	{
		Real_PowerBuffer = judge.base_info->power_heat_data.chassis_power_buffer;
		Real_PowerBuffer = constrain(Real_PowerBuffer, 0, 60);
		Limit_k = Real_PowerBuffer / 60.f;
		if(Real_PowerBuffer < 18)
		{
			Limit_k = pow(Limit_k,3);
		}
		else 
		{
			Limit_k = pow(Limit_k,2);
		}
		judge.base_info->chassis_out_put_max = Limit_k * (float)chassis->config->chassis_output_max;
	}
	else 
	{
		judge.base_info->chassis_out_put_max = (float)chassis->config->chassis_output_max;
	}
  
  /* 直线校正begin */
	front_err = sgn(chassis->info->target_front_speed - chassis->info->measure_front_speed);
	right_err = sgn(chassis->info->target_right_speed - chassis->info->measure_right_speed);
	cycle_err = sgn(chassis->info->target_cycle_speed - chassis->info->measure_cycle_speed);
	out_LF += abs(out_LF) * (float)( front_err + right_err + cycle_err) * 0.1;
	out_RF += abs(out_RF) * (float)(-front_err + right_err + cycle_err) * 0.1;
	out_LB += abs(out_LB) * (float)( front_err - right_err + cycle_err) * 0.1;
	out_RB += abs(out_RB) * (float)(-front_err - right_err + cycle_err) * 0.1;
  sum = abs(out_LF) + abs(out_RF) + abs(out_LB) + abs(out_RB);
	/* 直线校正end */
	
	if(sum > judge.base_info->chassis_out_put_max)
	{
		Limit_k = (float)judge.base_info->chassis_out_put_max / (float)sum;
		out_LF = (float)out_LF * Limit_k;
		out_RF = (float)out_RF * Limit_k;
		out_LB = (float)out_LB * Limit_k;
		out_RB = (float)out_RB * Limit_k;
    
    front_err = out_RB;
		
		can1_tx_buf[(chassis->motor_LF->can->rx_id - 0x201) * 2] = (out_LF >> 8) & 0xFF;
		can1_tx_buf[(chassis->motor_LF->can->rx_id - 0x201) * 2 + 1] = (out_LF & 0xFF);
		can1_tx_buf[(chassis->motor_RF->can->rx_id - 0x201) * 2] = (out_RF >> 8) & 0xFF;
		can1_tx_buf[(chassis->motor_RF->can->rx_id - 0x201) * 2 + 1] = (out_RF & 0xFF);
		can1_tx_buf[(chassis->motor_LB->can->rx_id - 0x201) * 2] = (out_LB >> 8) & 0xFF;
		can1_tx_buf[(chassis->motor_LB->can->rx_id - 0x201) * 2 + 1] = (out_LB & 0xFF);
		can1_tx_buf[(chassis->motor_RB->can->rx_id - 0x201) * 2] = (out_RB >> 8) & 0xFF;
		can1_tx_buf[(chassis->motor_RB->can->rx_id - 0x201) * 2 + 1] = (out_RB & 0xFF);
	}
}

/**
  * @brief  底盘电机更新from底盘
  * @param  
  * @retval 
  */
void motor_chassis_update(chassis_t *chassis)
{
	int16_t front, right, round;
	
	/* 底盘电机速度限制 */
	motor_speed_limit(chassis, &front, &right, &round);
	
	chassis->motor_LF->pid_speed->info->target =   front + right + round;
	chassis->motor_RF->pid_speed->info->target = - front + right + round;
	chassis->motor_LB->pid_speed->info->target =   front - right + round;
	chassis->motor_RB->pid_speed->info->target = - front - right + round;
}

/**
  * @brief  底盘电机速度限制
  * @param  
  * @retval 
  */
void motor_speed_limit(chassis_t *chassis, int16_t *front, int16_t *right, int16_t *round)
{
	int16_t speed_sum;
	float K;
	speed_sum = abs(chassis->info->target_front_speed) + \
              abs(chassis->info->target_right_speed) + \
              abs(chassis->info->target_cycle_speed);
	if(speed_sum > chassis->config->chassis_speed_max)
	{
		K = (float)chassis->config->chassis_speed_max / (float)speed_sum;
	}
	else 
	{
		K = 1;
	}
	*front = chassis->info->target_front_speed * K;
	*right = chassis->info->target_right_speed * K;
	*round = chassis->info->target_cycle_speed * K;
	chassis->info->target_front_speed = (float)chassis->info->target_front_speed * K;
	chassis->info->target_right_speed = (float)chassis->info->target_right_speed * K;
	chassis->info->target_cycle_speed = (float)chassis->info->target_cycle_speed * K;
}
