/**
  ******************************************************************************
  * @file           : chassis.c/h
  * @brief          : 
  * @note           : finish 2022-2-11 12:33:13
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

extern bool gimbal_to_machine;
extern uint8_t can1_tx_buf[16];

chassis_config_t chassis_config = 
{
	.chassis_cycle_speed_mode = chassis_cycle_speed_mode_init,
	.chassis_target_speed_mode = chassis_target_speed_mode_init,
	.chassis_cycle_mode_0_speed = chassis_cycle_mode_0_speed_init,
	.chassis_cycle_mode_1_speed = chassis_cycle_mode_1_speed_init,
	.chassis_cycle_mode_2_speed = chassis_cycle_mode_2_speed_init,
	.chassis_output_max = chassis_output_max_init,
	.chassis_speed_max = chassis_speed_max_init,
	.straight_rectify_coefficient = straight_rectify_coefficient_init,
};
chassis_info_t chassis_info;
chassis_t chassis = 
{
	.motor = chassis_motor,
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
  * @brief  ���̳�ʼ��
  * @param  
  * @retval 
  */
void chassis_init(chassis_t *chassis)
{
	/* ��ʼ���ĸ���� */
	chassis->motor[0].init(&chassis->motor[0]);
	chassis->motor[1].init(&chassis->motor[1]);
	chassis->motor[2].init(&chassis->motor[2]);
	chassis->motor[3].init(&chassis->motor[3]);
	
	/* ���������ʼ�� */
	chassis_commond_init(chassis);
	
	/* ������Ϣ��ʼ�� */
	chassis_info_init(chassis->info);
}

/**
  * @brief  ���������ʼ��
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
  * @brief  ������Ϣ��ʼ��
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
  * @brief  ���̿�������
  * @param  
  * @retval 
  */
void chassis_ctrl_task(chassis_t *chassis)
{
	/* ����ģʽ���� */
  chassis_mode_update(chassis);
	
	/* ���̵�ǰ�ٶȸ��� */
	chassis_measure_speed_update(chassis);
	
	/* ����������Ӧ */
  chassis_commond_respond(chassis);
	
	/* ���̹��� */
  chassis_work(chassis);
	
	/* ���̲���ϵͳ���� */
	chassis_judge_limit(chassis);
	
	/* �����ʼ�� */
	chassis_commond_init(chassis);
}

/**
  * @brief  ����ģʽ����
  * @param  
  * @retval 
  */
void chassis_mode_update(chassis_t *chassis)
{
	int16_t yaw_angle = gimbal.info->yaw_motor_angle;
	
	/* ģʽ�л� */
  if(car_mode_change == true)
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
      case gyro_CAR:
        chassis->info->move_mode = C_M_special;
        chassis->info->cycle_mode = C_C_follow;
        break;
      case fly_slope_CAR:
        chassis->info->move_mode = C_M_special;
        chassis->info->cycle_mode = C_C_follow;
        break;
      case high_shoot_CAR:
        chassis->info->move_mode = C_M_stop;
        chassis->info->cycle_mode = C_C_stop;
        break;
      case machine_CAR:
				chassis->info->back_cnt = 0;
        chassis->info->move_mode = C_M_special;
        chassis->info->cycle_mode = C_C_back;
        break;
      default :
        break;
    }
  }
	
	/* ��еģʽ���⴦�� */
	if(car.move_mode_status == machine_CAR)
	{
		/* δ��ʱ */
		if(chassis->info->back_cnt < chassis_back_time)
		{
			chassis->info->back_cnt++;
			/* �ѹ�λ */
			if(yaw_angle == 0)
			{
				chassis->info->move_mode = C_M_normal;
				chassis->info->cycle_mode = C_C_normal;
				gimbal_to_machine = true;
				chassis->info->back_cnt = chassis_back_time + 1;
			}
		}
		/* ��ʱ */
		else if(chassis->info->back_cnt == chassis_back_time)
		{
			chassis->info->move_mode = C_M_normal;
			chassis->info->cycle_mode = C_C_normal;
			gimbal_to_machine = true;
			chassis->info->back_cnt = chassis_back_time + 1;
		}
		/* ��� */
		else 
		{
		}
	}
}

/**
  * @brief  ���̵�ǰ�ٶȸ���
  * @param  
  * @retval 
  */
void chassis_measure_speed_update(chassis_t *chassis)
{
	int16_t motor_speed_LF = chassis->motor[LF].base_info->speed;
	int16_t motor_speed_RF = chassis->motor[RF].base_info->speed;
	int16_t motor_speed_LB = chassis->motor[LB].base_info->speed;
	int16_t motor_speed_RB = chassis->motor[RB].base_info->speed;
	
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
  * @brief  ����������Ӧ
  * @param  
  * @retval 
  */
void chassis_commond_respond(chassis_t *chassis)
{
	/* chassis_cycle_off��С���� */
  if(chassis_cycle_off == true)
  {
    if(chassis->info->cycle_mode == C_C_cycle)
    {
      chassis->info->cycle_mode = C_C_follow;
    }
		else
		{
		}
  }
	else
	{
	}
	
	/* chassis_cycle_on��С���� */
  if(chassis_cycle_on == true)
  {
    if(chassis->info->cycle_mode == C_C_follow)
    {
      chassis->info->cycle_mode = C_C_cycle;
    }
//		else if(chassis->info->cycle_mode == C_C_cycle && car.ctrl_mode == RC_CAR)
//		{
//			chassis->config->chassis_cycle_speed_mode++;
//			chassis->config->chassis_cycle_speed_mode %= 3;
//		}
		else
		{
		}
  }
	else
	{
	}
	
	/* chassis_cycle_changeС�����л�*/
  if(chassis_cycle_change == true)
  {
    if(chassis->info->cycle_mode == C_C_follow)
    {
      chassis->info->cycle_mode = C_C_cycle;
    }
    else if(chassis->info->cycle_mode == C_C_cycle)
    {
      chassis->info->cycle_mode = C_C_follow;
    }
		else
		{
		}
  }
	else
	{
	}
	
	if(chassis_speed_limit == true)
	{
		chassis_speed_max_auto_set(chassis);
	}
	else 
	{
		chassis->config->chassis_speed_max = 8000;
	}
}

void chassis_speed_max_auto_set(chassis_t *chassis)
{
	switch(judge.base_info->chassis_power_limit)
	{
		default:
			chassis->config->chassis_speed_max = 5000;
	}
}

/**
  * @brief  ���̹���
  * @param  
  * @retval 
  */
void chassis_work(chassis_t *chassis)
{
	int16_t yaw_angle_err, front, right;
	float yaw_angle_err_f;  //yaw��ǶȻ����ƣ�һȦ2�У�
	
	yaw_angle_err = gimbal.info->yaw_motor_angle;
	yaw_angle_err_f = (double)yaw_angle_err / 4096.f * 3.14159;
	
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
          chassis->info->target_front_speed = (float)rc.base_info->ch3 * (float)chassis->config->chassis_speed_max / 660.f;
          chassis->info->target_right_speed = (float)rc.base_info->ch2 * (float)chassis->config->chassis_speed_max / 660.f;
          break;
        case KEY_CAR:
					/* ���ݰ����������α仯 */
					front = 0;
					right = 0;
					if(chassis->config->chassis_target_speed_mode == 0)
					{
						front += (float)rc.base_info->W.cnt / (float)KEY_W_CNT_MAX * (float)chassis->config->chassis_speed_max;
						front -= (float)rc.base_info->S.cnt / (float)KEY_S_CNT_MAX * (float)chassis->config->chassis_speed_max;
						right += (float)rc.base_info->D.cnt / (float)KEY_D_CNT_MAX * (float)chassis->config->chassis_speed_max;
						right -= (float)rc.base_info->A.cnt / (float)KEY_A_CNT_MAX * (float)chassis->config->chassis_speed_max;
					}
					/* ��ֵ��΢�ָ����� */
					else if(chassis->config->chassis_target_speed_mode == 1)
					{
						/* ǰ���ٶ� */
						front = (rc.base_info->W.value - rc.base_info->S.value) * chassis->config->chassis_speed_max;
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
						
						/* �����ٶ� */
						right = (rc.base_info->D.value - rc.base_info->A.value) * chassis->config->chassis_speed_max;
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
					/* ��ֵ */
          chassis->info->target_front_speed = front;
					chassis->info->target_right_speed = right;
          break;
        default:
          break;
      }
      break;
    case C_M_special:
			switch(car.ctrl_mode)
      {
        case RC_CAR:
          chassis->info->target_front_speed = rc.base_info->ch3 * cos(yaw_angle_err_f) - rc.base_info->ch2 * sin(yaw_angle_err_f);
					chassis->info->target_right_speed  = rc.base_info->ch2 * cos(yaw_angle_err_f) + rc.base_info->ch3 * sin(yaw_angle_err_f);
					chassis->info->target_front_speed *= chassis->config->chassis_speed_max / 660.f;
					chassis->info->target_right_speed *= chassis->config->chassis_speed_max / 660.f;
          break;
        case KEY_CAR:
					front = 0;
					right = 0;
				
					/* ���ݰ����������α仯 */
					if(chassis->config->chassis_target_speed_mode == 0)
					{
						front += (float)rc.base_info->W.cnt / (float)KEY_W_CNT_MAX * (float)chassis->config->chassis_speed_max;
						front -= (float)rc.base_info->S.cnt / (float)KEY_S_CNT_MAX * (float)chassis->config->chassis_speed_max;
						right += (float)rc.base_info->D.cnt / (float)KEY_D_CNT_MAX * (float)chassis->config->chassis_speed_max;
						right -=(float)rc.base_info->A.cnt / (float)KEY_A_CNT_MAX * (float)chassis->config->chassis_speed_max;
					}
					
					/* ��ֵ��΢�ָ����� */
					else if(chassis->config->chassis_target_speed_mode == 1)
					{
						/* ǰ���ٶ� */
						front = (rc.base_info->W.value - rc.base_info->S.value) * chassis->config->chassis_speed_max;
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
						
						/* �����ٶ� */
						right = (rc.base_info->D.value - rc.base_info->A.value) * chassis->config->chassis_speed_max;
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
					
					/* ��ֵ */
				  chassis->info->target_front_speed = front * cos(yaw_angle_err_f) - right * sin(yaw_angle_err_f);
					chassis->info->target_right_speed  = right * cos(yaw_angle_err_f) + front * sin(yaw_angle_err_f);
          break;
        default:
          break;
      }
      break;
  }
	
	switch(chassis->info->cycle_mode)
	{
		case C_C_offline:
			break;
		case C_C_stop:
			chassis->info->target_cycle_speed = 0;
			break;
		case C_C_normal:
			switch(car.ctrl_mode)
			{
				case RC_CAR:
					chassis->info->target_cycle_speed = rc.base_info->ch0 * chassis->config->chassis_speed_max / 660;
					break;
				case KEY_CAR:
					chassis->info->target_cycle_speed = rc.info->mouse_x_K;
					break;
				default:
					break;
			}
			break;
			
		case C_C_follow:
			
			//����ǰ��ɵߵ�
			if(abs(yaw_angle_err) > 2048)
			{
				yaw_angle_err -= 4096 * sgn(yaw_angle_err);
			}
			
			//����
			if(abs(yaw_angle_err) >= 100)
			{
				chassis->info->target_cycle_speed = yaw_angle_err * 6.f - 300.f * sgn(yaw_angle_err);
			}
			else 
			{
				chassis->info->target_cycle_speed = yaw_angle_err * 3.f;
			}
			
			chassis->info->speed_cnt = 0;
			
			break;
			
		case C_C_back:
			chassis->info->target_cycle_speed = yaw_angle_err * 5;
			break;
		case C_C_cycle:
			switch(chassis->config->chassis_cycle_speed_mode)
			{
				case 0:
					/*����ת�� */
					chassis->info->target_cycle_speed = chassis->config->chassis_cycle_mode_0_speed;
					break;
				case 1:
					/* �ȼ���ת�� */
					if(chassis->info->speed_cnt <= chassis_cycle_T)
					{
						chassis->info->target_cycle_speed = (float)chassis->config->chassis_cycle_mode_1_speed / (float)chassis_cycle_T * chassis->info->speed_cnt + 2000;
					}
					else if(chassis->info->speed_cnt <= chassis_cycle_T * 2)
					{
						chassis->info->target_cycle_speed = (float)chassis->config->chassis_cycle_mode_1_speed / (float)chassis_cycle_T * (2 * chassis_cycle_T - chassis->info->speed_cnt) + 2000;
					}
					else 
					{
						chassis->info->speed_cnt = 0;
					}
					chassis->info->speed_cnt++;
					break;
				case 2:
					/* ���Ǻ���ʽ */
					if(chassis->info->speed_cnt <= 2 * chassis_cycle_T)
					{
						chassis->info->target_cycle_speed = (float)chassis->config->chassis_cycle_mode_2_speed * (float)sin(chassis->info->speed_cnt / (float)chassis_cycle_T * (float)3.1415);
					}
					else 
					{
						chassis->info->speed_cnt = 0;
					}
					chassis->info->speed_cnt++;
					break;
				default:
					break;
			}
			chassis_cycle_debug();
			break;
		default:
			break;
	}
	
	/* ���̵������from���� */
	motor_chassis_update(chassis);
	
	/* ������� */
	chassis->motor[LF].ctrl(&chassis->motor[LF]);
	chassis->motor[RF].ctrl(&chassis->motor[RF]);
	chassis->motor[LB].ctrl(&chassis->motor[LB]);
	chassis->motor[RB].ctrl(&chassis->motor[RB]);
}

/**
  * @brief  ���̲���ϵͳ����
  * @param  
  * @retval 
  */
void chassis_judge_limit(chassis_t *chassis)
{
	chassis_config_t *config = chassis->config;
	
	float Real_PowerBuffer, Limit_k;
	
	/* ������������ */
	int32_t out_LF,out_RF,out_LB,out_RB;
	out_LF = chassis->motor[LF].pid_speed->info->out;
	out_RF = chassis->motor[RF].pid_speed->info->out;
	out_LB = chassis->motor[LB].pid_speed->info->out;
	out_RB = chassis->motor[RB].pid_speed->info->out;
	
	/* �������������judge.base_info->chassis_out_put_max */
	if(judge.info->status == DEV_ONLINE)
	{
		Real_PowerBuffer = judge.base_info->chassis_power_buffer;
		Real_PowerBuffer = constrain(Real_PowerBuffer, 0, judge.config->buffer_max);
		Limit_k = Real_PowerBuffer / (float)judge.config->buffer_max;
		if(Real_PowerBuffer < 18)
		{
			Limit_k = pow(Limit_k,3);
		}
		else 
		{
			Limit_k = pow(Limit_k,2);
		}
		judge.base_info->chassis_out_put_max = Limit_k * (float)config->chassis_output_max;
	}
	else 
	{
		judge.base_info->chassis_out_put_max = 1600;
	}
		
	/* ֱ��У��begin */
	int16_t front_err, right_err, cycle_err;
	front_err = sgn(chassis->info->target_front_speed - chassis->info->measure_front_speed);
	right_err = sgn(chassis->info->target_right_speed - chassis->info->measure_right_speed);
	cycle_err = sgn(chassis->info->target_cycle_speed - chassis->info->measure_cycle_speed);
	out_LF += abs(out_LF) * (float)( front_err + right_err + cycle_err) * config->straight_rectify_coefficient;
	out_RF += abs(out_RF) * (float)(-front_err + right_err + cycle_err) * config->straight_rectify_coefficient;
	out_LB += abs(out_LB) * (float)( front_err - right_err + cycle_err) * config->straight_rectify_coefficient;
	out_RB += abs(out_RB) * (float)(-front_err - right_err + cycle_err) * config->straight_rectify_coefficient;
	/* ֱ��У��end */
	
	/* ��������ܺ� */
	int32_t out_sum;
	out_sum = abs(out_LF) + abs(out_RF) + abs(out_LB) + abs(out_RB);
	
	/* ���Ƶ���������� */
	if(out_sum > judge.base_info->chassis_out_put_max)
	{
		Limit_k = (float)judge.base_info->chassis_out_put_max / (float)out_sum;
		out_LF = (float)out_LF * Limit_k;
		out_RF = (float)out_RF * Limit_k;
		out_LB = (float)out_LB * Limit_k;
		out_RB = (float)out_RB * Limit_k;
	}
	
	can1_tx_buf[(chassis->motor[LF].can->rx_id - 0x201) * 2] = (out_LF >> 8) & 0xFF;
	can1_tx_buf[(chassis->motor[LF].can->rx_id - 0x201) * 2 + 1] = (out_LF & 0xFF);
	can1_tx_buf[(chassis->motor[RF].can->rx_id - 0x201) * 2] = (out_RF >> 8) & 0xFF;
	can1_tx_buf[(chassis->motor[RF].can->rx_id - 0x201) * 2 + 1] = (out_RF & 0xFF);
	can1_tx_buf[(chassis->motor[LB].can->rx_id - 0x201) * 2] = (out_LB >> 8) & 0xFF;
	can1_tx_buf[(chassis->motor[LB].can->rx_id - 0x201) * 2 + 1] = (out_LB & 0xFF);
	can1_tx_buf[(chassis->motor[RB].can->rx_id - 0x201) * 2] = (out_RB >> 8) & 0xFF;
	can1_tx_buf[(chassis->motor[RB].can->rx_id - 0x201) * 2 + 1] = (out_RB & 0xFF);
}

/**
  * @brief  ���̵������from����
  * @param  
  * @retval 
  */
void motor_chassis_update(chassis_t *chassis)
{
	int16_t front, right, round;
	
	/* ���̵���ٶ����� */
	motor_speed_limit(chassis, &front, &right, &round);
	
	chassis->motor[LF].pid_speed->info->target =   front + right + round;
	chassis->motor[RF].pid_speed->info->target = - front + right + round;
	chassis->motor[LB].pid_speed->info->target =   front - right + round;
	chassis->motor[RB].pid_speed->info->target = - front - right + round;
}

/**
  * @brief  ���̵���ٶ�����
  * @param  
  * @retval 
  */
void motor_speed_limit(chassis_t *chassis, int16_t *front, int16_t *right, int16_t *round)
{
	chassis_config_t *config = chassis->config;
	
	int16_t speed_sum;
	float K;
	
	speed_sum = abs(chassis->info->target_front_speed) \
            + abs(chassis->info->target_right_speed) \
            + abs(chassis->info->target_cycle_speed);
	
	if(speed_sum > config->chassis_speed_max)
	{
		K = (float)config->chassis_speed_max / (float)speed_sum;
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
