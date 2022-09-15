/**
  ******************************************************************************
  * @file           : gimbal.c/h
  * @brief          : 
  * @note           : finish 2022-2-11 14:45:09
	*                   2022-3-11 将gimbal_to_machine命令执行从gimbal_mode_update
	*                         函数移至gimbal_commond_respond函数
	*                   2022-3-11 更新gimbal_yaw_can_update函数，增加差分速度变量
	*                   2022-3-11 更新gimbal_pitch_can_update函数，增加差分速度变量
	*                   2022-3-11 更新gimbal_yaw_angle_check函数
	*                   2022-3-11 增加对yaw_motor_angle_target的check
	*                   2022-3-11 更新gimbal_pitch_angle_check函数
  ******************************************************************************
  */

#include "gimbal.h"
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
#include "vector.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern uint8_t can1_tx_buf[16];
extern uint8_t can2_tx_buf[16];

bool gimbal_to_machine = false;
bool gimbal_init_Y_O_N = false;
bool gimbal_machine_debug_on = false;
bool gimbal_machine_debug_off = false;

gimbal_config_t gimbal_config = 
{
	.yaw_motor_angle_middle = yaw_motor_angle_middle_init,
	.pitch_motor_angle_middle = pitch_motor_angle_middle_init,
	.yaw_angle_change_shoot = yaw_angle_change_shoot_init,
	.pitch_angle_change_shoot = pitch_angle_change_shoot_init,
};
gimbal_info_t gimbal_info;
gimbal_t gimbal = 
{
	.yaw = &gimbal_yaw_motor,    //（俯视顺时针为负）
	.pitch = &gimbal_pitch_motor,//（向上为负）(-643~1055)
	.config = &gimbal_config,
	.info = &gimbal_info,
};

vector_t vector_yaw_imu_angle;
vector_t vector_yaw_imu_speed;

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
	
	/* 云台信息初始化 */
	gimbal_info_init(gimbal->info);
	
	/* 云台命令初始化 */
	gimbal_commond_init(gimbal);
	
	vector_init(&vector_yaw_imu_angle, 6);
	vector_init(&vector_yaw_imu_speed, 6);
}

/**
  * @brief  云台信息初始化
  * @param  
  * @retval 
  */
void gimbal_info_init(gimbal_info_t *info)
{
	info->pitch_mode = G_P_offline;
	info->yaw_mode = G_Y_offline;
}

/**
  * @brief  云台命令初始化
  * @param  
  * @retval 
  */
void gimbal_commond_init(gimbal_t *gimbal)
{
	gimbal_l_90 = false;
	gimbal_l_shoot = false;
	gimbal_r_90 = false;
	gimbal_r_shoot = false;
	gimbal_r_180 = false;
	gimbal_u_shoot = false;
	gimbal_d_shoot = false;
	gimbal_to_machine = false;
	gimbal_machine_debug_on = false;
	gimbal_machine_debug_off = false;
	gimbal_yaw_lock = false;
	gimbal_yaw_unlock = false;
	gimbal_pitch_lock = false;
	gimbal_pitch_unlock = false;
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
  if(car_mode_change == true)
  {
    switch (car.move_mode_status)
    {
			case offline_CAR:
        gimbal->info->yaw_mode = G_Y_offline;
        gimbal->info->pitch_mode = G_P_offline;
				break;
			case init_CAR:
				gimbal->info->yaw_mode = G_Y_follow;
        gimbal->info->pitch_mode = G_P_machine;
				if(abs(gimbal->info->yaw_motor_angle) <= 3)
				{
					gimbal_init_Y_O_N = true;
				}
				break;
      case gyro_CAR:
				gimbal->info->yaw_motor_angle_target = 0;
				gimbal->info->yaw_imu_angle_target = gimbal->info->yaw_imu_angle;
        gimbal->info->yaw_mode = G_Y_gyro;
        gimbal->info->pitch_mode = G_P_gyro;
        break;
      case fly_slope_CAR:
				gimbal->info->yaw_imu_angle_target = gimbal->info->yaw_imu_angle;
        gimbal->info->yaw_mode = G_Y_gyro;
        gimbal->info->pitch_mode = G_P_gyro;
        break;
      case high_shoot_CAR:
				gimbal->info->yaw_motor_angle_target = gimbal->info->yaw_motor_angle;
				gimbal->info->pitch_motor_angle_target = gimbal->info->pitch_motor_angle;
        gimbal->info->yaw_mode = G_Y_machine;
        gimbal->info->pitch_mode = G_P_machine;
        break;
      case machine_CAR:
				gimbal->info->yaw_imu_angle_target = gimbal->info->yaw_imu_angle;
				gimbal->info->yaw_mode = G_Y_gyro;
        gimbal->info->pitch_mode = G_P_gyro;
        break;
      default:
        break;
    }
  }
}

/**
  * @brief  云台命令响应
  * @param  
  * @retval 
  */
void gimbal_commond_respond(gimbal_t *gimbal)
{
	if(gimbal_l_90 == true)
	{
		switch(car.move_mode_status)
		{
			case gyro_CAR:
				gimbal->info->yaw_imu_angle_target -= 90;
				break;
			default:
				break;
		}
	}
	if(gimbal_l_shoot == true)
	{
		switch(car.move_mode_status)
		{
			case high_shoot_CAR:
				gimbal->info->yaw_motor_angle_target -= gimbal->config->yaw_angle_change_shoot;
				break;
			default:
				break;
		}
	}
	if(gimbal_r_90 == true)
	{
		switch(car.move_mode_status)
		{
			case gyro_CAR:
				gimbal->info->yaw_imu_angle_target += 90;
				break;
			default:
				break;
		}
	}
	if(gimbal_r_shoot == true)
	{
		switch(car.move_mode_status)
		{
			case high_shoot_CAR:
				gimbal->info->yaw_motor_angle_target += gimbal->config->yaw_angle_change_shoot;
				break;
			default:
				break;
		}
	}
	if(gimbal_r_180 == true)
	{
		switch(car.move_mode_status)
		{
			case gyro_CAR:
				gimbal->info->yaw_imu_angle_target += 180;
				break;
			default:
				break;
		}
	}
	if(gimbal_u_shoot == true)
	{
		switch(car.move_mode_status)
		{
			case high_shoot_CAR:
				gimbal->info->pitch_motor_angle_target += gimbal->config->pitch_angle_change_shoot;
				break;
			default:
				break;
		}
	}
	if(gimbal_d_shoot == true)
	{
		switch(car.move_mode_status)
		{
			case high_shoot_CAR:
				gimbal->info->pitch_motor_angle_target -= gimbal->config->pitch_angle_change_shoot;
				break;
			default:
				break;
		}
	}
	if(gimbal_to_machine == true)
	{
		switch(car.move_mode_status)
		{
			case machine_CAR:
				gimbal->info->yaw_mode = G_Y_follow;
				gimbal->info->pitch_mode = G_P_machine;
				break;
			default:
				break;
		}
	}
	if(auto_on == true)
	{
		switch(car.move_mode_status)
		{
			case gyro_CAR:
				if(gimbal->info->yaw_mode != G_Y_keep)
				{
					gimbal->info->yaw_mode = G_Y_auto;
				}
				if(gimbal->info->pitch_mode != G_P_keep)
				{
					gimbal->info->pitch_mode = G_P_auto;
				}
				break;
			default:
				break;
		}
		auto_on = false;
	}
	if(auto_off == true)
	{
		gimbal->info->yaw_motor_angle_target = 0;
		gimbal->info->yaw_mode = G_Y_gyro;
		gimbal->info->pitch_mode = G_P_gyro;
		auto_off = false;
	}
	if(gimbal_machine_debug_on == true)
	{
		switch(car.move_mode_status)
		{
			case gyro_CAR:
				gimbal->info->pitch_mode = G_P_machine;
				break;
			default:
				break;
		}
	}
	if(gimbal_machine_debug_off == true)
	{
		switch(car.move_mode_status)
		{
			case gyro_CAR:
				if(gimbal->info->pitch_mode == G_P_machine)
				{
					gimbal->info->pitch_mode = G_P_gyro;
				}
				break;
			default:
				break;
		}
	}
	if(gimbal_yaw_lock == true)
	{
		if((gimbal->info->yaw_mode == G_Y_gyro) || (gimbal->info->yaw_mode == G_Y_auto))
		{
			gimbal->info->yaw_mode = G_Y_keep;
		}
	}
	if(gimbal_yaw_unlock == true)
	{
		gimbal->info->yaw_mode = G_Y_gyro;
	}
	if(gimbal_pitch_lock == true)
	{
		if((gimbal->info->pitch_mode == G_P_gyro) || (gimbal->info->pitch_mode == G_P_auto))
		{
			gimbal->info->pitch_mode = G_P_keep;
		}
	}
	if(gimbal_pitch_unlock == true)
	{
		gimbal->info->pitch_mode = G_P_gyro;
	}
}

/**
  * @brief  云台yaw轴can中断更新
  * @param  
  * @retval 
  */
void gimbal_yaw_can_update(gimbal_t *gimbal)
{
	gimbal_info_t *info = gimbal->info;
	gimbal_config_t *config = gimbal->config;
	motor_6020_base_info_t *yaw = gimbal->yaw->base_info;
	
	info->yaw_motor_angle = config->yaw_motor_angle_middle - yaw->angle;
	info->yaw_motor_speed = - yaw->speed;
	
	gimbal_yaw_angle_check(gimbal);
}

/**
  * @brief  云台pitch轴can中断更新
  * @param  
  * @retval 
  */
void gimbal_pitch_can_update(gimbal_t *gimbal)
{
	gimbal_info_t *info = gimbal->info;
	gimbal_config_t *config = gimbal->config;
	motor_6020_base_info_t *pitch = gimbal->pitch->base_info;
	
	info->pitch_motor_angle = config->pitch_motor_angle_middle - pitch->angle;
	info->pitch_motor_speed = - pitch->speed;
	
	gimbal_pitch_angle_check(gimbal);
}

/**
  * @brief  云台yaw轴角度检查
  * @param  
  * @retval 
  */
void gimbal_yaw_angle_check(gimbal_t *gimbal)
{
	gimbal_info_t *info = gimbal->info;
	
	float angle;
	
	angle = info->yaw_imu_angle;//-180°~180°
	while (abs(angle) > 180)
	{
		angle -= 360 * sgn(angle);
	}
	info->yaw_imu_angle = angle;
	
	angle = info->yaw_imu_angle_target;//-180°~180°
	while (abs(angle) > 180)
	{
		angle -= 360 * sgn(angle);
	}
	info->yaw_imu_angle_target = angle;
	
	angle = info->yaw_motor_angle;//-4096~4096
	while (abs(angle) > 4096)
	{
		angle -= 8192 * sgn(angle);
	}
	info->yaw_motor_angle = angle;
	
	angle = info->yaw_motor_angle_target;//-4096~4096
	while (abs(angle) > 4096)
	{
		angle -= 8192 * sgn(angle);
	}
	info->yaw_motor_angle_target = angle;
}

/**
  * @brief  云台pitch轴角度检查
  * @param  
  * @retval 
  */
void gimbal_pitch_angle_check(gimbal_t *gimbal)
{
	gimbal_info_t *info = gimbal->info;
	
	float angle;
	
	angle = info->pitch_motor_angle_target;
	if(angle > 900)
	{
		angle = 900;
	}
	if(angle < -550)
	{
		angle = -550;
	}
	info->pitch_motor_angle_target = angle;
	
	angle = gimbal->info->pitch_imu_angle_target;
	if(angle > 40)
	{
		angle = 40;
	}
	if(angle < -25)
	{
		angle = -25;
	}
	gimbal->info->pitch_imu_angle_target = angle;
}

/**
  * @brief  云台工作
  * @param  
  * @retval 
  */
float vision_see;
float gimbal_see;
void gimbal_work(gimbal_t *gimbal)
{
	switch(gimbal->info->yaw_mode)
	{
		case G_Y_offline:
			break;
		case G_Y_follow:
			gimbal->info->yaw_motor_angle_target = 0;
			gimbal_yaw_motor_pid_angle_ctrl(gimbal);
			break;
		case G_Y_gyro:
			switch(car.ctrl_mode)
			{
				case RC_CAR:
					gimbal->info->yaw_imu_angle_target += rc.base_info->ch0 / 2000.f;//最快增量330dps
					gimbal_yaw_angle_check(gimbal);
					gimbal_yaw_imu_pid_angle_ctrl(gimbal);
					break;
				case KEY_CAR:
					gimbal->info->yaw_imu_angle_target += rc.info->mouse_x_K / 500.f;
					gimbal_yaw_angle_check(gimbal);
					gimbal_yaw_imu_pid_angle_ctrl(gimbal);	
					break;
				default:
					break;
			}
			break;
		case G_Y_machine:
			gimbal_yaw_motor_pid_angle_ctrl(gimbal);
			break;
		case G_Y_auto:
			if(vision.output->yaw_move != 0)
			{
				gimbal->info->yaw_imu_angle_target = gimbal->info->yaw_imu_angle + vision.output->yaw_move;
				vision_see = gimbal->info->yaw_imu_angle_target;
				gimbal_yaw_angle_check(gimbal);
				vision.output->yaw_move= 0;
			}
			gimbal_yaw_imu_pid_angle_ctrl(gimbal);
			break;
		case G_Y_keep:
			gimbal_yaw_imu_pid_angle_ctrl(gimbal);
			break;
		default:
			break;
	}
	switch(gimbal->info->pitch_mode)
	{
		case G_P_offline:
			break;
		case G_P_gyro:
			switch(car.ctrl_mode)
			{
				case RC_CAR:
					gimbal->info->pitch_imu_angle_target += rc.base_info->ch1 / 10000.f;
					gimbal_pitch_angle_check(gimbal);
					gimbal_pitch_imu_pid_angle_ctrl(gimbal);
					break;
				case KEY_CAR:
					gimbal->info->pitch_imu_angle_target -= rc.info->mouse_y_K / 1000.f;
					gimbal_pitch_angle_check(gimbal);
					gimbal_pitch_imu_pid_angle_ctrl(gimbal);
					break;
			}
			break;
		case G_P_machine:
			switch(car.ctrl_mode)
			{
				case RC_CAR:
					gimbal->info->pitch_motor_angle_target += rc.base_info->ch1 / 10000.f * 22.f;
					gimbal_pitch_angle_check(gimbal);
					gimbal_pitch_motor_pid_angle_ctrl(gimbal);
					break;
				case KEY_CAR:
					gimbal->info->pitch_motor_angle_target -= rc.info->mouse_y_K / 3000.f;
					gimbal_pitch_angle_check(gimbal);
					gimbal_pitch_motor_pid_angle_ctrl(gimbal);
					break;
			}
			break;
		case G_P_auto:
			if(vision.output->pitch_move != 0)
			{
				gimbal->info->pitch_imu_angle_target = gimbal->info->pitch_imu_angle + vision.output->pitch_move;
				gimbal_pitch_angle_check(gimbal);
				vision.output->pitch_move = 0;
			}
			gimbal_pitch_imu_pid_angle_ctrl(gimbal);
			break;
		case G_P_keep:
			gimbal_pitch_imu_pid_angle_ctrl(gimbal);
			break;
	}
}

/**
  * @brief  云台yaw轴电机速度环
  * @param  
  * @retval 
  */
void gimbal_yaw_motor_pid_speed_ctrl(gimbal_t *gimbal)
{
	pid_info_t *pid = gimbal->yaw->motor_pid_speed->info;
	
	int16_t output;
	
	pid->measure = gimbal->info->yaw_imu_speed;
	single_pid_cal(pid);
	output = pid->out;
	if(gimbal->yaw->can->hcan == &hcan1)
	{
		can1_tx_buf[(gimbal->yaw->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
		can1_tx_buf[(gimbal->yaw->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
	}
	else if(gimbal->yaw->can->hcan == &hcan2)
	{
		can2_tx_buf[(gimbal->yaw->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
		can2_tx_buf[(gimbal->yaw->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
	}
}

/**
  * @brief  云台yaw轴陀螺仪速度环
  * @param  
  * @retval 
  */
void gimbal_yaw_imu_pid_speed_ctrl(gimbal_t *gimbal)
{
	pid_info_t *pid = gimbal->yaw->imu_pid_speed->info;
	
	int16_t output;
	
	pid->measure = gimbal->info->yaw_imu_speed;
	single_pid_cal(pid);
	output = pid->out;
	if(gimbal->yaw->can->hcan == &hcan1)
	{
		can1_tx_buf[(gimbal->yaw->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
		can1_tx_buf[(gimbal->yaw->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
	}
	else if(gimbal->yaw->can->hcan == &hcan2)
	{
		can2_tx_buf[(gimbal->yaw->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
		can2_tx_buf[(gimbal->yaw->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
	}
}

/**
  * @brief  云台yaw轴电机角度环
  * @param  
  * @retval 
  */
void gimbal_yaw_motor_pid_angle_ctrl(gimbal_t *gimbal)
{
	pid_info_t *pid = gimbal->yaw->motor_pid_angle->info;
	gimbal_info_t *info = gimbal->info;
	
	float target = info->yaw_motor_angle_target;
	float measure = info->yaw_motor_angle;
	float err = target - measure;
	
	//+-180°跳变检测
	if(abs(err) > 4096)
	{
		measure -= 4096 * one(measure);
	}
	
	pid->target = target;
	pid->measure = measure;
	
	/* 电机精度不够，到位以后变相变为速度环，误差较大后还原 */
	if(pid->measure == pid->target)
	{
		pid->blind_err = 3;
	}
	else if(abs(pid->measure - pid->target) >= 3)
	{
		pid->blind_err = 0;
	}
	
	single_pid_cal(pid);
	
	gimbal->yaw->motor_pid_speed->info->target = pid->out;
	gimbal_yaw_motor_pid_speed_ctrl(gimbal);
}

/**
  * @brief  云台yaw轴陀螺仪角度环
  * @param  
  * @retval 
  */
void gimbal_yaw_imu_pid_angle_ctrl(gimbal_t *gimbal)
{
	pid_info_t *pid = gimbal->yaw->imu_pid_angle->info;
	gimbal_info_t *info = gimbal->info;
	
	float target = info->yaw_imu_angle_target;
	float measure = info->yaw_imu_angle;
	float err = target - measure;
	
	//+-180°跳变检测
	if(abs(err) > 180)
	{
		measure -= 360.f * sgn(measure);
	}
	
	pid->target = target;
	pid->measure = measure;

	single_pid_cal(pid);
	
	gimbal->yaw->imu_pid_speed->info->target = pid->out;
	gimbal_yaw_imu_pid_speed_ctrl(gimbal);
}

/**
  * @brief  云台pitch轴电机速度环
  * @param  
  * @retval 
  */
void gimbal_pitch_motor_pid_speed_ctrl(gimbal_t *gimbal)
{
	pid_info_t *pid = gimbal->pitch->motor_pid_speed->info;
	
	int16_t output;
	
	pid->measure = gimbal->info->pitch_motor_speed;
	single_pid_cal(pid);
	output = pid->out;
	if(gimbal->pitch->can->hcan == &hcan1)
	{
		can1_tx_buf[(gimbal->pitch->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
		can1_tx_buf[(gimbal->pitch->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
	}
	else if(gimbal->pitch->can->hcan == &hcan2)
	{
		can2_tx_buf[(gimbal->pitch->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
		can2_tx_buf[(gimbal->pitch->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
	}
}

/**
  * @brief  云台pitch轴陀螺仪速度环
  * @param  
  * @retval 
  */
void gimbal_pitch_imu_pid_speed_ctrl(gimbal_t *gimbal)
{
	pid_info_t *pid = gimbal->pitch->imu_pid_speed->info;
	
	int16_t output;
	
	pid->measure = gimbal->info->pitch_imu_speed;
	single_pid_cal(pid);
	output = pid->out;
	if(gimbal->pitch->can->hcan == &hcan1)
	{
		can1_tx_buf[(gimbal->pitch->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
		can1_tx_buf[(gimbal->pitch->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
	}
	else if(gimbal->pitch->can->hcan == &hcan2)
	{
		can2_tx_buf[(gimbal->pitch->can->rx_id - 0x201) * 2] = (output >> 8) & 0xFF;
		can2_tx_buf[(gimbal->pitch->can->rx_id - 0x201) * 2 + 1] = (output & 0xFF);
	}
}

/**
  * @brief  云台pitch轴陀螺仪角度环
  * @param  
  * @retval 
  */
void gimbal_pitch_imu_pid_angle_ctrl(gimbal_t *gimbal)
{
	gimbal_info_t *info = gimbal->info;
	pid_info_t *pid = gimbal->pitch->imu_pid_angle->info;
	
	pid->target = info->pitch_imu_angle_target;
	pid->measure = info->pitch_imu_angle;
	single_pid_cal(pid);
	
	gimbal->pitch->imu_pid_speed->info->target = pid->out;
	gimbal_pitch_imu_pid_speed_ctrl(gimbal);
}

/**
  * @brief  云台pitch轴电机角度环
  * @param  
  * @retval 2022-2-28 15:01:00 简化并增加操作1
  */
void gimbal_pitch_motor_pid_angle_ctrl(gimbal_t *gimbal)
{
	gimbal_info_t *info = gimbal->info;
	
	float target = gimbal->info->pitch_motor_angle_target;
	float measure = gimbal->info->pitch_motor_angle;
	float err = target - measure;
	info->pitch_imu_angle_target = lowpass(info->pitch_imu_angle_target, \
	                                       info->pitch_imu_angle + err / 4096.f * 180.f, \
	                                       0.05);
	gimbal_pitch_imu_pid_angle_ctrl(gimbal);
}

/**
  * @brief  云台陀螺仪更新
  * @param  
  * @retval 
  */
void gimbal_imu_update(gimbal_t *gimbal)
{
	gimbal_info_t *info = gimbal->info;
	imu_base_info_t *base_info = imu.base_info;
	
	info->yaw_imu_angle = base_info->yaw;
	vector_update(&vector_yaw_imu_angle, info->yaw_imu_angle);
	info->yaw_imu_speed = base_info->yaw_dif_speed_ave;
	info->pitch_imu_angle = base_info->pitch;
	info->pitch_imu_speed = base_info->pitch_dif_speed_ave;
  
	vector_update(&vector_yaw_imu_speed, info->yaw_imu_speed);
}
