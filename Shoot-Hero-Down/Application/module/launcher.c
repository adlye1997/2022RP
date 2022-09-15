/**
  ******************************************************************************
  * @file           : launcher.c/h
  * @brief          : 
  * @note           : 
  ******************************************************************************
  */

#include "launcher.h"
#include "launcher_motor.h"
#include "config.h"
#include "config_launcher.h"
#include "car.h"
#include "main.h"
#include "judge.h"
#include "math_support.h"
#include "string.h"

extern uint8_t can1_tx_buf[16];
extern uint8_t can2_tx_buf[16];
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

launcher_config_t launcher_config = 
{
	.lock_cnt_max = 100,
	.init_cnt_max = 20,
	.servo_work_cnt_max = 600,
	
	.pitch_distance = 6700000,  //5800000
	.pitch_distance_permit_max = 6500000,  //5600000
	.pitch_distance_permit_min = 0,
	.pitch_init_speed = 2000,
	.launcher_pitch_init_output_max = 1000,
	.launcher_pitch_normal_output_max = 12500,
	.launcher_pitch_move_angle = 8000,
	
	.yaw_half_distance = 2300000,  //2100000
	.yaw_distance_permit_max = 2200000,  //2000000
	.yaw_distance_permit_min = -2200000,
	.yaw_init_speed = 8000,
	.launcher_yaw_init_output_max = 2000,
	.launcher_yaw_normal_output_max = 9000,
	.launcher_yaw_move_angle = 8000,
	
	.push_return_distance = 955000,  //970000
  .mode_return_push_return_distance_dif = 100000,
	.push_ready_speed = -4000,
	
	.servo_open_num = 27,
	.servo_close_num = 65,
	
	.dial_reload_angle = 31450,
	.dial_unload_angle = 3246,
	
	.launcher_light_check_time_max = 2000,
	.launcher_light_init_time_max = 500,
	.launcher_light_ball_fall_time_max = 1500,
	.launcher_light_ball_jump_time_max = 300,
	.auto_delay_cnt_max = 1000,
};
launcher_info_t launcher_info;
launcher_work_info_t launcher_work_info;
pitch_work_info_t pitch_work_info;
yaw_work_info_t yaw_work_info;
push_work_info_t push_work_info;
dial_work_info_t dial_work_info;
servo_work_info_t servo_work_info;
light_work_info_t light_work_info;
launcher_t launcher = 
{
	.yaw = &yaw,
	.pitch = &pitch,
	.push = &push,
	.dial = &dial,
	
	.config = &launcher_config,
	.info = &launcher_info,
	.launcher_work = &launcher_work_info,
	.pitch_work = &pitch_work_info,
	.yaw_work = &yaw_work_info,
	.push_work = &push_work_info,
	.dial_work = &dial_work_info,
	.servo_work = &servo_work_info,
	.light_work = &light_work_info,
};

/**
  * @brief  发射机构初始化
  * @param  
  * @retval 
  */
void launcher_init(launcher_t *launcher)
{
	/* 发射机构电机初始化 */
	launcher->pitch->init(launcher->pitch);
	launcher->yaw->init(launcher->yaw);
	launcher->push->init(launcher->push);
	launcher->dial->init(launcher->dial);
	SERVO_PWM = launcher->config->servo_close_num;
	
	/* 发射机构信息初始化 */
  launcher_info_init(launcher->info);
	
	/* 工作信息初始化 */
	launcher_work_info_init(launcher->launcher_work);
	launcher_pitch_work_init(launcher->pitch_work);
	launcher_yaw_work_init(launcher->yaw_work);
	launcher_push_work_init(launcher->push_work);
	launcher_dial_work_init(launcher->dial_work);
	launcher_servo_work_init(launcher->servo_work);
	
	/* 发射机构命令初始化 */
	launcher_commond_init(launcher);
}

/**
  * @brief  发射机构信息初始化
  * @param  
  * @retval 
  */
void launcher_info_init(launcher_info_t *info)
{
	memset(info,0,sizeof(launcher_info_t));
}

/**
  * @brief  发射机构工作信息初始化
  * @param  
  * @retval 
  */
void launcher_work_info_init(launcher_work_info_t *info)
{	
	info->command = L_L_done;
	info->status = L_L_done;
	info->init_Y_O_N = 0;
}

void launcher_pitch_work_init(pitch_work_info_t *info)
{
	info->command = L_PI_relax;
	info->init_cnt = 0;
	info->init_Y_O_N = 0;
	info->status = L_PI_relax;
}

void launcher_yaw_work_init(yaw_work_info_t *info)
{
	info->command = L_Y_relax;
	info->init_cnt = 0;
	info->init_Y_O_N = 0;
	info->status = L_Y_relax;
}

void launcher_push_work_init(push_work_info_t *info)
{
	info->cnt = 0;
	info->status = L_PU_relax;
}

void launcher_dial_work_init(dial_work_info_t *info)
{
	info->status = L_D_relax;
}

void launcher_servo_work_init(servo_work_info_t *info)
{
	info->status = L_S_done;
	info->cnt = 0;
}

/**
  * @brief  发射机构命令初始化
  * @param  
  * @retval 
  */
void launcher_commond_init(launcher_t *launcher)
{
  Launcher_Shoot = false;
	Launcher_Relax = false;
	Launcher_Ready = false;
	Launcher_Return = false;
	dial_reload = false;
	dial_unload = false;
	launcher_u = false;
	launcher_d = false;
	launcher_l = false;
	launcher_r = false;
	launcher_u_s = false;
	launcher_d_s = false;
	launcher_l_s = false;
	launcher_r_s = false;
  launcher_auto = false;
  Launcher_Init = false;
}

/**
  * @brief  发射机构控制任务
  * @param  
  * @retval 
  */
void launcher_ctrl_task(launcher_t *launcher)
{
	/* 模式更新 */
	launcher_mode_update(launcher);
	
//	/* 裁判系统限制 */
//	judge_limit(launcher);
	
	/* 发射机构命令响应 */
	launcher_commond_respond(launcher);
	
	/* 发射机构工作 */
	launcher_work(launcher);
	
	/* 发射机构命令初始化 */
	launcher_commond_init(launcher);
}

/**
  * @brief  发射机构模式更新
  * @param  
  * @retval 
  */
void launcher_mode_update(launcher_t *launcher)
{
	uint8_t *command = &launcher->launcher_work->command;
	
	switch(car.move_mode_status)
	{
		case offline_CAR:
			*command = L_L_offline;
			break;
		case init_CAR:
			break;
		case high_shoot_CAR:
			break;
		default:
			break;
	}
}

/**
  * @brief  发射机构命令响应
  * @param  
  * @retval
  */
void launcher_commond_respond(launcher_t *launcher)
{
	launcher_work_info_t *launcher_work = launcher->launcher_work;
	launcher_info_t *info = launcher->info;
	launcher_config_t *config = launcher->config;
	
	if(Launcher_Shoot == true)
	{
		launcher_work->command = L_L_shoot;
	}
	if(Launcher_Ready == true)
	{
		launcher_work->command = L_L_ready;
	}
	if(Launcher_Return == true)
	{
		launcher_work->command = L_L_return;
	}
  if(launcher_auto == true)
  {
    launcher_work->command = L_L_auto;
  }
  if(Launcher_Init == true)
  {
    launcher_work->command = L_L_init;
  }
	if((Launcher_Relax == true)&&(launcher_work->status != L_L_relax))
	{
		launcher_work->command = L_L_relax;
	}
	if(dial_reload == true)
	{
		info->target_dial_angle += config->dial_reload_angle;
		launcher->dial_work->status = L_D_angle;
	}
	if(dial_unload == true)
	{
		info->target_dial_angle -= config->dial_unload_angle;
		launcher->dial_work->status = L_D_angle;
	}
	if(launcher_u == true)
	{
		info->target_pitch_angle += config->launcher_pitch_move_angle;
		launcher_pitch_angle_check(launcher);
	}
	if(launcher_d == true)
	{
		info->target_pitch_angle -= config->launcher_pitch_move_angle;
		launcher_pitch_angle_check(launcher);
	}
	if(launcher_l == true)
	{
		info->target_yaw_angle -= config->launcher_yaw_move_angle;
		launcher_yaw_angle_check(launcher);
	}
	if(launcher_r == true)
	{
		info->target_yaw_angle += config->launcher_yaw_move_angle;
		launcher_yaw_angle_check(launcher);
	}
	if(launcher_u_s == true)
	{
		info->target_pitch_angle += config->launcher_pitch_move_angle / 25;
		launcher_pitch_angle_check(launcher);
	}
	if(launcher_d_s == true)
	{
		info->target_pitch_angle -= config->launcher_pitch_move_angle / 25;
		launcher_pitch_angle_check(launcher);
	}
	if(launcher_l_s == true)
	{
		info->target_yaw_angle -= config->launcher_yaw_move_angle / 25;
		launcher_yaw_angle_check(launcher);
	}
	if(launcher_r_s == true)
	{
		info->target_yaw_angle += config->launcher_yaw_move_angle / 25;
		launcher_yaw_angle_check(launcher);
	}
}

/**
  * @brief  发射机构工作
  * @param  
  * @retval 
  */
uint16_t time;
void launcher_work(launcher_t *launcher)
{
	launcher_config_t *config = launcher->config;
	launcher_info_t *info = launcher->info;
	launcher_work_info_t *launcher_work = launcher->launcher_work;
	pitch_work_info_t *pitch_work = launcher->pitch_work;
	yaw_work_info_t *yaw_work = launcher->yaw_work;
	push_work_info_t *push_work = launcher->push_work;
	dial_work_info_t *dial_work = launcher->dial_work;
	servo_work_info_t *servo_work = launcher->servo_work;
	
	if(launcher_work->command == L_L_stop)
	{
		launcher_work->status = L_L_stop;
	}
	else if(launcher_work->command == L_L_offline)
	{
		launcher_work->status = L_L_offline;
	}
	else if(launcher_work->command == L_L_init)
	{
		launcher_work->status = L_L_init;
	}
	
	switch (launcher_work->status)
	{
		case L_L_offline:
			pitch_work->status = L_PI_relax;
			yaw_work->status = L_Y_relax;
			push_work->status = L_PU_relax;
			dial_work->status = L_D_relax;
      launcher_work->work_status = 0;
			launcher_work->status = L_L_done;
			break;
		case L_L_stop:
			pitch_work->status = L_PI_stop;
			yaw_work->status = L_Y_stop;
			push_work->status = L_PU_stop;
			dial_work->status = L_D_stop;
      launcher_work->work_status = 0;
			launcher_work->status = L_L_done;
			break;
		case L_L_init:
			pitch_work->status = L_PI_init;
			yaw_work->status = L_Y_init;
			push_work->status = L_PU_stop;
			dial_work->status = L_D_stop;
      launcher_work->work_status = 0;
			if( (launcher->pitch_work->init_Y_O_N == 1) &&\
				  (launcher->yaw_work->init_Y_O_N == 1) )
			{
				launcher_work->init_Y_O_N = 1;
				launcher_work->status = L_L_done;
			}
			break;
		case L_L_ready:
			pitch_work->status = L_PI_normal;
			yaw_work->status = L_Y_normal;
			switch(launcher_work->work_status)
			{
				case 0:
					push_work->status = L_PU_stop;
					dial_work->status = L_D_stop;
					launcher_work->work_status = 1;
					break;
				case 1:
					info->target_push_speed = config->push_ready_speed;
					push_work->status = L_PU_speed;
					launcher_work->work_status = 2;
					break;
				case 2:
					if(push_work->status == L_PU_lock)
					{
						SERVO_PWM = config->servo_open_num;
						servo_work->status = L_S_work;
						launcher_work->work_status = 3;
					}
					break;
				case 3:
					if(servo_work->status == L_S_done)
					{
						info->target_push_speed = config->push_ready_speed;
						push_work->status = L_PU_speed;
						launcher_work->work_status = 4;
					}
					break;
				case 4:
					if(push_work->status == L_PU_lock)
					{
						SERVO_PWM = config->servo_close_num;
						servo_work->status = L_S_work;
						launcher_work->work_status = 5;
					}
					break;
				case 5:
					if(servo_work->status == L_S_done)
					{
						info->push_angle = 0;
						info->target_push_angle = config->push_return_distance;
						push_work->status = L_PU_angle;
						launcher_work->work_status = 6;
					}
					break;
				case 6:
					if(push_work->status == L_PU_done)
					{
						launcher_work->ready_shoot_Y_O_N = 1;
						launcher_work->status = L_L_done;
						launcher_work->work_status = 0;
					}
					break;
				default :
          launcher_work->work_status = 0;
					break;
			}
			break;
		case L_L_shoot:
			pitch_work->status = L_PI_normal;
			yaw_work->status = L_Y_normal;
      push_work->status = L_PU_stop;
			dial_work->status = L_D_stop;
			switch(launcher_work->work_status)
			{
				case 0:
          SERVO_PWM = config->servo_open_num;
					servo_work->status = L_S_work;
					launcher_work->work_status = 1;
					break;
				case 1:
					if(servo_work->status == L_S_done)
					{
						launcher_work->ready_shoot_Y_O_N = 0;
						launcher_work->status = L_L_done;
						launcher_work->work_status = 0;
					}
					break;
        default:
          launcher_work->work_status = 0;
          break;
			}
			break;
		case L_L_return:
			pitch_work->status = L_PI_normal;
			yaw_work->status = L_Y_normal;
			dial_work->status = L_D_stop;
			switch(launcher_work->work_status)
			{
				case 0:
					push_work->status = L_PU_stop;
					launcher_work->work_status = 1;
					break;
				case 1:
					info->target_push_speed = config->push_ready_speed;
					push_work->status = L_PU_speed;
					launcher_work->work_status = 2;
					break;
				case 2:
					if(push_work->status == L_PU_lock)
					{
						SERVO_PWM = config->servo_open_num;
						servo_work->status = L_S_work;
						launcher_work->work_status = 3;
					}
					break;
				case 3:
					if(servo_work->status == L_S_done)
					{
						info->push_angle = 0;
						info->target_push_angle = config->push_return_distance - config->mode_return_push_return_distance_dif;
						push_work->status = L_PU_angle;
						launcher_work->work_status = 4;
					}
					break;
				case 4:
					if(push_work->status == L_PU_done)
					{
						launcher_work->ready_shoot_Y_O_N = 0;
						launcher_work->status = L_L_done;
						launcher_work->work_status = 0;
					}
					break;
        default:
          launcher_work->work_status = 0;
          break;
			}
			break;
		case L_L_auto:
			pitch_work->status = L_PI_stop;
			yaw_work->status = L_Y_stop;
			switch(launcher_work->work_status)
			{
				case 0:
					push_work->status = L_PU_stop;
					dial_work->status = L_D_stop;
					launcher_work->work_status = 1;
					break;
				case 1:
					info->target_push_speed = config->push_ready_speed;
					push_work->status = L_PU_speed;
					launcher_work->work_status = 2;
					break;
				case 2:
					if(push_work->status == L_PU_lock)
					{
						SERVO_PWM = config->servo_open_num;
						servo_work->status = L_S_work;
						launcher_work->work_status = 3;
					}
					break;
				case 3:
					if(servo_work->status == L_S_done)
					{
						info->target_push_speed = config->push_ready_speed;
						push_work->status = L_PU_speed;
						launcher_work->work_status = 4;
					}
					break;
				case 4:
					if(push_work->status == L_PU_lock)
					{
						SERVO_PWM = config->servo_close_num;
						servo_work->status = L_S_work;
						launcher_work->work_status = 5;
					}
					break;
				case 5:
					if(servo_work->status == L_S_done)
					{
            if(launcher->info->light_read_result == HAVE)
            {
              launcher_work->status = L_L_relax;
            }
            else 
            {
              info->push_angle = 0;
              info->target_push_angle = config->push_return_distance;
              push_work->status = L_PU_angle;
              info->target_dial_angle += config->dial_reload_angle;
              dial_work->status = L_D_angle;
              time = 0;
              launcher_work->work_status = 6;
            }
					}
					break;
				case 6:
					if(time <= config->launcher_light_check_time_max)
					{
						switch(launcher_light_check(launcher, time))
						{
							case L_LI_done:
								launcher_work->work_status = 7;
								break;
							case L_LI_checking:
								break;
							default:
								launcher_work->status = L_L_relax;
								break;
						}
						time++;
					}
					else 
					{
						launcher_work->status = L_L_relax;
					}
					break;
				case 7:
					if(push_work->status == L_PU_done)
					{
						SERVO_PWM = config->servo_open_num;
						servo_work->status = L_S_work;
						launcher_work->work_status = 8;
						break;
					}
					break;
				case 8:
					if(servo_work->status == L_S_done)
					{
						launcher_work->work_status = 0;
            launcher_work->status = L_L_done;
					}
					break;
        default:
          break;
			}
			break;
		case L_L_relax:
      pitch_work->status = L_PI_normal;
      yaw_work->status = L_Y_normal;
      launcher_work->work_status = 0;
			switch(launcher_work->command)
			{
				case L_L_ready:
					launcher_work->status = L_L_ready;
					launcher_work->work_status = 0;
					break;
				case L_L_shoot:
					if(launcher_work->ready_shoot_Y_O_N == 1)
					{
						launcher_work->status = L_L_shoot;
						launcher_work->work_status = 0;
					}
					else 
					{
						launcher_work->status = L_L_ready;
						launcher_work->work_status = 0;
					}
					break;
				case L_L_return:
					launcher_work->status = L_L_return;
					launcher_work->work_status = 0;
					break;
        case L_L_auto:
          launcher_work->status = L_L_auto;
					launcher_work->work_status = 0;
          break;
        case L_L_init:
          launcher_work->status = L_L_init;
					launcher_work->work_status = 0;
				default:
					break;
			}
			break;
		case L_L_done:
			if(launcher_work->command == L_L_relax)
			{
					launcher_work->status = L_L_relax;
			}
			break;
		default:
			break;
	}
	
	launcher_pitch_work(launcher);
	launcher_yaw_work(launcher);
	launcher_push_work(launcher);
	launcher_dial_work(launcher);
	launcher_servo_work(launcher);
	
	info->light_read_result = HAL_GPIO_ReadPin(REBOOT_READ_GPIO_Port, REBOOT_READ_Pin);
}

/**
  * @brief  发射机构pitch轴工作
  * @param  
  * @retval 
  */
void launcher_pitch_work(launcher_t *launcher)
{
	pitch_work_info_t *pitch_work = launcher->pitch_work;
	launcher_config_t *config = launcher->config;
	launcher_info_t *info = launcher->info;
	
	switch(pitch_work->status)
	{
		case L_PI_relax:
			info->target_pitch_angle = info->pitch_angle;
			launcher_pitch_angle_check(launcher);
			break;
		case L_PI_stop:
			launcher_pitch_angle_ctrl(launcher);
			break;
		case L_PI_init:
			if(pitch_work->init_Y_O_N == 0)
			{
				launcher->pitch->pid_speed->info->out_max = config->launcher_pitch_init_output_max;
				info->target_pitch_speed = config->pitch_init_speed;
				launcher_pitch_speed_ctrl(launcher);
				if(info->pitch_speed == 0 && launcher->pitch->info->status == DEV_ONLINE)
				{
					pitch_work->init_cnt++;
					if(pitch_work->init_cnt >= config->init_cnt_max)
					{
						launcher->pitch->pid_speed->info->out_max = config->launcher_pitch_normal_output_max;
						info->pitch_angle = config->pitch_distance;
						info->target_pitch_angle = config->pitch_distance;
						pitch_work->init_cnt = 0;
						pitch_work->init_Y_O_N = 1;
						launcher_pitch_angle_check(launcher);
					}
				}
				else //速度不为0
				{
					pitch_work->init_cnt = 0;
				}
			}
			else //初始化完成
			{
				launcher_pitch_angle_ctrl(launcher);
			}
			break;
		case L_PI_normal:
			if(car.ctrl_mode == RC_CAR && car.move_mode_status == high_shoot_CAR)
			{
				launcher->info->target_pitch_angle += rc.base_info->ch3;
				launcher_pitch_angle_check(launcher);
			}
			launcher_pitch_angle_ctrl(launcher);
			break;
		default:
			break;
	}
}

/**
  * @brief  发射机构yaw轴工作
  * @param  
  * @retval finish
  */
void launcher_yaw_work(launcher_t *launcher)
{
	yaw_work_info_t *yaw_work = launcher->yaw_work;
	launcher_config_t *config = launcher->config;
	launcher_info_t *info = launcher->info;
	
	switch(yaw_work->status)
	{
		case L_Y_relax:
			info->target_yaw_angle = info->yaw_angle;
			launcher_yaw_angle_check(launcher);
			break;
		case L_Y_stop:
			launcher_yaw_angle_ctrl(launcher);
			break;
		case L_Y_init:
			if(yaw_work->init_Y_O_N == 0)
			{
				launcher->yaw->pid_speed->info->out_max = config->launcher_yaw_init_output_max;
				info->target_yaw_speed = config->yaw_init_speed;
				launcher_yaw_speed_ctrl(launcher);
				if(launcher->yaw->pid_speed->info->measure == 0 && launcher->yaw->info->status == DEV_ONLINE)
				{
					yaw_work->init_cnt++;
					if(yaw_work->init_cnt >= config->init_cnt_max)
					{
						launcher->yaw->pid_speed->info->out_max = config->launcher_yaw_normal_output_max;
						info->yaw_angle = config->yaw_half_distance;
						info->target_yaw_angle = 0;
						yaw_work->init_cnt = 0;
						yaw_work->init_Y_O_N = 1;
						launcher_yaw_angle_check(launcher);
					}
				}
				else //速度不为0
				{
					yaw_work->init_cnt = 0;
				}
			}
			else //初始化完成
			{
				launcher_yaw_angle_ctrl(launcher);
			}
			break;
		case L_Y_normal:
			if(car.ctrl_mode == RC_CAR && car.move_mode_status == high_shoot_CAR)
			{
				launcher->info->target_yaw_angle += rc.base_info->ch2;
				launcher_yaw_angle_check(launcher);
			}
			launcher_yaw_angle_ctrl(launcher);
			break;
		default:
			break;
	}
}

/**
  * @brief  发射机构push工作
  * @param  
  * @retval finish
  */
void launcher_push_work(launcher_t *launcher)
{
	push_work_info_t *push_work = launcher->push_work;
	launcher_config_t *config = launcher->config;
	launcher_info_t *info = launcher->info;
  
  if(push_work->status != L_PU_speed && push_work->status != L_PU_angle)
  {
     push_work->cnt = 0;
  }
	
	switch(push_work->status)
	{
		case L_PU_relax:
			info->target_push_angle = info->push_angle;
			break;
		case L_PU_stop:
			info->target_push_speed = 0;
			launcher_push_speed_ctrl(launcher);
			break;
		case L_PU_angle:
			launcher_push_angle_ctrl(launcher);
			if(info->push_speed == 0)
			{
				push_work->cnt++;
				if(push_work->cnt >= config->lock_cnt_max)
				{
					push_work->cnt = 0;
					push_work->status = L_PU_done;
				}
			}
			else 
			{
				push_work->cnt = 0;
			}
			break;
		case L_PU_speed:
			launcher_push_speed_ctrl(launcher);
			if(info->push_speed == 0)
			{
				push_work->cnt ++;
				if(push_work->cnt >= config->lock_cnt_max)
				{
					push_work->cnt = 0;
					push_work->status = L_PU_lock;
				}
			}
			else 
			{
				push_work->cnt = 0;
			}
			break;
		case L_PU_lock:
			info->target_push_speed = 0;
			launcher_push_speed_ctrl(launcher);
			break;
		case L_PU_done:
			info->target_push_speed = 0;
			launcher_push_speed_ctrl(launcher);
			break;
		default:
			break;
	}
}

/**
  * @brief  发射机构dial工作
  * @param  
  * @retval finish
  */
void launcher_dial_work(launcher_t *launcher)
{
	dial_work_info_t *dial_work = launcher->dial_work;
	launcher_config_t *config = launcher->config;
	launcher_info_t *info = launcher->info;
	
	switch(dial_work->status)
	{
		case L_D_relax:
			info->target_dial_angle = info->dial_angle;
			break;
		case L_D_angle:
			launcher_dial_angle_ctrl(launcher);
			if(info->dial_speed == 0)
			{
				dial_work->cnt++;
				if(dial_work->cnt >= config->lock_cnt_max)
				{
					dial_work->cnt = 0;
					dial_work->status = L_D_relax;
				}
			}
			else 
			{
				dial_work->cnt = 0;
			}
			break;
		case L_D_stop:
			info->target_dial_angle = info->dial_angle;
			info->target_dial_speed = 0;
			launcher_dial_speed_ctrl(launcher);
			break;
		default:
			break;
	}
}

/**
  * @brief  发射机构servo工作
  * @param  
  * @retval 
  */
void launcher_servo_work(launcher_t *launcher)
{
	servo_work_info_t *servo_work = launcher->servo_work;
	launcher_config_t *config = launcher->config;
	
	switch(servo_work->status)
	{
		case L_S_work:
			servo_work->cnt++;
			if(servo_work->cnt >= config->servo_work_cnt_max)
			{
				servo_work->cnt = 0;
				servo_work->status = L_S_done;
			}
			break;
		case L_S_done:
			servo_work->cnt = 0;
			break;
	}
}

/**
  * @brief  发射机构光电门检测
  * @param  
  * @retval 
  */
uint8_t launcher_light_check(launcher_t *launcher, uint16_t time)
{
	launcher_config_t *config = launcher->config;
	light_work_info_t *work = launcher->light_work;
	uint8_t light_read_result = launcher->info->light_read_result;
	
	if(time == 0)
	{
		work->cnt = 0;
		work->status = 0;
	}
	
	switch(work->status)
	{
		case 0:
			if(light_read_result == NONE)  //初始化成功
			{
				work->cnt = 0;
				work->status = 1;
			}
			else if(light_read_result == HAVE)
			{
				work->cnt++;
				if(work->cnt >= config->launcher_light_init_time_max)
				{
					return L_LI_always_on;  //光电门一直处于检测到状态
				}
			}
			break;
		case 1:
			if(light_read_result == HAVE)  //检测到弹丸
			{
				work->cnt = 0;
				work->status = 2;
			}
			else if(light_read_result == NONE)
			{
				work->cnt++;
				if(work->cnt >= config->launcher_light_ball_fall_time_max)
				{
					return L_LI_noball;  //检测不到弹丸
				}
			}
			break;
		case 2:
			if(light_read_result == NONE)
			{
				work->cnt = 0;
				work->status = 3;
			}
			else if(light_read_result == HAVE) 
			{
				work->cnt++;
				if(work->cnt >= config->launcher_light_ball_stable_time_max)
				{
					return L_LI_done;  //弹丸稳定
				}
			}
			break;
		case 3:
			if(light_read_result == NONE)  
			{
				work->cnt++;
				if(work->cnt >= config->launcher_light_ball_jump_time_max)
				{
					return L_LI_outball;  //弹丸掉出
				}
			}
			else if(light_read_result == HAVE)
			{
				work->cnt = 0;
				work->status = 2;
			}
			break;
	}
	
	return L_LI_checking;
}

/**
  * @brief  发射机构pitch轴角度审查
  * @param  
  * @retval finish
  */
void launcher_pitch_angle_check(launcher_t *launcher)
{
	launcher_info_t *info = launcher->info;
	launcher_config_t *config = launcher->config;
	
  if(launcher->pitch_work->init_Y_O_N == true)
  {
    if(info->target_pitch_angle > config->pitch_distance_permit_max)
    {
      info->target_pitch_angle = config->pitch_distance_permit_max;
    }
//    else if(info->target_pitch_angle < config->pitch_distance_permit_min)
//    {
//      info->target_pitch_angle = config->pitch_distance_permit_min;
//    } 
  }
}

/**
  * @brief  发射机构yaw轴角度审查
  * @param  
  * @retval finish
  */
void launcher_yaw_angle_check(launcher_t *launcher)
{
	launcher_info_t *info = launcher->info;
	launcher_config_t *config = launcher->config;
	
  if(launcher->yaw_work->init_Y_O_N == true)
  {
    if(info->target_yaw_angle > config->yaw_distance_permit_max)
    {
      info->target_yaw_angle = config->yaw_distance_permit_max;
    }
    else if(info->target_yaw_angle < config->yaw_distance_permit_min)
    {
      info->target_yaw_angle = config->yaw_distance_permit_min;
    }
  }
}

/**
  * @brief  发射机构pitch轴can更新
  * @param  
  * @retval finish
  */
void launcher_pitch_can_update(launcher_t *launcher)
{
	launcher_info_t *info = launcher->info;
	
	info->pitch_angle -= launcher->pitch->base_info->angle_add;
	info->pitch_speed = - launcher->pitch->base_info->speed;
}

/**
  * @brief  发射机构yaw轴can更新
  * @param  
  * @retval finish
  */
void launcher_yaw_can_update(launcher_t *launcher)
{
	launcher_info_t *info = launcher->info;
	
	info->yaw_angle -= launcher->yaw->base_info->angle_add;
	info->yaw_speed = - launcher->yaw->base_info->speed;
}

/**
  * @brief  发射机构push轴can更新
  * @param  
  * @retval finish
  */
void launcher_push_can_update(launcher_t *launcher)
{
	launcher_info_t *info = launcher->info;
	
	info->push_angle -= launcher->push->base_info->angle_add;
	info->push_speed = - launcher->push->base_info->speed;
}

/**
  * @brief  发射机构dial轴can更新
  * @param  
  * @retval finish
  */
void launcher_dial_can_update(launcher_t *launcher)
{
	launcher_info_t *info = launcher->info;
	
	info->dial_angle -= launcher->dial->base_info->angle_add;
	info->dial_speed = - launcher->dial->base_info->speed;
}

/**
  * @brief  发射机构pitch轴角度环
  * @param  
  * @retval finish
  */
void launcher_pitch_angle_ctrl(launcher_t *launcher)
{
	pid_info_t *info = launcher->pitch->pid_angle->info;
	
	info->target = launcher->info->target_pitch_angle;
	info->measure = launcher->info->pitch_angle;
	single_pid_cal(info);
	
	launcher->info->target_pitch_speed = info->out;
	launcher_pitch_speed_ctrl(launcher);
}

/**
  * @brief  发射机构yaw轴角度环
  * @param  
  * @retval finish
  */
void launcher_yaw_angle_ctrl(launcher_t *launcher)
{
	pid_info_t *info = launcher->yaw->pid_angle->info;
	
	info->target = launcher->info->target_yaw_angle;
	info->measure = launcher->info->yaw_angle;
	single_pid_cal(info);
	
	launcher->info->target_yaw_speed = info->out;
	launcher_yaw_speed_ctrl(launcher);
}

/**
  * @brief  发射机构push轴角度环
  * @param  
  * @retval finish
  */
void launcher_push_angle_ctrl(launcher_t *launcher)
{
	pid_info_t *info = launcher->push->pid_angle->info;
	
	info->target = launcher->info->target_push_angle;
	info->measure = launcher->info->push_angle;
	single_pid_cal(info);
	
	launcher->info->target_push_speed = info->out;
	launcher_push_speed_ctrl(launcher);
}

/**
  * @brief  发射机构dial轴角度环
  * @param  
  * @retval finish
  */
void launcher_dial_angle_ctrl(launcher_t *launcher)
{
	pid_info_t *info = launcher->dial->pid_angle->info;
	
	info->target = launcher->info->target_dial_angle;
	info->measure = launcher->info->dial_angle;
	single_pid_cal(info);
	
	launcher->info->target_dial_speed = info->out;
	launcher_dial_speed_ctrl(launcher);
}

/**
  * @brief  发射机构pitch轴速度环
  * @param  
  * @retval finish
  */
void launcher_pitch_speed_ctrl(launcher_t *launcher)
{
	pid_info_t *info = launcher->pitch->pid_speed->info;
	motor_3508_t *motor = launcher->pitch;
	
	int16_t output;
	
	info->target = launcher->info->target_pitch_speed;
	info->measure = launcher->info->pitch_speed;
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

/**
  * @brief  发射机构yaw轴速度环
  * @param  
  * @retval finish
  */
void launcher_yaw_speed_ctrl(launcher_t *launcher)
{
	pid_info_t *info = launcher->yaw->pid_speed->info;
	motor_2006_t *motor = launcher->yaw;
	
	int16_t output;
	
	info->target = launcher->info->target_yaw_speed;
	info->measure = launcher->info->yaw_speed;
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

/**
  * @brief  发射机构push轴速度环
  * @param  
  * @retval finish
  */
void launcher_push_speed_ctrl(launcher_t *launcher)
{
	pid_info_t *info = launcher->push->pid_speed->info;
	motor_3508_t *motor = launcher->push;
	
	int16_t output;
	
	info->target = launcher->info->target_push_speed;
	info->measure = launcher->info->push_speed;
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

/**
  * @brief  发射机构dial速度环
  * @param  
  * @retval finish
  */
void launcher_dial_speed_ctrl(launcher_t *launcher)
{
	pid_info_t *info = launcher->dial->pid_speed->info;
	motor_3508_t *motor = launcher->dial;
	
	int16_t output;
	
	info->target = launcher->info->target_dial_speed;
	info->measure = launcher->info->dial_speed;
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

///**
//  * @brief  发射机构滴答任务
//  * @param  
//  * @retval 
//  */
//void launcher_tick_task(launcher_t *launcher)
//{
//	motor_3508_info_t *info = launcher->friction_left->info;
//	motor_3508_config_t *config = launcher->friction_left->config;
//	
//	info->offline_cnt++;
//	if(info->offline_cnt >= config->offline_cnt_max)
//	{
//		info->status = DEV_OFFLINE;
//	}
//}
