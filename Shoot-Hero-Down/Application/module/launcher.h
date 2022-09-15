/**
  ******************************************************************************
  * @file           : launcher.c/h
  * @brief          : 
  * @note           : finish 2022-3-3 16:25:29
  ******************************************************************************
  */

#include "stm32f4xx_hal.h"
#include "3508_motor.h"
#include "2006_motor.h"

#ifndef SERVO_PWM
#define SERVO_PWM	TIM1->CCR2
#endif

/* pitch轴状态 */
typedef enum
{
	L_PI_relax,  //卸力
	L_PI_stop,  //停止
	L_PI_init,  //初始化
	L_PI_normal,  //正常
}pitch_work_status_e;

/* yaw轴状态 */
typedef enum
{
	L_Y_relax,  //卸力
	L_Y_stop,  //停止
	L_Y_init,  //初始化
	L_Y_normal,  //正常
}yaw_work_status_e;

/* push状态  */
typedef enum
{
	L_PU_relax,  //卸力
	L_PU_stop,  //停止
	L_PU_angle,  //角度环
	L_PU_speed,  //速度环
	L_PU_lock,  //速度环堵转
	L_PU_done,  //角度环到位或堵转
}push_work_status_e;

/* 拨盘状态 */
typedef enum 
{
	L_D_relax,  //卸力
	L_D_stop,  //停止
	L_D_angle,  //角度环
}dial_work_status_e;

/* 舵机状态 */
typedef enum 
{
	L_S_work,  //工作
	L_S_done,  //完成
}servo_work_status_e;

/* 光电门状态 */
typedef enum
{
	L_LI_checking,  //检测中
	L_LI_always_on,  //光电门持续检测到
	L_LI_noball,  //检测不到弹丸
	L_LI_outball,  //弹丸掉出
	L_LI_done,  //完成
}light_work_status_e;

/* 读光电门结果 */
typedef enum
{
	HAVE,  //有物体
	NONE,  //无物体
}light_read_result_e;

typedef enum 
{
	L_L_offline,  //失联
	L_L_stop,     //停止
	L_L_init,     //初始化
	L_L_done,     //指令执行完毕
	L_L_ready,    //准备射击
	L_L_shoot,    //射击
	L_L_return,   //恢复
	L_L_relax,    //等待指令7
	L_L_none,     //无指令
	L_L_auto,     //自动射击
	L_L_out_auto, //退出自动射击
}launcher_work_status_e;

typedef struct
{
	uint16_t lock_cnt_max;
	uint16_t init_cnt_max;
	uint16_t servo_work_cnt_max;
	
	uint32_t pitch_distance;
	int32_t pitch_distance_permit_max;
	int32_t pitch_distance_permit_min;
	int32_t pitch_init_speed;  //pitch轴初始化速度
	int16_t launcher_pitch_init_output_max;
	int16_t launcher_pitch_normal_output_max;
	int16_t launcher_pitch_move_angle;
	
	uint32_t yaw_half_distance;
	int32_t yaw_distance_permit_max;
	int32_t yaw_distance_permit_min;
	int32_t yaw_init_speed;  //yaw轴初始化速度
	int16_t launcher_yaw_init_output_max;
	int16_t launcher_yaw_normal_output_max;
	int16_t launcher_yaw_move_angle;
	
	uint32_t push_return_distance;
  uint32_t mode_return_push_return_distance_dif;
	int32_t push_ready_speed;  //负数

	int16_t servo_open_num;
	int16_t servo_close_num;
	
	int16_t dial_reload_angle;  //正数
	int16_t dial_unload_angle;  //正数
	
	uint16_t launcher_light_check_time_max;  //光电检测最长时间
	uint16_t launcher_light_init_time_max;  //光电门初始化为无的最长时间
	uint16_t launcher_light_ball_fall_time_max;  //光电门检测弹丸是否落下的最长时间
	uint16_t launcher_light_ball_stable_time_max;  //光电门检测弹丸稳定的最长时间
	uint16_t launcher_light_ball_jump_time_max;  //光电门检测跳弹最长时间
	uint16_t auto_delay_cnt_max;  //自动射击延时最长时间
}launcher_config_t;

typedef struct 
{
	int32_t pitch_angle;  //上正下负
	int32_t target_pitch_angle;
	int32_t pitch_speed;
	int32_t target_pitch_speed;
	int32_t yaw_angle;  //左负右正
	int32_t target_yaw_angle;
	int32_t yaw_speed;
	int32_t target_yaw_speed;
	int32_t push_angle;  //上正下负
	int32_t target_push_angle;
	int32_t push_speed;
	int32_t target_push_speed;
	int32_t dial_angle;  //上正下负
	int32_t target_dial_angle;
	int32_t dial_speed;
	int32_t target_dial_speed;
	uint8_t light_read_result;
	uint16_t auto_delay_cnt;  //自动射击延时计数
	uint8_t auto_work_status;  //退出自动射击时自动射击工作模式
}launcher_info_t;

typedef struct 
{
	uint8_t command;
	uint8_t status;
	uint8_t init_Y_O_N;
	uint8_t work_status;
	uint8_t ready_shoot_Y_O_N;
}launcher_work_info_t;

typedef struct
{
	uint8_t command;
	uint8_t status;
	uint8_t init_Y_O_N;
	int16_t init_cnt;
}pitch_work_info_t;

typedef struct
{
	uint8_t command;
	uint8_t status;
	uint8_t init_Y_O_N;
	int16_t init_cnt;
}yaw_work_info_t;

typedef struct
{
	uint8_t status;
	int16_t cnt;
}push_work_info_t;

typedef struct 
{
	uint8_t status;
	uint16_t cnt;
}dial_work_info_t;

typedef struct
{
	uint8_t status;
	uint16_t cnt;
}servo_work_info_t;

typedef struct
{
	uint8_t status;
	uint16_t cnt;
}light_work_info_t;

typedef struct 
{
	motor_2006_t  *yaw;
	motor_3508_t  *pitch; //从下往上0~-pitch_distance_machine
	motor_3508_t  *push;
	motor_3508_t  *dial;
	
	launcher_config_t *config;
	launcher_info_t *info;
	launcher_work_info_t *launcher_work;
	pitch_work_info_t *pitch_work;
	yaw_work_info_t *yaw_work;
	push_work_info_t *push_work;
	dial_work_info_t *dial_work;
	servo_work_info_t *servo_work;
	light_work_info_t *light_work;
}launcher_t;

extern launcher_t launcher;

void launcher_init(launcher_t *launcher);
void launcher_info_init(launcher_info_t *info);
void launcher_commond_init(launcher_t *launcher);
void launcher_work_info_init(launcher_work_info_t *info);
void launcher_pitch_work_init(pitch_work_info_t *info);
void launcher_yaw_work_init(yaw_work_info_t *info);
void launcher_push_work_init(push_work_info_t *info);
void launcher_dial_work_init(dial_work_info_t *info);
void launcher_servo_work_init(servo_work_info_t *info);

void launcher_ctrl_task(launcher_t *launcher);
void launcher_mode_update(launcher_t *launcher);

void launcher_commond_respond(launcher_t *launcher);
void launcher_work(launcher_t *launcher);
void launcher_pitch_work(launcher_t *launcher);
void launcher_yaw_work(launcher_t *launcher);
void launcher_push_work(launcher_t *launcher);
void launcher_dial_work(launcher_t *launcher);
void launcher_servo_work(launcher_t *launcher);
uint8_t launcher_light_check(launcher_t *launcher, uint16_t time);

void launcher_pitch_angle_check(launcher_t *launcher);
void launcher_yaw_angle_check(launcher_t *launcher);

void launcher_pitch_can_update(launcher_t *launcher);
void launcher_yaw_can_update(launcher_t *launcher);
void launcher_push_can_update(launcher_t *launcher);
void launcher_dial_can_update(launcher_t *launcher);

void launcher_pitch_speed_ctrl(launcher_t *launcher);
void launcher_pitch_angle_ctrl(launcher_t *launcher);
void launcher_yaw_speed_ctrl(launcher_t *launcher);
void launcher_yaw_angle_ctrl(launcher_t *launcher);
void launcher_push_speed_ctrl(launcher_t *launcher);
void launcher_push_angle_ctrl(launcher_t *launcher);
void launcher_dial_speed_ctrl(launcher_t *launcher);
void launcher_dial_angle_ctrl(launcher_t *launcher);
