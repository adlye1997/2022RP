/**
  ******************************************************************************
  * @file           : vision.c\h
  * @brief          : 
  * @update         : 
  ******************************************************************************
  */
  
#ifndef __VISION_H
#define __VISION_H

#include "stm32f4xx_hal.h"
#include "vision_potocol.h"
#include "timer.h"

typedef struct
{
	//视觉接收数据
	float yaw_err;  //当前时刻yaw轴角度差
	float yaw_err_last;  //上一时刻yaw轴角度差
	float pitch_err;  //pitch轴角度差
	float distance;  //距离
	uint8_t find_Y_O_N;  //是否识别到标志
	uint8_t target_car_num;  //目标车数字
	
	//视觉接收数据的信息
	uint16_t find_times;  //视觉识别到装甲板次数
	uint16_t lost_times;  //视觉没识别到装甲板次数
	uint16_t real_find_times;  //真实识别到装甲板次数
	uint16_t real_lost_times;  //真实没识别到装甲板次数
	float target_yaw_angle;  //目标yaw轴角度
	float target_yaw_angle_last;  //上一帧目标yaw轴角度
	float camera_yaw_speed;  //相对相机yaw轴速度
	float target_yaw_speed;  //目标yaw轴速度
	float target_pitch_angle;  //目标pitch轴角度
	float target_yaw_angle_begin;  //第一帧目标yaw轴角度
	float target_yaw_angle_end;  //最后一帧目标yaw轴角度
	float target_yaw_angle_total;  //第一帧与最后一帧目标yaw轴角度差
}vision_base_info_t;

typedef struct
{
	uint16_t offline_cnt_max;
	uint16_t shoot_delay_cnt_max;  //射击延时计数上限
	
	uint16_t find_times_max;  //识别到装甲板次数上限
	uint16_t lost_times_max;  //没识别到装甲板次数上限
	uint16_t find_times_eli;  //识别到前几帧剔除
	uint16_t real_find_times_max;  //真实识别到装甲板次数上限
	uint16_t real_lost_times_max;  //真实没识别到装甲板次数上限
	float target_yaw_angle_dif_max;  //目标角度差值最大值
	uint16_t offline_error_cnt_max;  //错误数据失联计数阈值
	uint16_t target_yaw_speed_max;  //目标yaw轴角速度上限
	float yaw_shoot_bias;  //yaw轴补偿
	float pitch_shoot_bias;  //pitch轴补偿
	float predict_con;  //自瞄预测系数
	float predict_lp_con;  //自瞄预测量低通滤波系数
	float spin_target_yaw_err;  //反陀螺识别角度差
	float predict_con_F;  //反陀螺预测系数
	float predict_con_F_bias;  //反陀螺预测系数偏置
	
	uint8_t shoot_permit;  //射击允许标志
}vision_config_t;

typedef struct
{
	uint16_t offline_cnt;  //失联计数
	uint16_t offline_cnt_last;  //上一次失联计数
	uint8_t status;  //状态
	uint8_t interrupt;  //中断标志
	
	uint16_t offline_error_cnt;  //失联错误数据计数
	
	float yaw_predict;  //yaw轴自瞄预测量
	float yaw_predict_K;  //yaw轴自瞄预测量（低通滤波）
	uint8_t spin_Y_O_N;  //反陀螺标志
	
	float target_yaw_angle_ave;  //目标yaw轴角度均值
	float target_yaw_angle_ave_last;  //上一次目标yaw轴角度均值
	float target_yaw_speed_ave;  //目标yaw轴速度均值
	float target_pitch_angle_F;  //反陀螺时目标pitch轴角度
	float target_pitch_angle_F_last;  //上一个反陀螺时目标pitch轴角度
	float target_pitch_angle_F_yaw_err;  //反陀螺时目标pitch轴角度取值时的yaw_err
	float target_distance_F;  //反陀螺时目标距离
	float target_distance_F_last;  //反陀螺时目标距离
	float target_distance_F_yaw_err;  //反陀螺时目标距离取值时的yaw_err
	float yaw_predict_F;  //反陀螺yaw轴预测量
	float yaw_predict_F_shoot;  //反陀螺yaw轴击打预测量
	
	uint16_t shoot_delay_cnt;  //射击延时计数
	uint8_t shoot_Y_O_N;  //是否可以射击标志
}vision_info_t;

typedef struct
{
	float yaw_move;
	float pitch_move;
}vision_output_t;

typedef struct 
{
	vision_base_info_t *base_info;
	vision_config_t *config;
	vision_info_t *info;
	vision_output_t *output;
	vision_tx_info_t *tx_info;
	vision_rx_info_t *rx_info;
}vision_t;

extern vision_t vision;
extern timer_t      vision_task_timer;

void vision_init(vision_t *vision);
void vision_base_info_init(vision_t *vision);
void vision_info_init(vision_t *vision);
void vision_output_init(vision_t *vision);

void vision_ctrl_task(vision_t *vision);

void vision_tick_task(vision_t *vision);

bool vision_send( uint8_t  datau8_1,\
                  float    dataf_1, \
                  float    dataf_2, \
                  uint8_t  datau8_2);
bool vision_recieve(uint8_t  *rxBuf);

#endif
