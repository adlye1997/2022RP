/**
  ******************************************************************************
  * @file           : gimbal.c/h
  * @brief          : 
  * @note           : 
	*                   2020.3.11 修改gimbal_t里的指针名
  ******************************************************************************
  */

#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "stm32F4xx_hal.h"
#include "6020_motor.h"

/* 云台yaw轴模式枚举 */
typedef enum 
{
	G_Y_offline,  //失联
	G_Y_follow,   //跟随底盘
	G_Y_gyro,     //陀螺仪
	G_Y_machine,  //机械
	G_Y_auto,     //自瞄
	G_Y_shoot,    //吊射
	G_Y_keep,     //保持
}gimbal_yaw_mode_e;

/* 云台pitch轴模式枚举 */
typedef enum 
{
	G_P_offline,  //失联
	G_P_gyro,     //陀螺仪
	G_P_machine,  //机械
	G_P_auto,     //自瞄
	G_P_shoot,    //吊射
	G_P_keep,     //保持
}gimbal_pitch_mode_e;

typedef struct 
{
	uint16_t yaw_angle_change_shoot;    //吊射模式yaw轴角度控制变化量
	uint16_t pitch_angle_change_shoot;  //吊射模式pitch轴角度控制变化量
	uint16_t pitch_motor_angle_middle;  //pitch轴电机中值
	uint16_t yaw_motor_angle_middle;    //yaw轴电机中值
}gimbal_config_t;

typedef struct
{
	float    yaw_imu_angle;             //yaw轴 世界坐标系 角度(-180°~180°)      (顺时针为正)
	int16_t  yaw_motor_angle;           //yaw轴 相对底盘   角度(-4096~4096)      (顺时针为正)
	float    yaw_imu_speed;             //yaw轴 世界坐标系 速度(dps)             (顺时针为正)
	int16_t  yaw_motor_speed;           //yaw轴 相对底盘   速度(rpm)             (顺时针为正)
	float    pitch_imu_angle;           //pitch轴 世界坐标系 角度(-90°~90°)      (向上为正)
	int16_t  pitch_motor_angle;         //pitch轴 相对底盘   角度(-2048~2048)    (向上为正)
	float    pitch_imu_speed;           //pitch轴 世界坐标系 速度(dps)           (向上为正)
	float    pitch_motor_speed;         //pitch轴 相对底盘   速度(rpm)           (向上为正)
	float    judge_yaw_angle;           //裁判系统枪管角度(d)
	float    yaw_imu_angle_target;      //yaw轴 世界坐标系 目标角度(-180°~180°)  (顺时针为正)
	int16_t  yaw_motor_angle_target;    //yaw轴 相对底盘   目标角度(-4096~4096)  (顺时针为正)
	float    pitch_imu_angle_target;    //pitch轴 世界坐标系 目标角度(-90°~90°)  (向上为正)
	float    pitch_motor_angle_target;  //pitch轴 相对底盘   目标角度(-2048~2048)(向上为正)
	uint8_t  yaw_mode;                  //yaw轴模式
	uint8_t  pitch_mode;                //pitch轴模式
}gimbal_info_t;

typedef struct 
{
	motor_6020_t    *yaw;
	motor_6020_t    *pitch;
	gimbal_config_t *config;
	gimbal_info_t   *info;
}gimbal_t;

extern gimbal_t gimbal;

void gimbal_init(gimbal_t *gimbal);
void gimbal_info_init(gimbal_info_t *info);
void gimbal_commond_init(gimbal_t *gimbal);

void gimbal_ctrl_task(gimbal_t *gimbal);
void gimbal_mode_update(gimbal_t *gimbal);
void gimbal_commond_respond(gimbal_t *gimbal);
void gimbal_work(gimbal_t *gimbal);

void gimbal_yaw_can_update(gimbal_t *gimbal);             //云台yaw轴can更新
void gimbal_pitch_can_update(gimbal_t *gimbal);           //云台pitch轴can更新
void gimbal_imu_update(gimbal_t *gimbal);                 //云台陀螺仪更新

void gimbal_yaw_angle_check(gimbal_t *gimbal);            //云台yaw轴角度检查
void gimbal_pitch_angle_check(gimbal_t *gimbal);          //云台pitch轴角度检查

void gimbal_yaw_motor_pid_speed_ctrl(gimbal_t *gimbal);   //云台yaw轴电机速度环
void gimbal_yaw_imu_pid_speed_ctrl(gimbal_t *gimbal);     //云台yaw轴陀螺仪速度环
void gimbal_yaw_motor_pid_angle_ctrl(gimbal_t *gimbal);   //云台yaw轴电机角度环
void gimbal_yaw_imu_pid_angle_ctrl(gimbal_t *gimbal);     //云台yaw轴陀螺仪角度环
void gimbal_pitch_motor_pid_speed_ctrl(gimbal_t *gimbal); //云台pitch轴电机速度环
void gimbal_pitch_imu_pid_speed_ctrl(gimbal_t *gimbal);   //云台pitch轴陀螺仪速度环
void gimbal_pitch_motor_pid_angle_ctrl(gimbal_t *gimbal); //云台pitch轴电机角度环
void gimbal_pitch_imu_pid_angle_ctrl(gimbal_t *gimbal);   //云台pitch轴陀螺仪角度环

#endif
