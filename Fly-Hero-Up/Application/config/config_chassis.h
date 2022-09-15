/**
  ******************************************************************************
  * @file           : config_chassis.c\h
  * @brief          : 
  * @note           : 
  ******************************************************************************
  */

#ifndef __CONFIG_CHASSIS_H
#define __CONFIG_CHASSIS_H

#define chassis_cycle_T 1000    //底盘小陀螺周期
#define chassis_back_time 2000  //陀螺仪模式->机械模式的底盘最大归位时间

#define chassis_cycle_speed_mode_init 0 //小陀螺模式下转速模式 //0匀速转动 //1匀加速转动 //2三角函数式
#define chassis_target_speed_mode_init 0 //底盘target速度变化模式 //0根据按键计数梯形变化 //1单值过微分跟踪器

#define chassis_cycle_mode_0_speed_init 3350 //底盘小陀螺模式0速度
#define chassis_cycle_mode_1_speed_init 2050 //底盘小陀螺模式1速度
#define chassis_cycle_mode_2_speed_init 2350 //底盘小陀螺模式2速度

#define chassis_output_max_init 50000// 40000 //底盘最大输出
#define chassis_speed_max_init 8000 //底盘最高速度

#define straight_rectify_coefficient_init 0.1  //直线校正系数（校正的输出与当前输出的比值）（范围0~1）

#endif
