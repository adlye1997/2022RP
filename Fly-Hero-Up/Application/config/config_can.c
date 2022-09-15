/**
  ******************************************************************************
  * @file           : config_can.c\h
  * @brief          : 
  * @note           : 2022-1-18 13:03:50
  ******************************************************************************
  */

#include "config_can.h"
#include "config.h"
#include "string.h"
#include "chassis.h"
#include "gimbal.h"
#include "launcher.h"
#include "judge.h"

void CAN1_Get_Data(uint32_t id, uint8_t *data)
{//处理CAN1接收的数据
	switch (id)
	{
		case chassis_motor_LF_id://底盘左前轮
			chassis.motor[LF].update(&chassis.motor[LF],data);
			break;
		case chassis_motor_RF_id://底盘右前轮
			chassis.motor[RF].update(&chassis.motor[RF],data);
			break;
		case chassis_motor_LB_id://底盘左后轮
			chassis.motor[LB].update(&chassis.motor[LB],data);
			break;
		case chassis_motor_RB_id://底盘右后轮
			chassis.motor[RB].update(&chassis.motor[RB],data);
			break;
		case gimbal_motor_yaw_id://云台yaw轴电机
			gimbal.yaw->update(gimbal.yaw,data);
			gimbal_yaw_can_update(&gimbal);
			break;
		case dial_motor_id://拨盘电机
			launcher.dial->update(launcher.dial,data);
			break;
		case game_robot_status:
			memcpy(&judge.base_info->shooter_cooling_limit, data, 2);
			memcpy(&judge.base_info->car_color, &data[2], 1);
			memcpy(&judge.base_info->chassis_power_limit, &data[3], 2);
			memcpy(&judge.base_info->shooter_id1_42mm_speed_limit, &data[5], 2);
			judge.info->offline_cnt = 0;
			judge.info->status = DEV_ONLINE;
			break;
		case power_heat_data://功率缓冲、热量
			memcpy(&judge.base_info->chassis_power_buffer,data,2);
			memcpy(&judge.base_info->shooter_cooling_heat,&data[2],2);
			memcpy(&judge.base_info->rfid, &data[4], 1);
			judge.info->offline_cnt = 0;
			judge.info->status = DEV_ONLINE;
			break;
//		case shoot_data://射速
//			memcpy(&judge.base_info->bullet_speed,data,4);
//			judge.info->offline_cnt = 0;
//			judge.info->status = DEV_ONLINE;
//			break;
//		case game_robot_pos://yaw轴角度
//			memcpy(&judge.base_info->gimbal_yaw_angle,data,4);
//			judge.info->offline_cnt = 0;
//			judge.info->status = DEV_ONLINE;
//			break;
		case robot_hurt:
			judge.base_info->hurt_type = data[0];
			switch(judge.base_info->hurt_type)
			{
				case 0x02:
					launcher.config->friction_motor_work_speed -= 100;
					if(judge.base_info->shooter_id1_42mm_speed_limit == 10)
					{
						launcher.config->friction_10ms_work_speed -= 100;
					}
					else if(judge.base_info->shooter_id1_42mm_speed_limit == 16)
					{
						launcher.config->friction_16ms_work_speed -= 100;
					}
					break;
				case 0x04:
					chassis.config->chassis_output_max -= 2000;
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}
}

void CAN2_Get_Data(uint32_t id, uint8_t *data)
{//处理CAN2接收的数据
	switch (id)
	{
		case gimbal_motor_pitch_id:
			gimbal.pitch->update(gimbal.pitch,data);
			gimbal_pitch_can_update(&gimbal);
			break;
		case friction_left_motor_id:
			launcher.friction_left->update(launcher.friction_left,data);
			break;
		case friction_right_motor_id:
			launcher.friction_right->update(launcher.friction_right,data);
			break;
		case position_motor_id:
			launcher.position->update(launcher.position,data);
			break;
		default:
			break;
	}
}
