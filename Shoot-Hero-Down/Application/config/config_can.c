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
#include "cap_protocol.h"

void CAN1_Get_Data(uint32_t id, uint8_t *data)
{//处理CAN1接收的数据
	switch (id)
	{
		case chassis_LF_can_id://底盘左前轮
			chassis.motor_LF->update(chassis.motor_LF,data);
			break;
		case chassis_RF_can_id://底盘右前轮
			chassis.motor_RF->update(chassis.motor_RF,data);
			break;
		case chassis_LB_can_id://底盘左后轮
			chassis.motor_LB->update(chassis.motor_LB,data);
			break;
		case chassis_RB_can_id://底盘右后轮
			chassis.motor_RB->update(chassis.motor_RB,data);
			break;
		case s_yaw_can_id:
			gimbal.yaw->update(gimbal.yaw, data);
			gimbal_yaw_can_update(&gimbal);
			break;
    case 0x030:
      cap_update(data);
      break;
		default:
			break;
	}
}

void CAN2_Get_Data(uint32_t id, uint8_t *data)
{//处理CAN2接收的数据
	switch (id)
	{
		case dial_can_id:
			launcher.dial->update(launcher.dial,data);
			launcher_dial_can_update(&launcher);
			break;
		case pitch_can_id:
			launcher.pitch->update(launcher.pitch,data);
			launcher_pitch_can_update(&launcher);
			break;
		case yaw_can_id:
			launcher.yaw->update(launcher.yaw,data);
			launcher_yaw_can_update(&launcher);
			break;
		case push_can_id:
			launcher.push->update(launcher.push,data);
			launcher_push_can_update(&launcher);
			break;
		case s_pitch_can_id:
			gimbal.pitch->update(gimbal.pitch, data);
			gimbal_pitch_can_update(&gimbal);
			break;
		case watch_can_id:
			gimbal.watch->update(gimbal.watch, data);
			gimbal_watch_can_update(&gimbal);
			break;
		default:
			break;
	}
}
