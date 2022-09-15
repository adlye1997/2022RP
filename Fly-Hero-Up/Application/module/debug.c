/**
  ******************************************************************************
  * @file           : debug.c\h
  * @brief          : 
  * @note           : 
  ******************************************************************************
  */
	
#include "debug.h"
#include "pid_feedback_system.h"
#include "gimbal.h"
#include "remote.h"
#include "tracking_differentiator.h"
#include "pid_feedback_system.h"
#include "math.h"
#include "math_support.h"
#include "usmart.h"
#include "chassis.h"
#include "judge.h"
#include "config_chassis.h"
#include "string.h"
#include "vision.h"

pitch_debug_t pitch_debug;
yaw_debug_t yaw_debug;

/**
	* @brief  ����С����debug
  * @param  
  * @retval 
  */
void chassis_cycle_debug(void)
{
	int16_t *speed;
	
	static uint16_t cnt = 0;
	uint8_t flag;
	int16_t config_speed_change = 100;
	int16_t config_time = 200;
	
	/* �������� */
	if(cnt == 0) 
	{
		/* �ж��Ƿ񳬹��� */
		if(judge.base_info->chassis_power_buffer >= judge.config->buffer_max)
		{
			flag = 1; //δ������
		}
		else 
		{
			flag = 0; //������
		}
		/* �򻯺������� */
		switch(chassis.config->chassis_cycle_speed_mode)
		{
			case 0:
				speed = &chassis.config->chassis_cycle_mode_0_speed;
				break;
			case 1:
				speed = &chassis.config->chassis_cycle_mode_1_speed;
				break;
			case 2:
				speed = &chassis.config->chassis_cycle_mode_2_speed;
				break;
		}
		/* ������Ϣ */
//		uint8_t txBuf[40] = "cycle_mode= ,speed=    ,buffer=  ";
//		num_to_str_2(chassis.config->chassis_cycle_speed_mode, &txBuf[11], 1);
//		num_to_str_2(abs(*speed), &txBuf[19], 4);
//		num_to_str_2(abs(judge.base_info->chassis_power_buffer), &txBuf[31], 2);
//		usmart_send_data(txBuf, 33);
		/* ����Ӧ */
		if(flag == 0)
		{
			(*speed) -= config_speed_change;
			if((*speed) < 3000)
			{
				(*speed) = 3000;
			}
		}
		else if(flag == 1)
		{
			(*speed) += config_speed_change;
			if((*speed) > 8000)
			{
				(*speed) = 8000;
			}
		}
	}
	cnt++;
	cnt	%= config_time;
}

/**
	* @brief  ���̵�ǰ�ٶȷ���
  * @param  
  * @retval 
  */
void chassis_measure_speed_send(void)
{
	static uint16_t cnt = 0;
	if(cnt == 0)
	{
		uint8_t txBuf[40] = "front=    ,right=    ,cycle=    ";
		num_to_str_2(abs(chassis.info->measure_front_speed), &txBuf[6], 4);
		num_to_str_2(abs(chassis.info->measure_right_speed), &txBuf[17], 4);
		num_to_str_2(abs(chassis.info->measure_cycle_speed), &txBuf[28], 4);
//		-(txBuf, 32);
	}
	cnt++;
	cnt %= 2000;
}

void debug_vision_receive(void)
{
//	uint8_t txBuf[60] = "time:   ,yaw=    ,pitch=    ,yawerr=    ,pitcherr=    \n";
//	num_to_str_2(abs(vision_task_timer.info->duration_ms), &txBuf[5], 3);
//	num_to_str_3((int16_t)gimbal.info->yaw_imu_angle, &txBuf[13], 4);
//	num_to_str_3((int16_t)gimbal.info->pitch_imu_angle, &txBuf[24], 4);
//	num_to_str_3((int16_t)vision.yaw_err, &txBuf[36], 4);
//	num_to_str_3((int16_t)vision.pitch_err, &txBuf[50], 4);	
//	usmart_send_data(txBuf,56);
}

extern vision_tx_info_t vision_tx_info;
void debug_vision_send(void)
{
	uint8_t txBuf[60];
	memcpy(txBuf ,&vision_tx_info, sizeof(vision_tx_info));
	memcpy(&txBuf[sizeof(vision_tx_info)], "\n", 2);
	usmart_send_data(txBuf, sizeof(vision_tx_info) + 2);
}

///**
//  * @brief  pitch�����ǶȻ�debug����
//  * @param  
//  * @retval 
//  */
//void pitch_motor_angle_pid_debug_task(int16_t up_angle, int16_t down_angle)
//{
//	pid_info_t *pid = gimbal.yaw->motor_pid_angle->info;
//	
//	static uint16_t t_reach = 0;  //��λʱ��
//	static uint16_t t_sum = 0;  //����ʱ��
//	int t;
//	uint8_t t_str[9];
//	uint16_t t_str_len;
//	
//	t_sum++;
//	
//	/* ��ʼ�� */
//	if((pid->target != down_angle)&&(pid->target != up_angle))
//	{
//		gimbal.info->pitch_motor_angle_target = up_angle;
//		t_reach = 0;
//		t_sum = 0;
//	}
//	/* �Ҿ�λ */
//	else if((pid->out == 0)&&(pid->target == up_angle))
//	{
//		t_reach++;
//		if(t_reach >= 500)
//		{
//			t = t_sum - t_reach;
//			if(num_to_str(t, t_str, &t_str_len) == 0)
//			{
//				t_str[t_str_len] = 'm';
//				t_str[t_str_len+1] = 's';
//				usmart_send_data(t_str, t_str_len+2);
//			}
//			gimbal.info->pitch_motor_angle_target = down_angle;
//			t_reach = 0;
//			t_sum = 0;
//		}
//	}
//	/* ���λ */
//	else if((pid->out == 0)&&(pid->target == down_angle))
//	{
//		t_reach++;
//		if(t_reach >= 500)
//		{
//			t = t_sum - t_reach;
//			if(num_to_str(t, t_str, &t_str_len) == 0)
//			{
//				t_str[t_str_len] = 'm';
//				t_str[t_str_len+1] = 's';
//				usmart_send_data(t_str, t_str_len+2);
//			}
//			gimbal.info->pitch_motor_angle_target = up_angle;
//			t_reach = 0;
//			t_sum = 0;
//		}
//	}
//	/* ������ */
//	else 
//	{
//		t_reach = 0;
//	}
//	
//	gimbal_pitch_motor_pid_angle_ctrl(&gimbal);
//}

///**
//  * @brief  pitch�������ǽǶȻ�debug����
//  * @param  
//  * @retval 
//  */
//void pitch_imu_angle_pid_debug_task(int16_t up_angle, int16_t down_angle)
//{
//	pid_info_t *pid = gimbal.pitch->imu_pid_angle->info;
//	
//	static uint16_t t_reach = 0;
//	static uint16_t t_sum = 0;  //����ʱ��
//	int t;
//	uint8_t t_str[9];
//	uint16_t t_str_len;
//	
//	t_sum++;
//	
//	/* ��ʼ�� */
//	if((pid->target != down_angle)&&(pid->target != up_angle))
//	{
//		gimbal.info->pitch_imu_angle_target = up_angle;
//		t_reach = 0;
//		t_sum = 0;
//	}
//	/* �Ҿ�λ */
//	else if((abs(pid->out) <= 1)&&(pid->target == up_angle))
//	{
//		t_reach++;
//		if(t_reach >= 500)
//		{
//			t = t_sum - t_reach;
//			if(num_to_str(t, t_str, &t_str_len) == 0)
//			{
//				t_str[t_str_len] = 'm';
//				t_str[t_str_len+1] = 's';
//				usmart_send_data(t_str, t_str_len+2);
//			}
//			gimbal.info->pitch_imu_angle_target = down_angle;
//			t_reach = 0;
//			t_sum = 0;
//		}
//	}
//	/* ���λ */
//	else if((abs(pid->out) <= 1)&&(pid->target == down_angle))
//	{
//		t_reach++;
//		if(t_reach >= 500)
//		{
//			t = t_sum - t_reach;
//			if(num_to_str(t, t_str, &t_str_len) == 0)
//			{
//				t_str[t_str_len] = 'm';
//				t_str[t_str_len+1] = 's';
//				usmart_send_data(t_str, t_str_len+2);
//			}
//			gimbal.info->pitch_imu_angle_target = up_angle;
//			t_reach = 0;
//			t_sum = 0;
//		}
//	}
//	/* ������ */
//	else 
//	{
//		t_reach = 0;
//	}
//	
//	gimbal_pitch_imu_pid_angle_ctrl(&gimbal);
//}

///**
//  * @brief  yaw�����ǶȻ�ѭ��debug����
//  * @param  
//  * @retval 
//  */
//void yaw_motor_angle_pid_debug_task(int16_t left_angle, int16_t right_angle)
//{
//	pid_info_t *pid = gimbal.yaw->motor_pid_angle->info;
//	
//	static uint16_t t_reach = 0;  //��λʱ��
//	static uint16_t t_sum = 0;  //����ʱ��
//	int t;
//	uint8_t t_str[9];
//	uint16_t t_str_len;
//	
//	t_sum++;
//	
//	/* ��ʼ�� */
//	if((pid->target != left_angle)&&(pid->target != right_angle))
//	{
//		gimbal.info->yaw_motor_angle_target = right_angle;
//		t_reach = 0;
//		t_sum = 0;
//	}
//	/* �Ҿ�λ */
//	else if((pid->out == 0)&&(pid->target == right_angle))
//	{
//		t_reach++;
//		if(t_reach >= 500)
//		{
//			t = t_sum - t_reach;
//			if(num_to_str(t, t_str, &t_str_len) == 0)
//			{
//				t_str[t_str_len] = 'm';
//				t_str[t_str_len+1] = 's';
//				usmart_send_data(t_str, t_str_len+2);
//			}
//			gimbal.info->yaw_motor_angle_target = left_angle;
//			t_reach = 0;
//			t_sum = 0;
//		}
//	}
//	/* ���λ */
//	else if((pid->out == 0)&&(pid->target == left_angle))
//	{
//		t_reach++;
//		if(t_reach >= 500)
//		{
//			t = t_sum - t_reach;
//			if(num_to_str(t, t_str, &t_str_len) == 0)
//			{
//				t_str[t_str_len] = 'm';
//				t_str[t_str_len+1] = 's';
//				usmart_send_data(t_str, t_str_len+2);
//			}
//			gimbal.info->yaw_motor_angle_target = right_angle;
//			t_reach = 0;
//			t_sum = 0;
//		}
//	}
//	/* ������ */
//	else 
//	{
//		t_reach = 0;
//	}
//	
//	gimbal_yaw_motor_pid_angle_ctrl(&gimbal);
//}

///**
//  * @brief  yaw�������ǽǶȻ�ѭ��debug����
//  * @param  
//  * @retval 
//  */
//void yaw_imu_angle_pid_debug_task(int16_t left_angle, int16_t right_angle)
//{
//	pid_info_t *pid = gimbal.yaw->imu_pid_angle->info;
//	
//	static uint16_t t_reach = 0;
//	static uint16_t t_sum = 0;  //����ʱ��
//	int t;
//	uint8_t t_str[9];
//	uint16_t t_str_len;
//	
//	t_sum++;
//	
//	/* ��ʼ�� */
//	if((pid->target != left_angle)&&(pid->target != right_angle))
//	{
//		gimbal.info->yaw_imu_angle_target = right_angle;
//		t_reach = 0;
//		t_sum = 0;
//	}
//	/* �Ҿ�λ */
//	else if((abs(pid->out) <= 1)&&(pid->target == right_angle))
//	{
//		t_reach++;
//		if(t_reach >= 500)
//		{
//			t = t_sum - t_reach;
//			if(num_to_str(t, t_str, &t_str_len) == 0)
//			{
//				t_str[t_str_len] = 'm';
//				t_str[t_str_len+1] = 's';
//				usmart_send_data(t_str, t_str_len+2);
//			}
//			gimbal.info->yaw_imu_angle_target = left_angle;
//			t_reach = 0;
//			t_sum = 0;
//		}
//	}
//	/* ���λ */
//	else if((abs(pid->out) <= 1)&&(pid->target == left_angle))
//	{
//		t_reach++;
//		if(t_reach >= 500)
//		{
//			t = t_sum - t_reach;
//			if(num_to_str(t, t_str, &t_str_len) == 0)
//			{
//				t_str[t_str_len] = 'm';
//				t_str[t_str_len+1] = 's';
//				usmart_send_data(t_str, t_str_len+2);
//			}
//			gimbal.info->yaw_imu_angle_target = right_angle;
//			t_reach = 0;
//			t_sum = 0;
//		}
//	}
//	/* ������ */
//	else 
//	{
//		t_reach = 0;
//	}
//	
//	gimbal_yaw_imu_pid_angle_ctrl(&gimbal);
//}
