/**
  ******************************************************************************
  * @file           : vision.c\h
  * @brief          : 
  * @note           : 
  ******************************************************************************
  */

#include "vision.h"
#include "string.h"
#include "config.h"
#include "timer.h"
#include "debug.h"
#include "gimbal.h"
#include "car.h"
#include "ave_filter.h"
#include "vector.h"
#include "math_support.h"
#include "math.h"

vision_config_t vision_config = 
{
	.offline_cnt_max = 50,
	.shoot_delay_cnt_max = 1000,  //�Զ�������ʱ��
	.find_times_max = 1000,
	.lost_times_max = 1000,
	.find_times_eli = 0,
	.real_find_times_max = 250,  //��ʵʶ��װ�װ�������ޣ��˳������ݱ�־��
	.real_lost_times_max = 100,  //��ʵûʶ��װ�װ�������ޣ��˳������ݱ�־��
	.target_yaw_angle_dif_max = 2,  //Ŀ��ǶȲ�ֵ���ֵ���л�װ�װ��־��
	.offline_error_cnt_max = 15,  //��������ʧ��������ֵ
	.target_yaw_speed_max = 100,  //Ŀ��yaw����ٶ�����
	.predict_con = 0.2,  //����Ԥ��ϵ��
	.predict_lp_con = 0.05,  //����Ԥ������ͨ�˲�ϵ��
	.spin_target_yaw_err = 2,  //������ʶ��ǶȲ�
	.predict_con_F_bias = 0.08,
};
vision_base_info_t vision_base_info;
vision_info_t vision_info;
vision_output_t vision_output;
vision_t vision = 
{
	.tx_info = &vision_tx_info,
	.rx_info = &vision_rx_info,
	.base_info = &vision_base_info,
	.config = &vision_config,
	.info = &vision_info,
	.output = &vision_output,
};

timer_info_t vision_timer_info;
timer_t      vision_task_timer = 
{
	.info = &vision_timer_info,
};

extern vector_t vector_yaw_imu_angle;
extern vector_t vector_yaw_imu_speed;

/**
  * @brief  �Ӿ���ʼ��
  * @param  
  * @retval 
  */
void vision_init(vision_t *vision)
{
	vision_base_info_init(vision);  //��ʼ��base_info
	vision_info_init(vision);  //��ʼ��info
	vision_output_init(vision);  //��ʼ��output
}

/**
  * @brief  �Ӿ�������Ϣ��ʼ��
  * @param  
  * @retval 
  */
void vision_base_info_init(vision_t *vision)
{
	memset(vision->base_info, 0, sizeof(vision_base_info_t));
}

/**
  * @brief  �Ӿ���Ϣ��ʼ��
  * @param  
  * @retval 
  */
void vision_info_init(vision_t *vision)
{
	memset(vision->info, 0, sizeof(vision_info_t));
}

/**
  * @brief  �Ӿ������ʼ��
  * @param  
  * @retval 
  */
void vision_output_init(vision_t *vision)
{
	memset(vision->output, 0, sizeof(vision_output_t));
}

/**
  * @brief  �Ӿ���������
  * @param  
  * @retval 
  */
/* Ŀ¼ */
//�����Ӿ�����
//**********����find_times��lost_times
//**********�޳�ǰ��֡
//�����Ӿ���Ϣ
//**********����real_find_times��real_lost_times
//**********����target_yaw_angle������real_find_times
//**********����camera_yaw_speed
//**********�������ݣ���ʱ�䲻�����ж�
//**********����target_yaw_speed
//**********����target_pitch_angle
//void vision_ctrl_task(vision_t *vision)
//{
//	vision_base_info_t *base_info = vision->base_info;
//	vision_config_t *config = vision->config;
//	vision_info_t *info = vision->info;
//	vision_output_t *output = vision->output;
//	double temp;
//	
//	if(info->interrupt == 1)
//	{
//		/* �����Ӿ����� begin */
//		
//			/* ����find_times��lost_times begin */
//			if(base_info->yaw_err != 0)  //ʶ��װ�װ�
//			{
//				base_info->find_times++;
//				
//				if(base_info->find_times >= config->find_times_max)  //���ٵ���װ�װ�
//				{
//					base_info->find_times = config->find_times_max + 1;
//				}

//				base_info->lost_times = 0;
//			}
//			else if(base_info->yaw_err == 0)  //ûʶ��װ�װ�
//			{
//				base_info->lost_times++;
//				
//				if(base_info->lost_times >= config->lost_times_max)  //��ʧװ�װ�
//				{
//					base_info->lost_times = config->lost_times_max + 1;
//				}
//				
//				base_info->find_times = 0;
//			}
//			/* ����find_times��lost_times end */
//			
//			
//			/* �޳�ǰ��֡ begin */
//			if(base_info->find_times <= config->find_times_eli)
//			{
//				base_info->yaw_err = 0;
//				base_info->pitch_err = 0;
//				base_info->distance = 0;
//				base_info->find_Y_O_N = 0;
//			}
//			/* �޳�ǰ��֡ end */
//			
//		/* �����Ӿ����� end */
//			
//		/* �����Ӿ���Ϣ begin */
//			
//			/* ����real_find_times��real_lost_times begin */
//			if(base_info->yaw_err != 0)  //ʶ��װ�װ�
//			{
//				base_info->real_find_times++;
//				
//				if(base_info->real_find_times >= config->real_find_times_max)  //���ٵ���װ�װ�
//				{
//					base_info->real_find_times = config->real_find_times_max + 1;
//				}
//				
//				base_info->real_lost_times = 0;
//			}
//			else if(base_info->yaw_err == 0)  //ûʶ��װ�װ�
//			{
//				base_info->real_lost_times++;
//				
//				if(base_info->real_lost_times >= config->real_lost_times_max)  //��ʧװ�װ�
//				{
//					base_info->real_lost_times = config->real_lost_times_max + 1;
//					base_info->real_find_times = 0;
//				}
//			}
//			/* ����real_find_times��real_lost_times end */
//			
//			/* ����target_yaw_angle������real_find_times begin */
//			if(base_info->yaw_err != 0)  //ʶ��װ�װ�
//			{
//				base_info->target_yaw_angle_last = base_info->target_yaw_angle;
//				base_info->target_yaw_angle = base_info->yaw_err + vector_get(&vector_yaw_imu_angle);//gimbal.info->yaw_imu_angle;
//				if( (abs(base_info->target_yaw_angle - base_info->target_yaw_angle_last) >= config->target_yaw_angle_dif_max)
//				&& (base_info->target_yaw_angle_last != 0) )
//				{
//					base_info->real_find_times = 1;
//				}
//				if(base_info->real_find_times == 1)
//				{
//					if(base_info->target_yaw_angle_last != 0)
//					{
//						base_info->target_yaw_angle_end = base_info->target_yaw_angle_last;
//						base_info->target_yaw_angle_total = base_info->target_yaw_angle_end - base_info->target_yaw_angle_begin;
//					}
//					base_info->target_yaw_angle_begin = base_info->target_yaw_angle;
//				}
//			}
//			/* ����target_yaw_angle������real_find_times end */
//			
//			/* ����camera_yaw_speed begin */
//			if((base_info->yaw_err_last == 0)||(base_info->yaw_err == 0))  //��һ֡�����һ֡��ʶ�𲻵�
//			{
//				base_info->camera_yaw_speed = 0;
//			}
//			else 
//			{
//				base_info->camera_yaw_speed = (base_info->yaw_err - base_info->yaw_err_last) * 333;
//			}
//			/* ����camera_yaw_speed end */
//			
//			/* �������ݣ���ʱ�䲻�����ж� begin */
//			if(info->offline_error_cnt > 0)
//			{
//				info->offline_error_cnt--;
//			}
//			if(info->offline_cnt_last >= config->offline_error_cnt_max)
//			{
//				info->offline_error_cnt += 2;
//			}
//			/* �������ݣ���ʱ�䲻�����ж� end */
//			
//			/* ����target_yaw_speed begin */
//			if(info->offline_error_cnt != 0)  //�޳��������ݣ���ʱ�䲻����
//			{
//				base_info->target_yaw_speed = base_info->target_yaw_speed;
//			}
//			else if(base_info->camera_yaw_speed != 0)
//			{
//				float target_yaw_speed_temp = base_info->camera_yaw_speed + vector_get(&vector_yaw_imu_speed);//gimbal.info->yaw_imu_speed;
//				if(abs(target_yaw_speed_temp) >= abs(config->target_yaw_speed_max))  //Ŀ������ٶ��޷�
//				{
//					base_info->target_yaw_speed = base_info->target_yaw_speed;
//				}
//				else 
//				{
//					base_info->target_yaw_speed = target_yaw_speed_temp;
//				}
//			}
//			else if(base_info->camera_yaw_speed == 0)
//			{
//				if(base_info->real_find_times > 1)
//				{
//					base_info->target_yaw_speed = base_info->target_yaw_speed;
//				}
//				else if(base_info->real_find_times <= 1)
//				{
//					base_info->target_yaw_speed = 0;
//				}
//			}
//			/* ����target_yaw_speed end */
//			
//			/* ����target_pitch_angle begin */
//			if(base_info->pitch_err != 0)
//			{
//				base_info->target_pitch_angle = base_info->pitch_err + gimbal.info->pitch_imu_angle;
//			}
//			/* ����target_pitch_angle end */
//			
//		/* �����Ӿ���Ϣ end */
//		
//		if(base_info->real_lost_times == 0)  //ʶ��װ�װ�
//		{
//			/* ���� begin */
//			
//				/* ̧ͷ���� begin */
//				//����װ�װ� 2022��8��3��
//        temp = base_info->distance / 1000.f;
//        config->yaw_shoot_bias = -1.5;
//        config->pitch_shoot_bias = 0.0544f * temp * temp * temp * temp
//                                  -0.7505f * temp * temp * temp
//                                  +3.8635f * temp * temp
//                                  -7.5982f * temp
//                                  +6.9375f;
//				
//				//��׼װ�װ� 2022��8��3��
////				temp = base_info->distance / 1000.f;
////				config->yaw_shoot_bias = -0.0624f * temp - 1.8658f;
////        if(config->yaw_shoot_bias >= -1.9)
////        {
////          config->yaw_shoot_bias = -1.9;
////        }
////        else if(config->yaw_shoot_bias <= -2)
////        {
////          config->yaw_shoot_bias = -2;
////        }
////				config->pitch_shoot_bias = 6.9493f * pow(temp, -1.289);
////        if(config->pitch_shoot_bias >= 6)
////        {
////          config->pitch_shoot_bias = 6;
////        }
//				/* ̧ͷ���� end */
//				
//				/* yaw��Ԥ���� begin */
//				temp = base_info->distance / 1000.f;
//				config->predict_con = 0.08f * temp;
//				info->yaw_predict = (base_info->target_yaw_speed) * config->predict_con;
//				info->yaw_predict_K = lowpass(info->yaw_predict_K, info->yaw_predict, config->predict_lp_con);
//				/* yaw��Ԥ���� end */
//				
//			/* ���� end */
//			
//			
//			/* ������ begin */
//				
//				/* ���ݴ��� begin */
//					
//					/* Ŀ��yaw��ǶȾ�ֵ begin */
//					if(base_info->real_find_times == 1)
//					{
//						info->target_yaw_angle_ave_last = info->target_yaw_angle_ave;
//						info->target_yaw_angle_ave = base_info->target_yaw_angle;
//					}
//					else if(base_info->real_find_times > 1)
//					{
//						info->target_yaw_angle_ave = ave(base_info->real_find_times - 1, 
//						                                 info->target_yaw_angle_ave, 
//						                                 base_info->target_yaw_angle);
//					}
//					/* Ŀ��yaw��ǶȾ�ֵ end */
//					
//					/* Ŀ��yaw���ٶȾ�ֵ begin */
//					info->target_yaw_speed_ave = ave(base_info->real_find_times - 2,
//					                                 info->target_yaw_speed_ave, 
//					                                 base_info->target_yaw_speed);
//					/* Ŀ��yaw���ٶȾ�ֵ end */
//					
//					/* Ŀ��pitch��Ƕ�ָ���ֵ begin */
//					if(base_info->real_find_times == 1)
//					{
//						info->target_pitch_angle_F_last = info->target_pitch_angle_F;
//						info->target_pitch_angle_F = base_info->target_pitch_angle;
//						info->target_pitch_angle_F_yaw_err = base_info->yaw_err;
//					}
//					else if(abs(base_info->yaw_err+config->yaw_shoot_bias) < abs(info->target_pitch_angle_F_yaw_err+config->yaw_shoot_bias))
//					{
//						info->target_pitch_angle_F = base_info->target_pitch_angle;
//						info->target_pitch_angle_F_yaw_err = base_info->yaw_err;
//					}
//					/* Ŀ��pitch��Ƕ�ָ���ֵ end */
//					
//					/* Ŀ�����ָ���ֵ begin */
//					if(base_info->real_find_times == 1)
//					{
//            info->target_distance_F_last = info->target_distance_F;
//						info->target_distance_F = base_info->distance;
//						info->target_distance_F_yaw_err = base_info->yaw_err;
//					}
//					else if(abs(base_info->yaw_err+config->yaw_shoot_bias) < abs(info->target_distance_F_yaw_err+config->yaw_shoot_bias))
//					{
//						info->target_distance_F = base_info->distance;
//						info->target_distance_F_yaw_err = base_info->yaw_err;
//					}
//					/* Ŀ�����ָ���ֵ end */
//					
//				/* ���ݴ��� end */
//				
//				/* ��С����ʶ�� begin */
//				if(base_info->real_find_times == config->real_find_times_max + 1)  //��ʱ����ٵ���װ�װ�
//				{
//					info->spin_Y_O_N = 0;
//					info->target_yaw_angle_ave = 0;
//					info->target_yaw_angle_ave_last = 0;
//					info->target_pitch_angle_F_last = 0;
//					info->yaw_predict_F = 0;
//				}
//				else if(base_info->real_find_times == 0)  //��ʱ��ʶ�𲻵�װ�װ�
//				{
//					info->spin_Y_O_N = 0;
//					base_info->target_yaw_angle_begin = 0;
//					base_info->target_yaw_angle_end = 0;
//					base_info->target_yaw_angle_total = 0;
//					base_info->target_yaw_angle_last = 0;
//					info->target_yaw_angle_ave_last = 0;
//					info->target_pitch_angle_F_last = 0;
//					info->yaw_predict_F = 0;
//				}
//				/* ��С����ʶ�� end */
//				
//				/* С����ʶ�� begin */
//				if(base_info->real_find_times == 1)
//				{
//					if( (abs(base_info->target_yaw_angle_begin - base_info->target_yaw_angle_end) >= config->spin_target_yaw_err)
//					&& (base_info->target_yaw_angle_end != 0) 
//          && (info->target_yaw_angle_ave_last != 0) )
//					{
//						info->spin_Y_O_N = 1;
//					}
//				}
//				/* С����ʶ�� end */ 
//				
//				/* ��� */
//				if(info->spin_Y_O_N == 0)
//				{
//					output->yaw_move =  base_info->yaw_err  //�����ƶ���
//														+ info->yaw_predict_K  //Ԥ����
//														+ config->yaw_shoot_bias;  //������
//					output->pitch_move = base_info->pitch_err + config->pitch_shoot_bias;
//				}
//				else if( (info->spin_Y_O_N == 1) && (base_info->real_find_times == 1) )
//				{
//					if(info->target_yaw_angle_ave_last != 0)  //�ڶ���
//					{
//						/* ������ʱyawԤ��������begin */
//						temp = info->target_distance_F_last / 1000.f;
//						config->predict_con_F = 0.08f * temp + config->predict_con_F_bias;
//						info->yaw_predict_F = info->target_yaw_speed_ave * config->predict_con_F;
//						if(base_info->target_yaw_angle_total < 0)
//						{
//							int times = 0;
//							while(info->yaw_predict_F < base_info->target_yaw_angle_begin - gimbal.info->yaw_imu_angle + base_info->target_yaw_angle_total)
//							{
//								info->yaw_predict_F -= base_info->target_yaw_angle_total;
//								times++;
//								if(times >= 10)
//								{
//									break;
//								}
//							}
//						}
//						else if(base_info->target_yaw_angle_total > 0)
//						{
//							int times = 0;
//							while(info->yaw_predict_F > base_info->target_yaw_angle_begin - gimbal.info->yaw_imu_angle + base_info->target_yaw_angle_total)
//							{
//								info->yaw_predict_F -= base_info->target_yaw_angle_total;
//								times++;
//								if(times >= 10)
//								{
//									break;
//								}
//							}
//						}
//					}
//					else if(info->target_yaw_angle_ave_last == 0)  //��һ��
//					{
//						info->yaw_predict_F = 0;
//					}
//					/* ������ʱyawԤ��������end */
//					
//					//����װ�װ�2022��7��12��
//          temp = info->target_distance_F_last / 1000.f;
//          config->yaw_shoot_bias = -1.5;
//          config->pitch_shoot_bias = 0.0544f * temp * temp * temp * temp
//                                    -0.7505f * temp * temp * temp
//                                    +3.8635f * temp * temp
//                                    -7.5982f * temp
//                                    +6.9375f;
//					//��׼װ�װ� 2022��8��3��
////          temp = info->target_distance_F_last / 1000.f;
////          config->yaw_shoot_bias = -0.0624f * temp - 1.8658f;
////          if(config->yaw_shoot_bias >= -1.9)
////          {
////            config->yaw_shoot_bias = -1.9;
////          }
////          else if(config->yaw_shoot_bias <= -2)
////          {
////            config->yaw_shoot_bias = -2;
////          }
////          config->pitch_shoot_bias = 6.9493f * pow(temp, -1.289);
////          if(config->pitch_shoot_bias >= 6)
////          {
////            config->pitch_shoot_bias = 6;
////          }
//					
//					output->yaw_move = info->target_yaw_angle_ave_last - gimbal.info->yaw_imu_angle + config->yaw_shoot_bias;
//					if(info->target_pitch_angle_F_last != 0)
//					{
//						output->pitch_move = (info->target_pitch_angle_F + info->target_pitch_angle_F_last) / 2.f - gimbal.info->pitch_imu_angle + config->pitch_shoot_bias;
//					}
//					else if(info->target_pitch_angle_F_last == 0)
//					{
//						output->pitch_move = info->target_pitch_angle_F - gimbal.info->pitch_imu_angle + config->pitch_shoot_bias;
//					}
//				}
//			/* ������end */
//		}
//		else //ûʶ��װ�װ�
//		{
//			output->yaw_move = 0;
//			output->pitch_move = 0;
//		}
//		
//		info->interrupt = 0;
//	}
//}

/**
  * @brief  �Ӿ�ʵʱ����
  * @param  
  * @retval 
  */
void vision_tick_task(vision_t *vision)
{
	vision_info_t *info = vision->info;
	vision_config_t *config = vision->config;
	vision_base_info_t *base_info = vision->base_info;
	
	info->offline_cnt++;
	if(info->offline_cnt >= vision->config->offline_cnt_max)
	{
		info->offline_cnt = vision->config->offline_cnt_max + 1;
		info->status = DEV_OFFLINE;
	}
	
//	info->shoot_delay_cnt++;
//	if(info->shoot_delay_cnt >= config->shoot_delay_cnt_max)
//	{
//		info->shoot_delay_cnt = config->shoot_delay_cnt_max + 1;
//	}
//	
//	float temp = info->yaw_predict_F + base_info->yaw_err + 2 * config->yaw_shoot_bias;
//	if(info->spin_Y_O_N == 1 && abs(temp) <= 0.02f  && info->yaw_predict_F != 0)
//	{
//		info->shoot_Y_O_N = 1;
//		if(info->shoot_delay_cnt >= config->shoot_delay_cnt_max && config->shoot_permit == 1)
//		{
//			config->shoot_permit = 0;
//			info->shoot_delay_cnt = 0;
//			Launcher_Shoot = true;
//		}
//	}
//	else 
//	{
//		info->shoot_Y_O_N = 0;
//	}
}

/**
* @brief  �Ӿ�����
  * @param  
  * @retval 
  */
bool vision_send( uint8_t  datau8_1,\
                  float    dataf_1, \
                  float    dataf_2, \
                  uint8_t  datau8_2)
{
	memcpy(&vision_tx_info.datau8_1, &datau8_1, 1);
	memcpy((void*)&vision_tx_info.dataf_1, &dataf_1, 4);
	memcpy((void*)&vision_tx_info.dataf_2, &dataf_2, 4);
	memcpy(&vision_tx_info.datau8_2, &datau8_2, 1);
	if(vision_send_data() == true)
	{
			return true;
	}
	return false;
}

/**
  * @brief  �Ӿ�����
  * @param  
  * @retval 
  */
bool vision_recieve(uint8_t  *rxBuf)
{
	vision_base_info_t *base_info = vision.base_info;
//	vision_config_t *config = vision.config;
	vision_info_t *info = vision.info;
	
	timer_cycle(&vision_task_timer);
	
	info->offline_cnt_last = info->offline_cnt;
	info->offline_cnt = 0;
	
	if(vision_recieve_data(rxBuf) == true)
	{
		info->interrupt = 1;
		
		/* ��¼yaw_err_last */
		base_info->yaw_err_last = vision.base_info->yaw_err;
		
		//��������
//		memcpy(&datau8_1, &Vision_Rx_Info.datau8_1, 1);
		memcpy(&base_info->pitch_err, (void*)&vision_rx_info.dataf_1, 4);
		memcpy(&base_info->yaw_err, (void*)&vision_rx_info.dataf_2, 4);
		memcpy(&base_info->distance, (void*)&vision_rx_info.dataf_3, 4);
		memcpy(&base_info->find_Y_O_N, &vision_rx_info.datau8_2, 1);
		memcpy(&base_info->target_car_num, &vision_rx_info.datau8_3, 1);
		
		vision.output->yaw_move = base_info->yaw_err;
		vision.output->pitch_move = base_info->pitch_err;

		return true;
   }
   return false;
}

