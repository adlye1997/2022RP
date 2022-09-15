/**
  ******************************************************************************
  * @file           : judge_potocol.c\h
	* @author         : czf
	* @date           : 2022-4-22 14:39:50
  * @brief          : 
	* @history        : 
  ******************************************************************************
  */
	
#ifndef __JUDGE_POTOCOL_H
#define __JUDGE_POTOCOL_H

#include "stm32f4xx_hal.h"

/* ֡ͷ */
typedef struct 
{
	uint8_t SOF;  //����֡��ʼ�ֽڣ��̶�ֵΪ 0xA5
	uint16_t data_length;  //����֡�� data �ĳ���
	uint8_t seq;  //�����
	uint8_t CRC8;  //֡ͷ CRC8 У��
}judge_frame_header_t;

typedef struct 
{
	judge_frame_header_t *frame_header;
	uint16_t cmd_id;
	uint16_t frame_tail;
}drv_judge_info_t;

extern drv_judge_info_t drv_judge_info;

void judge_recive(uint8_t *rxBuf);

/********************from����ϵͳ����Э�鸽¼V1.3�����ݽṹ��********************/

/* ����״̬���ݣ�0x0001������Ƶ�ʣ�1Hz */
typedef struct
{
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
} ext_game_status_t;

/* ����������ݣ�0x0002������Ƶ�ʣ������������� */
typedef struct
{
	uint8_t winner;
} ext_game_result_t;

/* ������Ѫ�����ݣ�0x0003������Ƶ�ʣ�1Hz */
typedef struct
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP; 
	uint16_t red_3_robot_HP; 
	uint16_t red_4_robot_HP; 
	uint16_t red_5_robot_HP; 
	uint16_t red_7_robot_HP; 
	uint16_t red_outpost_HP;
	uint16_t red_base_HP; 
	uint16_t blue_1_robot_HP; 
	uint16_t blue_2_robot_HP; 
	uint16_t blue_3_robot_HP; 
	uint16_t blue_4_robot_HP; 
	uint16_t blue_5_robot_HP; 
	uint16_t blue_7_robot_HP; 
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
} ext_game_robot_HP_t;

/* �˹�������ս���ӳ�\�ͷ����ֲ���Ǳ��ģʽ״̬��0x0005������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ�����л����� */
typedef struct
{
	uint8_t F1_zone_status:1;
	uint8_t F1_zone_buff_debuff_status:3; 
	uint8_t F2_zone_status:1;
	uint8_t F2_zone_buff_debuff_status:3; 
	uint8_t F3_zone_status:1;
	uint8_t F3_zone_buff_debuff_status:3; 
	uint8_t F4_zone_status:1;
	uint8_t F4_zone_buff_debuff_status:3; 
	uint8_t F5_zone_status:1;
	uint8_t F5_zone_buff_debuff_status:3; 
	uint8_t F6_zone_status:1;
	uint8_t F6_zone_buff_debuff_status:3;
	uint16_t red1_bullet_left;
	uint16_t red2_bullet_left;
	uint16_t blue1_bullet_left;
	uint16_t blue2_bullet_left;
	uint8_t lurk_mode;
	uint8_t res;
} ext_ICRA_buff_debuff_zone_status_t;

/* �����¼����ݣ�0x0101������Ƶ�ʣ�1Hz */
typedef __packed struct
{
	uint32_t event_type;
} ext_event_data_t;

/* ����վ������ʶ��0x0102������Ƶ�ʣ������ı����, ���ͷ�Χ������������ */
typedef __packed struct
{
	uint8_t supply_projectile_id; 
	uint8_t supply_robot_id; 
	uint8_t supply_projectile_step; 
	uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

/* ���о�����Ϣ��cmd_id (0x0104)������Ƶ�ʣ��������淢������ */
typedef __packed struct
{
	uint8_t level;
	uint8_t foul_robot_id; 
} ext_referee_warning_t;

/* ���ڷ���ڵ���ʱ��cmd_id (0x0105)������Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ������������ */
typedef __packed struct
{
	uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

/* ����������״̬��0x0201������Ƶ�ʣ�10Hz */
typedef __packed struct
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t remain_HP;
	uint16_t max_HP;
	uint16_t shooter_id1_17mm_cooling_rate;
	uint16_t shooter_id1_17mm_cooling_limit;
	uint16_t shooter_id1_17mm_speed_limit;
	uint16_t shooter_id2_17mm_cooling_rate;
	uint16_t shooter_id2_17mm_cooling_limit;
	uint16_t shooter_id2_17mm_speed_limit;
	uint16_t shooter_id1_42mm_cooling_rate;
	uint16_t shooter_id1_42mm_cooling_limit;
	uint16_t shooter_id1_42mm_speed_limit;
	uint16_t chassis_power_limit;
	uint8_t mains_power_gimbal_output : 1;
	uint8_t mains_power_chassis_output : 1;
	uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;

/* ʵʱ�����������ݣ�0x0202������Ƶ�ʣ�50Hz */
typedef __packed struct
{
	uint16_t chassis_volt; 
	uint16_t chassis_current; 
	float chassis_power;
	uint16_t chassis_power_buffer; 
	uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t shooter_id2_17mm_cooling_heat;
	uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

/* ������λ�ã�0x0203������Ƶ�ʣ�10Hz */
typedef __packed struct
{
	float x;
	float y;
	float z;
	float yaw;
} ext_game_robot_pos_t;

/* ���������棺0x0204������Ƶ�ʣ�1Hz */
typedef __packed struct
{
	uint8_t power_rune_buff;
}ext_buff_t;

/* ���л���������״̬��0x0205������Ƶ�ʣ�10Hz */
typedef __packed struct
{
	uint8_t attack_time;
} aerial_robot_energy_t;

/* �˺�״̬��0x0206������Ƶ�ʣ��˺��������� */
typedef __packed struct
{
	uint8_t armor_id : 4;
	uint8_t hurt_type : 4;
} ext_robot_hurt_t;

/* ʵʱ�����Ϣ��0x0207������Ƶ�ʣ�������� */
typedef __packed struct
{
	uint8_t bullet_type;
	uint8_t shooter_id;
	uint8_t bullet_freq;
	float bullet_speed;
} ext_shoot_data_t;

/* �ӵ�ʣ�෢������0x0208������Ƶ�ʣ�10Hz ���ڷ��ͣ����л����˷��� */
typedef __packed struct
{
	uint16_t bullet_remaining_num_17mm;
	uint16_t bullet_remaining_num_42mm;
	uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

/* ������ RFID ״̬��0x0209������Ƶ�ʣ�1Hz�����ͷ�Χ����һ������ */
typedef __packed struct
{
	uint32_t rfid_status;
} ext_rfid_status_t;

/* ���ڻ����˿ͻ���ָ�����ݣ�0x020A������Ƶ�ʣ�10Hz�����ͷ�Χ����һ������ */
typedef __packed struct
{
	uint8_t dart_launch_opening_status;
	uint8_t dart_attack_target;
	uint16_t target_change_time;
	uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;

/********************������ö��********************/
enum
{
	ID_game_state       						= 0x0001,//����״̬���ݣ�1Hz
	ID_game_result 	   							= 0x0002,//����������ݣ�������������
	ID_game_robot_HP       					= 0x0003,//����������Ѫ�����ݣ�1Hz����
	ID_dart_status									= 0x0004,//���ڷ���״̬�����ڷ���ʱ����
	ID_ICRA_buff_debuff_zone_status = 0x0005,//�˹�������ս���ӳ���ͷ���״̬��1Hz
	ID_event_data  									= 0x0101,//�����¼����ݣ�1Hz
	ID_supply_projectile_action   	= 0x0102,//���ز���վ������ʶ����
	ID_referee_warning					 		= 0x0104,//���о������ݣ��������
	ID_dart_remaining_time					= 0x0105,//���ڷ���ڵ���ʱ��1Hz
	ID_game_robot_state    					= 0x0201,//������״̬���ݣ�10Hz
	ID_power_heat_data    					= 0x0202,//ʵʱ�����������ݣ�50Hz
	ID_game_robot_pos        				= 0x0203,//������λ�����ݣ�10Hz
	ID_buff_musk										= 0x0204,//�������������ݣ�1Hz
	ID_aerial_robot_energy					= 0x0205,//���л���������״̬���ݣ�10Hz��ֻ�п��л��������ط���
	ID_robot_hurt										= 0x0206,//�˺�״̬���ݣ��˺���������
	ID_shoot_data										= 0x0207,//ʵʱ������ݣ��ӵ��������
	ID_bullet_remaining							= 0x0208,//����ʣ�෢�����������л����ˣ��ڱ��������Լ�ICRA�����˷��ͣ�1Hz
	ID_rfid_status									= 0x0209,//������RFID״̬��1Hz
	ID_interactive_header_data			= 0x0301 //�����˽������ݣ����ͷ���������
};

/********************����ö��********************/
enum
{
	LEN_FRAME_HEAD 	                = 5,	    // ֡ͷ����
	LEN_CMD_ID 		                  = 2,	    // �����볤��
	LEN_FRAME_TAIL 	                = 2,	    // ֡βCRC16
	LEN_game_state       						= 3,			//0x0001,//����״̬���ݣ�1Hz
	LEN_game_result 	   						= 1,			//0x0002,//����������ݣ�������������
	LEN_game_robot_HP       				= 32,			//0x0003,//����������Ѫ�����ݣ�1Hz����
	LEN_dart_status									= 3,			//0x0004,//���ڷ���״̬�����ڷ���ʱ����
	LEN_ICRA_buff_debuff_zone_status= 3,			//0x0005,//�˹�������ս���ӳ���ͷ���״̬��1Hz
	LEN_event_data  								= 4,			//0x0101,//�����¼����ݣ�1Hz
	LEN_supply_projectile_action   	= 4,			//0x0102,//���ز���վ������ʶ����
	LEN_referee_warning					 		= 2,			//0x0104,//���о������ݣ��������
	LEN_dart_remaining_time					= 1,			//0x0105,//���ڷ���ڵ���ʱ��1Hz
	LEN_game_robot_state    				= 27,			//0x0201,//������״̬���ݣ�10Hz
	LEN_power_heat_data    					= 16,			//0x0202,//ʵʱ�����������ݣ�50Hz
	LEN_game_robot_pos        			= 16,			//0x0203,//������λ�����ݣ�10Hz
	LEN_buff_musk										= 1,			//0x0204,//�������������ݣ�1Hz
	LEN_aerial_robot_energy					= 3,			//0x0205,//���л���������״̬���ݣ�10Hz��ֻ�п��л��������ط���
	LEN_robot_hurt									= 1,			//0x0206,//�˺�״̬���ݣ��˺���������
	LEN_shoot_data									= 7,			//0x0207,//ʵʱ������ݣ��ӵ��������
	LEN_bullet_remaining						= 2,			//0x0208,//����ʣ�෢�����������л����ˣ��ڱ��������Լ�ICRA�����˷��ͣ�1Hz
	LEN_rfid_status									= 4				//0x0209,//������RFID״̬��1Hz
//	LEN_interactive_header_data			= n			//0x0301 //�����˽������ݣ����ͷ���������
};

#endif
