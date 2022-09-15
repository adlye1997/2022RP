/**
  ******************************************************************************
  * @file           : my_judge.c\h
	* @author         : czf
	* @date           : 
  * @brief          : ���ݡ�RoboMaster_����ϵͳ����Э�鸽¼ V1.3����д
	                    ��Ի����˼佻������
	* @history        : 
  ******************************************************************************
  */

/* Ŀ¼begin */

//���Ĳ�
//**********ʵʱ����
//������
//**********��������
//�ڶ���
//**********��ȡͼ������֡
//********************��ȡֱ������֡
//********************��ȡ��������֡
//********************��ȡ��Բ����֡
//********************��ȡ��Բ����֡
//********************��ȡԲ������֡
//********************��ȡ����������֡
//********************��ȡ����������֡
//********************��ȡ�ַ�����֡
//**********����֡����
//********************���ͻ���һ��ͼ��֡����
//********************���ͻ��ƶ���ͼ��֡����
//********************���ͻ������ͼ��֡����
//********************���ͻ����߸�ͼ��֡����
//********************���ͻ����ַ�֡����
//��һ��
//**********���ڷ�������

/* Ŀ¼end */

#include "my_judge.h"
#include "crc8.h"
#include "crc16.h"
#include "string.h"
#include "stdbool.h"
#include "arm_math.h"
#include "stdio.h"
#include "judge_potocol.h"
#include "cap_protocol.h"
#include "judge.h"
#include "launcher.h"

/* ������ begin */
#define UI_huart huart4  //���ڽӿ�
#define send_frequency 30  //����Ƶ��
/* ������ end */

extern UART_HandleTypeDef UI_huart;

client_info_t client_info = 
{
	.robot_id = 1,
	.client_id = 0x0101,
};
uint8_t client_tx_buf[128];

ext_client_custom_graphic_seven_t g0;
ext_client_custom_graphic_seven_t g1;
/**************************************************���Ĳ�begin**************************************************/

/**
  * @brief  ʵʱ����
  * @param  �������ڣ�ms��
  * @retval 
  */
void tick_task(uint16_t time)
{
	uint16_t cnt_max = 1000 / time / send_frequency;
	uint8_t task_num_max = 2;
	static uint16_t cnt = 0;
	static uint8_t task_num = 0;
	
	if(cnt >= cnt_max)
	{
    client_info_update();
		switch(task_num)
		{
			case 0:
        draw_task_0();
				break;
			case 1:
        draw_task_1();
				break;
			case 2:
				break;
			case 3:
				break;
			case 4:
				break;
			case 5:
				break;
			case 6:
				break;
      default:
        break;
		}
		task_num++;
		task_num %= task_num_max;
		cnt = 0;
	}
	else 
	{
		cnt++;
	}
}

/**
  * @brief  �ͻ�����Ϣ����
  * @param  
  * @retval 
  */
void client_info_update(void)
{
  
}

/**
  * @brief  �滭����0
  * @param  
  * @retval 
  */
int test = 0;
void draw_task_0(void)
{
  static int cnt = 0;
  static int operate_tpye = ADD;
  
  if(cnt == 0)
  {
    operate_tpye = ADD;  //һ����һ����ADD
  }
  else 
  {
    operate_tpye = MODIFY;
  }
  
  /* ��׼ǰ��ս���� */
  g0.grapic_data_struct[0] = draw_line("g00",  //ͼ����
                                       ADD,  //ͼ�β���
                                       0,  //ͼ������0~9
                                       YELLOW,  //��ɫ
                                       2,  //��������
                                       Client_mid_position_x-17,  //��� x ����
                                       0,  //��� y ����
                                       Client_mid_position_x-17,  //�յ� x ����
                                       Client_mid_position_y*2);  //�յ� y ����
  /* ��׼ǰ��ս���� */
  g0.grapic_data_struct[1] = draw_line("g01",  //ͼ����
                                       ADD,  //ͼ�β���
                                       0,  //ͼ������0~9
                                       YELLOW,  //��ɫ
                                       2,  //��������
                                       Client_mid_position_x-16-10,  //��� x ����
                                       Client_mid_position_y+95,  //��� y ����
                                       Client_mid_position_x-16+10,  //�յ� x ����
                                       Client_mid_position_y+95);  //�յ� y ����
  /* ��׼�������� */
  g0.grapic_data_struct[2] = draw_line("g02",  //ͼ����
                                       ADD,  //ͼ�β���
                                       0,  //ͼ������0~9
                                       YELLOW,  //��ɫ
                                       3,  //��������
                                       Client_mid_position_x+100,  //��� x ����
                                       0,  //��� y ����
                                       Client_mid_position_x+100,  //�յ� x ����
                                       Client_mid_position_y*2);  //�յ� y ����
  /* ��׼���غ��� */
  g0.grapic_data_struct[3] = draw_line("g03",  //ͼ����
                                       ADD,  //ͼ�β���
                                       0,  //ͼ������0~9
                                       YELLOW,  //��ɫ
                                       3,  //��������
                                       Client_mid_position_x+90,  //��� x ����
                                       Client_mid_position_y,  //��� y ����
                                       Client_mid_position_x+110,  //�յ� x ����
                                       Client_mid_position_y);  //�յ� y ����
//  /* ͼ������ */
//  g0.grapic_data_struct[4] = draw_line("g04",  //ͼ����
//                                       ADD,  //ͼ�β���
//                                       0,  //ͼ������0~9
//                                       YELLOW,  //��ɫ
//                                       3,  //��������
//                                       Client_mid_position_x-100,  //��� x ����
//                                       0,  //��� y ����
//                                       Client_mid_position_x-100,  //�յ� x ����
//                                       Client_mid_position_y*2);  //�յ� y ����
//  /* ͼ������ */
//  g0.grapic_data_struct[5] = draw_line("g05",  //ͼ����
//                                       ADD,  //ͼ�β���
//                                       0,  //ͼ������0~9
//                                       YELLOW,  //��ɫ
//                                       3,  //��������
//                                       Client_mid_position_x-110,  //��� x ����
//                                       Client_mid_position_y,  //��� y ����
//                                       Client_mid_position_x-90,  //�յ� x ����
//                                       Client_mid_position_y);  //�յ� y ����

  client_send_seven_graphic(g0);
  
  cnt++;
  cnt %= 30;  //һ����һ����ADD
}

/**
  * @brief  �滭����1
  * @param  
  * @retval 
  */
void draw_task_1(void)
{
  static int cnt = 0;
  static int operate_tpye = ADD;
  
  if(cnt == 0)
  {
    operate_tpye = ADD;  //һ����һ����ADD
  }
  else 
  {
    operate_tpye = MODIFY;
  }
  
  /* ������ */  //
  g1.grapic_data_struct[0] = draw_line("g10",  //ͼ����
                                       operate_tpye,  //ͼ�β���
                                       1,  //ͼ������0~9
                                       YELLOW,  //��ɫ
                                       20,  //��������
                                       150,  //��� x ����
                                       Client_mid_position_y + 250,  //��� y ����
                                       450,  //�յ� x ����
                                       Client_mid_position_y + 250);  //�յ� y ����
  /* �������yaw�� */
  g1.grapic_data_struct[1] = draw_int("g11",  //ͼ����
                                      operate_tpye,  //ͼ�β���
                                      1,  //ͼ������0~9
                                      YELLOW,  //��ɫ
                                      30,  //�����С
                                      3,  //��������
                                      Client_mid_position_x*2 - 400,  //��� x ����
                                      Client_mid_position_y + 300,  //��� y ����
                                      launcher.info->yaw_angle);  //32 λ��������int32_t
  /* �������pitch�� */
  g1.grapic_data_struct[2] = draw_int("g12",  //ͼ����
                                      operate_tpye,  //ͼ�β���
                                      1,  //ͼ������0~9
                                      YELLOW,  //��ɫ
                                      30,  //�����С
                                      3,  //��������
                                      Client_mid_position_x*2 - 400,  //��� x ����
                                      Client_mid_position_y + 200,  //��� y ����
                                      launcher.info->pitch_angle);  //32 λ��������int32_t
  /* �Զ������־ */  //�����ڷ��� ������
  g1.grapic_data_struct[3] = draw_circle("g13",  //ͼ����
                                         operate_tpye,  //ͼ�β���
                                         1,  //ͼ������0~9
                                         YELLOW,  //��ɫ
                                         30,  //��������
                                         Client_mid_position_x*2 - 170,  //Բ�� x ����
                                         Client_mid_position_y+100,  //Բ�� y ����
                                         15);  //�뾶
  /* �����������־ */  //������ FUCHSIAû����
  g1.grapic_data_struct[4] = draw_circle("g14",  //ͼ����
                                         operate_tpye,  //ͼ�β���
                                         1,  //ͼ������0~9
                                         YELLOW,  //��ɫ
                                         30,  //��������
                                         Client_mid_position_x*2 - 170,  //Բ�� x ����
                                         Client_mid_position_y,  //Բ�� y ����
                                         15);  //�뾶
  
  client_send_seven_graphic(g1);
  
  cnt++;
  cnt %= 30;  //һ����һ����ADD
  
}

/**************************************************���Ĳ�end**************************************************/


/**************************************************������begin**************************************************/

/******************************��������begin******************************/

/******************************��������end******************************/

/**************************************************������end**************************************************/

/**************************************************�ڶ���begin**************************************************/

/******************************��ȡͼ������֡begin******************************/

/**
  * @brief  ��ȡֱ������֡
  * @param  
  * @retval ͼ�����ݽṹ��
  */
graphic_data_struct_t draw_line(char *name,  //ͼ����
	             uint8_t operate_tpye,  //ͼ�β���
               uint8_t layer,  //ͼ������0~9
               uint8_t color,  //��ɫ
               uint16_t width,  //��������
               uint16_t start_x,  //��� x ����
               uint16_t start_y,  //��� y ����
               uint16_t end_x,  //�յ� x ����
               uint16_t end_y)  //�յ� y ����
{
	graphic_data_struct_t data;
	
	memcpy(data.graphic_name, name, 3);
	data.operate_tpye = operate_tpye;
	data.graphic_tpye = 0;
	data.layer = layer;
	data.color = color;
	data.start_angle = 0;
	data.end_angle = 0;
	data.width = width;
	data.start_x = start_x;
	data.start_y = start_y;
	data.radius = 0;
	data.end_x = end_x;
	data.end_y = end_y;
	
	return data;
}

/**
  * @brief  ��ȡ��������֡
  * @param  
  * @retval ͼ�����ݽṹ��
  */
graphic_data_struct_t draw_rectangle(char *name,  //ͼ����
	             uint8_t operate_tpye,  //ͼ�β���
               uint8_t layer,  //ͼ������0~9
               uint8_t color,  //��ɫDSFZa
               uint16_t width,  //��������
               uint16_t start_x,  //��� x ����
               uint16_t start_y,  //��� y ����
               uint16_t end_x,  //�ԽǶ��� x ����
               uint16_t end_y)  //�ԽǶ��� y ����
{
	graphic_data_struct_t data;
	
	memcpy(data.graphic_name, name, 3);
	data.operate_tpye = operate_tpye;
	data.graphic_tpye = 1;
	data.layer = layer;
	data.color = color;
	data.start_angle = 0;
	data.end_angle = 0;
	data.width = width;
	data.start_x = start_x;
	data.start_y = start_y;
	data.radius = 0;
	data.end_x = end_x;
	data.end_y = end_y;
	
	return data;
}

/**
  * @brief  ��ȡ��Բ����֡
  * @param  
  * @retval ͼ�����ݽṹ��
  */
graphic_data_struct_t draw_circle(char *name,  //ͼ����
	             uint8_t operate_tpye,  //ͼ�β���
               uint8_t layer,  //ͼ������0~9
               uint8_t color,  //��ɫ
               uint16_t width,  //��������
               uint16_t ciclemid_x,  //Բ�� x ����
               uint16_t ciclemid_y,  //Բ�� y ����
               uint16_t radius)  //�뾶
{
	graphic_data_struct_t data;
	
	memcpy(data.graphic_name, name, 3);
	data.operate_tpye = operate_tpye;
	data.graphic_tpye = 2;
	data.layer = layer;
	data.color = color;
	data.start_angle = 0;
	data.end_angle = 0;
	data.width = width;
	data.start_x = ciclemid_x;
	data.start_y = ciclemid_y;
	data.radius = radius;
	data.end_x = 0;
	data.end_y = 0;
	
	return data;
}

/**
  * @brief  ��ȡ��Բ����֡
  * @param  
  * @retval ͼ�����ݽṹ��
  */
graphic_data_struct_t draw_ellipse(char *name,  //ͼ����
	             uint8_t operate_tpye,  //ͼ�β���
               uint8_t layer,  //ͼ������0~9
               uint8_t color,  //��ɫ
               uint16_t width,  //��������
               uint16_t start_x,  //Բ�� x ����
               uint16_t start_y,  //Բ�� y ����
               uint16_t end_x,  //x ���᳤��
               uint16_t end_y)  //y ���᳤��
{
	graphic_data_struct_t data;
	
	memcpy(data.graphic_name, name, 3);
	data.operate_tpye = operate_tpye;
	data.graphic_tpye = 3;
	data.layer = layer;
	data.color = color;
	data.start_angle = 0;
	data.end_angle = 0;
	data.width = width;
	data.start_x = start_x;
	data.start_y = start_y;
	data.radius = 0;
	data.end_x = end_x;
	data.end_y = end_y;
	
	return data;
}

/**
  * @brief  ��ȡԲ������֡
  * @param  
  * @retval ͼ�����ݽṹ��
  */
graphic_data_struct_t draw_arc(char *name,  //ͼ����
	             uint8_t operate_tpye,  //ͼ�β���
               uint8_t layer,  //ͼ������0~9
               uint8_t color,  //��ɫ
               uint16_t start_angle,  //��ʼ�Ƕ�
               uint16_t end_angle,  //��ֹ�Ƕ�
               uint16_t width,  //��������
               uint16_t circlemin_x,  //Բ�� x ����
               uint16_t circlemin_y,  //Բ�� y ����
               uint16_t end_x,  //x ���᳤��
               uint16_t end_y)  //y ���᳤��
{
	graphic_data_struct_t data;
	
	memcpy(data.graphic_name, name, 3);
	data.operate_tpye = operate_tpye;
	data.graphic_tpye = 4;
	data.layer = layer;
	data.color = color;
	data.start_angle = start_angle;
	data.end_angle = end_angle;
	data.width = width;
	data.start_x = circlemin_x;
	data.start_y = circlemin_y;
	data.radius = 0;
	data.end_x = end_x;
	data.end_y = end_y;
	
	return data;
}

/**
  * @brief  ��ȡ����������֡
  * @param  
  * @retval ͼ�����ݽṹ��
  */
graphic_data_struct_t draw_float(char *name,  //ͼ����
	             uint8_t operate_tpye,  //ͼ�β���
               uint8_t layer,  //ͼ������0~9
               uint8_t color,  //��ɫ
               uint16_t size,  //�����С
               uint16_t decimal,  //С��λ��Ч����
               uint16_t width,  //��������
               uint16_t start_x,  //��� x ����
               uint16_t start_y,  //��� y ����
               int32_t num)  //���� 1000 ���� 32 λ��������int32_t
{
	graphic_data_struct_t data;
	
	memcpy(data.graphic_name, name, 3);
	data.operate_tpye = operate_tpye;
	data.graphic_tpye = 5;
	data.layer = layer;
	data.color = color;
	data.start_angle = size;
	data.end_angle = decimal;
	data.width = width;
	data.start_x = start_x;
	data.start_y = start_y;
	data.radius = num;
	data.end_x = num >> 10;
	data.end_y = num >> 21;
	
	return data;
}

/**
  * @brief  ��ȡ����������֡
  * @param  
  * @retval ͼ�����ݽṹ��
  */
graphic_data_struct_t draw_int(char *name,  //ͼ����
	             uint8_t operate_tpye,  //ͼ�β���
               uint8_t layer,  //ͼ������0~9
               uint8_t color,  //��ɫ
               uint16_t size,  //�����С
               uint16_t width,  //��������
               uint16_t start_x,  //��� x ����
               uint16_t start_y,  //��� y ����
               int32_t num)  //32 λ��������int32_t
{
	graphic_data_struct_t data;
	
	memcpy(data.graphic_name, name, 3);
	data.operate_tpye = operate_tpye;
	data.graphic_tpye = 6;
	data.layer = layer;
	data.color = color;
	data.start_angle = size;
	data.end_angle = 0;
	data.width = width;
	data.start_x = start_x;
	data.start_y = start_y;
	data.radius = num;
	data.end_x = num >> 10;
	data.end_y = num >> 21;
	
	return data;
}

/**
  * @brief  ��ȡ�ַ�����֡
  * @param  
  * @retval ͼ�����ݽṹ��
  */
graphic_data_struct_t draw_char(char *name,  //ͼ����
	             uint8_t operate_tpye,  //ͼ�β���
               uint8_t layer,  //ͼ������0~9
               uint8_t color,  //��ɫ
               uint16_t size,  //�����С
               uint16_t length,  //�ַ�����
               uint16_t width,  //��������
               uint16_t start_x,  //��� x ����
               uint16_t start_y)  //��� y ����
{
	graphic_data_struct_t data;
	
	memcpy(data.graphic_name, name, 3);
	data.operate_tpye = operate_tpye;
	data.graphic_tpye = 7;
	data.layer = layer;
	data.color = color;
	data.start_angle = size;
	data.end_angle = length;
	data.width = width;
	data.start_x = start_x;
	data.start_y = start_y;
	data.radius = 0;
	data.end_x = 0;
	data.end_y = 0;
	
	return data;
}

/******************************��ȡͼ������֡end******************************/


/******************************����֡����begin******************************/

/**
	* @brief  ���ͻ���һ��ͼ��֡����
  * @param  
  * @retval 
  */
uint8_t client_send_single_graphic(ext_client_custom_graphic_single_t data)
{
	frame_t frame;
	ext_student_interactive_header_data_t data_header;
	
	/* ֡ͷ */
	frame.frame_header.SOF = 0xA5;
	frame.frame_header.data_length = LEN_ID_draw_one_graphic;
	frame.frame_header.seq = 0;
	memcpy(client_tx_buf, &frame.frame_header, 4);
	Append_CRC8_Check_Num(client_tx_buf, 5);
	
	/* ������ID */
	frame.cmd_id = 0x301;
	memcpy(&client_tx_buf[5], (void*)&frame.cmd_id, 2);
	
	/* ���ݶ� */
	data_header.data_cmd_id = ID_draw_one_graphic;
	data_header.sender_ID = client_info.robot_id;
	data_header.receiver_ID = client_info.client_id;
	memcpy(&client_tx_buf[7], &data_header, 6);
	memcpy(&client_tx_buf[13], &data.grapic_data_struct, 15);
	
	/* ֡β */
	Append_CRC16_Check_Sum(client_tx_buf, 5 + 2 + LEN_ID_draw_one_graphic + 2);
	
	/* ���� */
	return uart_send_data(client_tx_buf, 5 + 2 + LEN_ID_draw_one_graphic + 2);
}

/**
	* @brief  ���ͻ��ƶ���ͼ��֡����
  * @param  
  * @retval 
  */
uint8_t client_send_double_graphic(ext_client_custom_graphic_double_t data)
{
	frame_t frame;
	ext_student_interactive_header_data_t data_header;
	
	/* ֡ͷ */
	frame.frame_header.SOF = 0xA5;
	frame.frame_header.data_length = LEN_ID_draw_two_graphic;
	frame.frame_header.seq = 0;
	memcpy(client_tx_buf, &frame.frame_header, 4);
	Append_CRC8_Check_Num(client_tx_buf, 5);
	
	/* ������ID */
	frame.cmd_id = 0x301;
	memcpy(&client_tx_buf[5], (void*)&frame.cmd_id, 2);
	
	/* ���ݶ� */
	data_header.data_cmd_id = ID_draw_two_graphic;
	data_header.sender_ID = client_info.robot_id;
	data_header.receiver_ID = client_info.client_id;
	memcpy(&client_tx_buf[7], &data_header, 6);
	memcpy(&client_tx_buf[13], data.grapic_data_struct, 15*2);
	
	/* ֡β */
	Append_CRC16_Check_Sum(client_tx_buf, 5 + 2 + LEN_ID_draw_two_graphic + 2);
	
	/* ���� */
	return uart_send_data(client_tx_buf, 5 + 2 + LEN_ID_draw_two_graphic + 2);
}

/**
	* @brief  ���ͻ������ͼ��֡����
  * @param  
  * @retval 
  */
uint8_t client_send_five_graphic(ext_client_custom_graphic_five_t data)
{
	frame_t frame;
	ext_student_interactive_header_data_t data_header;
	
	/* ֡ͷ */
	frame.frame_header.SOF = 0xA5;
	frame.frame_header.data_length = LEN_ID_draw_five_graphic;
	frame.frame_header.seq = 0;
	memcpy(client_tx_buf, &frame.frame_header, 4);
	Append_CRC8_Check_Num(client_tx_buf, 5);
	
	/* ������ID */
	frame.cmd_id = 0x301;
	memcpy(&client_tx_buf[5], (void*)&frame.cmd_id, 2);
	
	/* ���ݶ� */
	data_header.data_cmd_id = ID_draw_five_graphic;
	data_header.sender_ID = client_info.robot_id;
	data_header.receiver_ID = client_info.client_id;
	memcpy(&client_tx_buf[7], &data_header, 6);
	memcpy(&client_tx_buf[13], data.grapic_data_struct, 15*5);
	
	/* ֡β */
	Append_CRC16_Check_Sum(client_tx_buf, 5 + 2 + LEN_ID_draw_five_graphic + 2);
	
	/* ���� */
	return uart_send_data(client_tx_buf, 5 + 2 + LEN_ID_draw_five_graphic + 2);
}

/**
	* @brief  ���ͻ����߸�ͼ��֡����
  * @param  
  * @retval 
  */
uint8_t client_send_seven_graphic(ext_client_custom_graphic_seven_t data)
{
	frame_t frame;
	ext_student_interactive_header_data_t data_header;
	
	/* ֡ͷ */
	frame.frame_header.SOF = 0xA5;
	frame.frame_header.data_length = LEN_ID_draw_seven_graphic;
	frame.frame_header.seq = 0;
	memcpy(client_tx_buf, &frame.frame_header, 4);
	Append_CRC8_Check_Num(client_tx_buf, 5);
	
	/* ������ID */
	frame.cmd_id = 0x301;
	memcpy(&client_tx_buf[5], (void*)&frame.cmd_id, 2);
	
	/* ���ݶ� */
	data_header.data_cmd_id = ID_draw_seven_graphic;
	data_header.sender_ID = client_info.robot_id;
	data_header.receiver_ID = client_info.client_id;
	memcpy(&client_tx_buf[7], &data_header, 6);
	memcpy(&client_tx_buf[13], data.grapic_data_struct, 15*7);
	
	/* ֡β */
	Append_CRC16_Check_Sum(client_tx_buf, 5 + 2 + LEN_ID_draw_seven_graphic + 2);
	
	/* ���� */
	return uart_send_data(client_tx_buf, 5 + 2 + LEN_ID_draw_seven_graphic + 2);
}

/**
	* @brief  ���ͻ����ַ�֡����
  * @param  
  * @retval 
  */
uint8_t client_send_char(ext_client_custom_character_t data)
{
	frame_t frame;
	ext_student_interactive_header_data_t data_header;
	
	/* ֡ͷ */
	frame.frame_header.SOF = 0xA5;
	frame.frame_header.data_length = LEN_ID_draw_char_graphic;
	frame.frame_header.seq = 0;
	memcpy(client_tx_buf, &frame.frame_header, 4);
	Append_CRC8_Check_Num(client_tx_buf, 5);
	
	/* ������ID */
	frame.cmd_id = 0x301;
	memcpy(&client_tx_buf[5], (void*)&frame.cmd_id, 2);
	
	/* ���ݶ� */
	data_header.data_cmd_id = ID_draw_char_graphic;
	data_header.sender_ID = client_info.robot_id;
	data_header.receiver_ID = client_info.client_id;
	memcpy(&client_tx_buf[7], &data_header, 6);
	memcpy(&client_tx_buf[13], &data.grapic_data_struct, 15);
	memcpy(&client_tx_buf[28], data.data, 30);
	
	/* ֡β */
	Append_CRC16_Check_Sum(client_tx_buf, 5 + 2 + LEN_ID_draw_char_graphic + 2);
	
	/* ���� */
	return uart_send_data(client_tx_buf, 5 + 2 + LEN_ID_draw_char_graphic + 2);
}

/******************************����֡����end****************************************/

/**************************************************�ڶ���end**************************************************/


/**************************************************��һ��begin**************************************************/

/**
	* @brief  ���ڷ�������
  * @param  
  * @retval 
  */
uint8_t uart_send_data(uint8_t *txbuf, uint16_t length)
{
	return HAL_UART_Transmit_DMA(&UI_huart, txbuf, length);
}

/**************************************************��һ��end**************************************************/