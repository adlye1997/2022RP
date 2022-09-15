/**
  ******************************************************************************
  * @file           : my_judge.c\h
	* @author         : czf
	* @date           : 
  * @brief          : 根据《RoboMaster_裁判系统串口协议附录 V1.3》编写
	                    针对机器人间交互数据
	* @history        : 
  ******************************************************************************
  */

/* 目录begin */

//第四层
//**********实时任务
//第三层
//**********绘制任务
//第二层
//**********获取图像数据帧
//********************获取直线数据帧
//********************获取矩形数据帧
//********************获取整圆数据帧
//********************获取椭圆数据帧
//********************获取圆弧数据帧
//********************获取浮点数数据帧
//********************获取整型数数据帧
//********************获取字符数据帧
//**********发送帧数据
//********************发送绘制一个图形帧数据
//********************发送绘制二个图形帧数据
//********************发送绘制五个图形帧数据
//********************发送绘制七个图形帧数据
//********************发送绘制字符帧数据
//第一层
//**********串口发送数据

/* 目录end */

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

/* 配置区 begin */
#define UI_huart huart4  //串口接口
#define send_frequency 30  //发送频率
/* 配置区 end */

extern UART_HandleTypeDef UI_huart;

client_info_t client_info = 
{
	.robot_id = 1,
	.client_id = 0x0101,
};
uint8_t client_tx_buf[128];

ext_client_custom_graphic_seven_t g0;
ext_client_custom_graphic_seven_t g1;
/**************************************************第四层begin**************************************************/

/**
  * @brief  实时任务
  * @param  更新周期（ms）
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
  * @brief  客户端信息更新
  * @param  
  * @retval 
  */
void client_info_update(void)
{
  
}

/**
  * @brief  绘画任务0
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
    operate_tpye = ADD;  //一秒有一次是ADD
  }
  else 
  {
    operate_tpye = MODIFY;
  }
  
  /* 瞄准前哨战竖线 */
  g0.grapic_data_struct[0] = draw_line("g00",  //图形名
                                       ADD,  //图形操作
                                       0,  //图层数，0~9
                                       YELLOW,  //颜色
                                       2,  //线条宽度
                                       Client_mid_position_x-17,  //起点 x 坐标
                                       0,  //起点 y 坐标
                                       Client_mid_position_x-17,  //终点 x 坐标
                                       Client_mid_position_y*2);  //终点 y 坐标
  /* 瞄准前哨战横线 */
  g0.grapic_data_struct[1] = draw_line("g01",  //图形名
                                       ADD,  //图形操作
                                       0,  //图层数，0~9
                                       YELLOW,  //颜色
                                       2,  //线条宽度
                                       Client_mid_position_x-16-10,  //起点 x 坐标
                                       Client_mid_position_y+95,  //起点 y 坐标
                                       Client_mid_position_x-16+10,  //终点 x 坐标
                                       Client_mid_position_y+95);  //终点 y 坐标
  /* 瞄准基地竖线 */
  g0.grapic_data_struct[2] = draw_line("g02",  //图形名
                                       ADD,  //图形操作
                                       0,  //图层数，0~9
                                       YELLOW,  //颜色
                                       3,  //线条宽度
                                       Client_mid_position_x+100,  //起点 x 坐标
                                       0,  //起点 y 坐标
                                       Client_mid_position_x+100,  //终点 x 坐标
                                       Client_mid_position_y*2);  //终点 y 坐标
  /* 瞄准基地横线 */
  g0.grapic_data_struct[3] = draw_line("g03",  //图形名
                                       ADD,  //图形操作
                                       0,  //图层数，0~9
                                       YELLOW,  //颜色
                                       3,  //线条宽度
                                       Client_mid_position_x+90,  //起点 x 坐标
                                       Client_mid_position_y,  //起点 y 坐标
                                       Client_mid_position_x+110,  //终点 x 坐标
                                       Client_mid_position_y);  //终点 y 坐标
//  /* 图传竖线 */
//  g0.grapic_data_struct[4] = draw_line("g04",  //图形名
//                                       ADD,  //图形操作
//                                       0,  //图层数，0~9
//                                       YELLOW,  //颜色
//                                       3,  //线条宽度
//                                       Client_mid_position_x-100,  //起点 x 坐标
//                                       0,  //起点 y 坐标
//                                       Client_mid_position_x-100,  //终点 x 坐标
//                                       Client_mid_position_y*2);  //终点 y 坐标
//  /* 图传横线 */
//  g0.grapic_data_struct[5] = draw_line("g05",  //图形名
//                                       ADD,  //图形操作
//                                       0,  //图层数，0~9
//                                       YELLOW,  //颜色
//                                       3,  //线条宽度
//                                       Client_mid_position_x-110,  //起点 x 坐标
//                                       Client_mid_position_y,  //起点 y 坐标
//                                       Client_mid_position_x-90,  //终点 x 坐标
//                                       Client_mid_position_y);  //终点 y 坐标

  client_send_seven_graphic(g0);
  
  cnt++;
  cnt %= 30;  //一秒有一次是ADD
}

/**
  * @brief  绘画任务1
  * @param  
  * @retval 
  */
void draw_task_1(void)
{
  static int cnt = 0;
  static int operate_tpye = ADD;
  
  if(cnt == 0)
  {
    operate_tpye = ADD;  //一秒有一次是ADD
  }
  else 
  {
    operate_tpye = MODIFY;
  }
  
  /* 超电条 */  //
  g1.grapic_data_struct[0] = draw_line("g10",  //图形名
                                       operate_tpye,  //图形操作
                                       1,  //图层数，0~9
                                       YELLOW,  //颜色
                                       20,  //线条宽度
                                       150,  //起点 x 坐标
                                       Client_mid_position_y + 250,  //起点 y 坐标
                                       450,  //终点 x 坐标
                                       Client_mid_position_y + 250);  //终点 y 坐标
  /* 发射机构yaw轴 */
  g1.grapic_data_struct[1] = draw_int("g11",  //图形名
                                      operate_tpye,  //图形操作
                                      1,  //图层数，0~9
                                      YELLOW,  //颜色
                                      30,  //字体大小
                                      3,  //线条宽度
                                      Client_mid_position_x*2 - 400,  //起点 x 坐标
                                      Client_mid_position_y + 300,  //起点 y 坐标
                                      launcher.info->yaw_angle);  //32 位整型数，int32_t
  /* 发射机构pitch轴 */
  g1.grapic_data_struct[2] = draw_int("g12",  //图形名
                                      operate_tpye,  //图形操作
                                      1,  //图层数，0~9
                                      YELLOW,  //颜色
                                      30,  //字体大小
                                      3,  //线条宽度
                                      Client_mid_position_x*2 - 400,  //起点 x 坐标
                                      Client_mid_position_y + 200,  //起点 y 坐标
                                      launcher.info->pitch_angle);  //32 位整型数，int32_t
  /* 自动发射标志 */  //绿正在发射 黄正常
  g1.grapic_data_struct[3] = draw_circle("g13",  //图形名
                                         operate_tpye,  //图形操作
                                         1,  //图层数，0~9
                                         YELLOW,  //颜色
                                         30,  //线条宽度
                                         Client_mid_position_x*2 - 170,  //圆心 x 坐标
                                         Client_mid_position_y+100,  //圆心 y 坐标
                                         15);  //半径
  /* 电磁铁开启标志 */  //绿吸上 FUCHSIA没吸上
  g1.grapic_data_struct[4] = draw_circle("g14",  //图形名
                                         operate_tpye,  //图形操作
                                         1,  //图层数，0~9
                                         YELLOW,  //颜色
                                         30,  //线条宽度
                                         Client_mid_position_x*2 - 170,  //圆心 x 坐标
                                         Client_mid_position_y,  //圆心 y 坐标
                                         15);  //半径
  
  client_send_seven_graphic(g1);
  
  cnt++;
  cnt %= 30;  //一秒有一次是ADD
  
}

/**************************************************第四层end**************************************************/


/**************************************************第三层begin**************************************************/

/******************************绘制任务begin******************************/

/******************************绘制任务end******************************/

/**************************************************第三层end**************************************************/

/**************************************************第二层begin**************************************************/

/******************************获取图像数据帧begin******************************/

/**
  * @brief  获取直线数据帧
  * @param  
  * @retval 图形数据结构体
  */
graphic_data_struct_t draw_line(char *name,  //图形名
	             uint8_t operate_tpye,  //图形操作
               uint8_t layer,  //图层数，0~9
               uint8_t color,  //颜色
               uint16_t width,  //线条宽度
               uint16_t start_x,  //起点 x 坐标
               uint16_t start_y,  //起点 y 坐标
               uint16_t end_x,  //终点 x 坐标
               uint16_t end_y)  //终点 y 坐标
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
  * @brief  获取矩形数据帧
  * @param  
  * @retval 图形数据结构体
  */
graphic_data_struct_t draw_rectangle(char *name,  //图形名
	             uint8_t operate_tpye,  //图形操作
               uint8_t layer,  //图层数，0~9
               uint8_t color,  //颜色DSFZa
               uint16_t width,  //线条宽度
               uint16_t start_x,  //起点 x 坐标
               uint16_t start_y,  //起点 y 坐标
               uint16_t end_x,  //对角顶点 x 坐标
               uint16_t end_y)  //对角顶点 y 坐标
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
  * @brief  获取整圆数据帧
  * @param  
  * @retval 图形数据结构体
  */
graphic_data_struct_t draw_circle(char *name,  //图形名
	             uint8_t operate_tpye,  //图形操作
               uint8_t layer,  //图层数，0~9
               uint8_t color,  //颜色
               uint16_t width,  //线条宽度
               uint16_t ciclemid_x,  //圆心 x 坐标
               uint16_t ciclemid_y,  //圆心 y 坐标
               uint16_t radius)  //半径
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
  * @brief  获取椭圆数据帧
  * @param  
  * @retval 图形数据结构体
  */
graphic_data_struct_t draw_ellipse(char *name,  //图形名
	             uint8_t operate_tpye,  //图形操作
               uint8_t layer,  //图层数，0~9
               uint8_t color,  //颜色
               uint16_t width,  //线条宽度
               uint16_t start_x,  //圆心 x 坐标
               uint16_t start_y,  //圆心 y 坐标
               uint16_t end_x,  //x 半轴长度
               uint16_t end_y)  //y 半轴长度
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
  * @brief  获取圆弧数据帧
  * @param  
  * @retval 图形数据结构体
  */
graphic_data_struct_t draw_arc(char *name,  //图形名
	             uint8_t operate_tpye,  //图形操作
               uint8_t layer,  //图层数，0~9
               uint8_t color,  //颜色
               uint16_t start_angle,  //起始角度
               uint16_t end_angle,  //终止角度
               uint16_t width,  //线条宽度
               uint16_t circlemin_x,  //圆心 x 坐标
               uint16_t circlemin_y,  //圆心 y 坐标
               uint16_t end_x,  //x 半轴长度
               uint16_t end_y)  //y 半轴长度
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
  * @brief  获取浮点数数据帧
  * @param  
  * @retval 图形数据结构体
  */
graphic_data_struct_t draw_float(char *name,  //图形名
	             uint8_t operate_tpye,  //图形操作
               uint8_t layer,  //图层数，0~9
               uint8_t color,  //颜色
               uint16_t size,  //字体大小
               uint16_t decimal,  //小数位有效个数
               uint16_t width,  //线条宽度
               uint16_t start_x,  //起点 x 坐标
               uint16_t start_y,  //起点 y 坐标
               int32_t num)  //乘以 1000 后，以 32 位整型数，int32_t
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
  * @brief  获取整型数数据帧
  * @param  
  * @retval 图形数据结构体
  */
graphic_data_struct_t draw_int(char *name,  //图形名
	             uint8_t operate_tpye,  //图形操作
               uint8_t layer,  //图层数，0~9
               uint8_t color,  //颜色
               uint16_t size,  //字体大小
               uint16_t width,  //线条宽度
               uint16_t start_x,  //起点 x 坐标
               uint16_t start_y,  //起点 y 坐标
               int32_t num)  //32 位整型数，int32_t
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
  * @brief  获取字符数据帧
  * @param  
  * @retval 图形数据结构体
  */
graphic_data_struct_t draw_char(char *name,  //图形名
	             uint8_t operate_tpye,  //图形操作
               uint8_t layer,  //图层数，0~9
               uint8_t color,  //颜色
               uint16_t size,  //字体大小
               uint16_t length,  //字符长度
               uint16_t width,  //线条宽度
               uint16_t start_x,  //起点 x 坐标
               uint16_t start_y)  //起点 y 坐标
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

/******************************获取图像数据帧end******************************/


/******************************发送帧数据begin******************************/

/**
	* @brief  发送绘制一个图形帧数据
  * @param  
  * @retval 
  */
uint8_t client_send_single_graphic(ext_client_custom_graphic_single_t data)
{
	frame_t frame;
	ext_student_interactive_header_data_t data_header;
	
	/* 帧头 */
	frame.frame_header.SOF = 0xA5;
	frame.frame_header.data_length = LEN_ID_draw_one_graphic;
	frame.frame_header.seq = 0;
	memcpy(client_tx_buf, &frame.frame_header, 4);
	Append_CRC8_Check_Num(client_tx_buf, 5);
	
	/* 命令码ID */
	frame.cmd_id = 0x301;
	memcpy(&client_tx_buf[5], (void*)&frame.cmd_id, 2);
	
	/* 数据段 */
	data_header.data_cmd_id = ID_draw_one_graphic;
	data_header.sender_ID = client_info.robot_id;
	data_header.receiver_ID = client_info.client_id;
	memcpy(&client_tx_buf[7], &data_header, 6);
	memcpy(&client_tx_buf[13], &data.grapic_data_struct, 15);
	
	/* 帧尾 */
	Append_CRC16_Check_Sum(client_tx_buf, 5 + 2 + LEN_ID_draw_one_graphic + 2);
	
	/* 发送 */
	return uart_send_data(client_tx_buf, 5 + 2 + LEN_ID_draw_one_graphic + 2);
}

/**
	* @brief  发送绘制二个图形帧数据
  * @param  
  * @retval 
  */
uint8_t client_send_double_graphic(ext_client_custom_graphic_double_t data)
{
	frame_t frame;
	ext_student_interactive_header_data_t data_header;
	
	/* 帧头 */
	frame.frame_header.SOF = 0xA5;
	frame.frame_header.data_length = LEN_ID_draw_two_graphic;
	frame.frame_header.seq = 0;
	memcpy(client_tx_buf, &frame.frame_header, 4);
	Append_CRC8_Check_Num(client_tx_buf, 5);
	
	/* 命令码ID */
	frame.cmd_id = 0x301;
	memcpy(&client_tx_buf[5], (void*)&frame.cmd_id, 2);
	
	/* 数据段 */
	data_header.data_cmd_id = ID_draw_two_graphic;
	data_header.sender_ID = client_info.robot_id;
	data_header.receiver_ID = client_info.client_id;
	memcpy(&client_tx_buf[7], &data_header, 6);
	memcpy(&client_tx_buf[13], data.grapic_data_struct, 15*2);
	
	/* 帧尾 */
	Append_CRC16_Check_Sum(client_tx_buf, 5 + 2 + LEN_ID_draw_two_graphic + 2);
	
	/* 发送 */
	return uart_send_data(client_tx_buf, 5 + 2 + LEN_ID_draw_two_graphic + 2);
}

/**
	* @brief  发送绘制五个图形帧数据
  * @param  
  * @retval 
  */
uint8_t client_send_five_graphic(ext_client_custom_graphic_five_t data)
{
	frame_t frame;
	ext_student_interactive_header_data_t data_header;
	
	/* 帧头 */
	frame.frame_header.SOF = 0xA5;
	frame.frame_header.data_length = LEN_ID_draw_five_graphic;
	frame.frame_header.seq = 0;
	memcpy(client_tx_buf, &frame.frame_header, 4);
	Append_CRC8_Check_Num(client_tx_buf, 5);
	
	/* 命令码ID */
	frame.cmd_id = 0x301;
	memcpy(&client_tx_buf[5], (void*)&frame.cmd_id, 2);
	
	/* 数据段 */
	data_header.data_cmd_id = ID_draw_five_graphic;
	data_header.sender_ID = client_info.robot_id;
	data_header.receiver_ID = client_info.client_id;
	memcpy(&client_tx_buf[7], &data_header, 6);
	memcpy(&client_tx_buf[13], data.grapic_data_struct, 15*5);
	
	/* 帧尾 */
	Append_CRC16_Check_Sum(client_tx_buf, 5 + 2 + LEN_ID_draw_five_graphic + 2);
	
	/* 发送 */
	return uart_send_data(client_tx_buf, 5 + 2 + LEN_ID_draw_five_graphic + 2);
}

/**
	* @brief  发送绘制七个图形帧数据
  * @param  
  * @retval 
  */
uint8_t client_send_seven_graphic(ext_client_custom_graphic_seven_t data)
{
	frame_t frame;
	ext_student_interactive_header_data_t data_header;
	
	/* 帧头 */
	frame.frame_header.SOF = 0xA5;
	frame.frame_header.data_length = LEN_ID_draw_seven_graphic;
	frame.frame_header.seq = 0;
	memcpy(client_tx_buf, &frame.frame_header, 4);
	Append_CRC8_Check_Num(client_tx_buf, 5);
	
	/* 命令码ID */
	frame.cmd_id = 0x301;
	memcpy(&client_tx_buf[5], (void*)&frame.cmd_id, 2);
	
	/* 数据段 */
	data_header.data_cmd_id = ID_draw_seven_graphic;
	data_header.sender_ID = client_info.robot_id;
	data_header.receiver_ID = client_info.client_id;
	memcpy(&client_tx_buf[7], &data_header, 6);
	memcpy(&client_tx_buf[13], data.grapic_data_struct, 15*7);
	
	/* 帧尾 */
	Append_CRC16_Check_Sum(client_tx_buf, 5 + 2 + LEN_ID_draw_seven_graphic + 2);
	
	/* 发送 */
	return uart_send_data(client_tx_buf, 5 + 2 + LEN_ID_draw_seven_graphic + 2);
}

/**
	* @brief  发送绘制字符帧数据
  * @param  
  * @retval 
  */
uint8_t client_send_char(ext_client_custom_character_t data)
{
	frame_t frame;
	ext_student_interactive_header_data_t data_header;
	
	/* 帧头 */
	frame.frame_header.SOF = 0xA5;
	frame.frame_header.data_length = LEN_ID_draw_char_graphic;
	frame.frame_header.seq = 0;
	memcpy(client_tx_buf, &frame.frame_header, 4);
	Append_CRC8_Check_Num(client_tx_buf, 5);
	
	/* 命令码ID */
	frame.cmd_id = 0x301;
	memcpy(&client_tx_buf[5], (void*)&frame.cmd_id, 2);
	
	/* 数据段 */
	data_header.data_cmd_id = ID_draw_char_graphic;
	data_header.sender_ID = client_info.robot_id;
	data_header.receiver_ID = client_info.client_id;
	memcpy(&client_tx_buf[7], &data_header, 6);
	memcpy(&client_tx_buf[13], &data.grapic_data_struct, 15);
	memcpy(&client_tx_buf[28], data.data, 30);
	
	/* 帧尾 */
	Append_CRC16_Check_Sum(client_tx_buf, 5 + 2 + LEN_ID_draw_char_graphic + 2);
	
	/* 发送 */
	return uart_send_data(client_tx_buf, 5 + 2 + LEN_ID_draw_char_graphic + 2);
}

/******************************发送帧数据end****************************************/

/**************************************************第二层end**************************************************/


/**************************************************第一层begin**************************************************/

/**
	* @brief  串口发送数据
  * @param  
  * @retval 
  */
uint8_t uart_send_data(uint8_t *txbuf, uint16_t length)
{
	return HAL_UART_Transmit_DMA(&UI_huart, txbuf, length);
}

/**************************************************第一层end**************************************************/
