/**
  ******************************************************************************
  * @file           : car.c/h
  * @brief          : 
  * @note           : finish 2022-2-19 12:51:56
	*                   ��ϣ�Ӣ�۲����ֲ�V1.2��
  ******************************************************************************
  */

#include "car.h"
#include "config_car.h"
#include "config.h"
#include "remote.h"
#include "gimbal.h"
#include "led.h"
#include "imu.h"
#include "chassis.h"
#include "launcher.h"

car_config_t car_config;
car_t car = 
{
	.config = &car_config,
};

/* ������־ */
bool car_init_Y_O_N = false;            //������ʼ���Ƿ���ɱ�־
bool car_mode_change = false;           //����ģʽ�л���־

/* �������� */
bool chassis_cycle_off = false;         //��С����
bool chassis_cycle_on = false;          //��С����
bool chassis_cycle_change = false;      //С�����л�

/* ��̨���� */
bool gimbal_to_check = false;           //��̨�л�checkģʽ
bool gimbal_out_check = false;          //��̨�˳�checkģʽ
bool gimbal_l_90 = false;               //��̨��ת90��
bool gimbal_l_shoot = false;            //��̨������ת����
bool gimbal_r_90 = false;               //��̨��ת90��
bool gimbal_r_shoot = false;            //��̨������ת����
bool gimbal_r_180 = false;              //��̨��ͷ
bool gimbal_u_shoot = false;            //��̨����̧ͷ����
bool gimbal_d_shoot = false;            //��̨�����ͷ����
bool launcher_auto = false;             //�Զ����

/* ����������� */
bool Launcher_Shoot = false;            //�����������
bool Launcher_Relax = false;            //�����������
bool Launcher_Ready = false;            //�������׼��
bool Launcher_Return = false;           //��������ָ�
bool Launcher_Init = false;             //���������ʼ��
bool dial_reload = false;               //���̲���
bool dial_unload = false;               //�����˵�
bool launcher_u = false;
bool launcher_d = false;
bool launcher_l = false;
bool launcher_r = false;
bool launcher_u_s = false;
bool launcher_d_s = false;
bool launcher_l_s = false;
bool launcher_r_s = false;

/* �������� */
bool auto_on = false;                   //������
bool auto_off = false;                  //������

/* ����ϵͳ���� */
bool judge_close = false;               //�ز���ϵͳ����

extern bool imu_init_Y_O_N;
extern bool gimbal_init_Y_O_N;
extern bool gimbal_machine_debug_on;
extern bool gimbal_machine_debug_off;

/**
  * @brief  ������ʼ��
  * @param  
  * @retval 
  */
void car_init(car_t *car)
{
  car->move_mode_status = offline_CAR;
	car->move_mode_commond = offline_CAR;
  car->ctrl_mode = RC_CAR;
  car_mode_change = false;
	
	/* �������������ʼ�� */
  car_commond_init(car);
}

void car_commond_init(car_t *car) 
{//�������������ʼ��
  chassis_cycle_off = false;
  chassis_cycle_on = false;
  chassis_cycle_change = false;
	gimbal_to_check = false;
  gimbal_l_90 = false;
  gimbal_l_shoot = false;
  gimbal_r_90 = false;
  gimbal_r_shoot = false;
  gimbal_r_180 = false;
	gimbal_u_shoot = false;
	gimbal_d_shoot = false;
	gimbal_machine_debug_on = false;
	gimbal_machine_debug_off = false;
  Launcher_Shoot = false;
  Launcher_Relax = false;
	Launcher_Return = false;
  Launcher_Ready = false;
  dial_reload = false;
	dial_unload = false;
  auto_on = false;
	auto_off = false;
  judge_close = false;
	launcher_u = false;
	launcher_d = false;
	launcher_l = false;
	launcher_r = false;
	launcher_u_s = false;
	launcher_d_s = false;
	launcher_l_s = false;
	launcher_r_s = false;
  Launcher_Init = false;
}

/**
  * @brief  ������������
  * @param  
  * @retval 
  */
void car_ctrl(car_t *car) 
{
  car_mode_commond_update(car);
  
	/* ����ģʽ״̬���� */
  car_mode_status_update(car);
}

void car_mode_commond_update(car_t *car) 
{
	/* ң����s2״̬ȷ�� */
  RC_s2_status_check(car);
	
  /* ����ģʽ���� */
  switch (car->ctrl_mode) //�жϿ���ģʽ
  {
    /* ң���� */
    case RC_CAR:
      RC_status_scan(car);
      break;
		
    /* ���� */
    case KEY_CAR:
			debug_special_scan(car);
      KEY_status_scan(car);
      break;
		
    default:
      break;
  }
}

/**
  * @brief  ң����s2����״̬ȷ��
  * @param  
  * @retval 
  */
void RC_s2_status_check(car_t *car)
{
  switch(rc.base_info->s2.status)
  {
    case up_R:
      car->move_mode_commond = high_shoot_CAR;
      car->ctrl_mode = RC_CAR;
      break;
    case mid_R:
			car->move_mode_commond = machine_CAR;
      car->ctrl_mode = KEY_CAR;
      break;
    case down_R:
      car->move_mode_commond = machine_CAR;
      car->ctrl_mode = RC_CAR;
      break;
    default:
      break;
  }
}

/**
  * @brief  ң��������ɨ��
  * @param  
  * @retval 2022-5-8 13:29:57
  */
void RC_status_scan(car_t *car)
{
	/* ң����s1״̬ȷ�� */
  RC_s1_status_check(car);
	
	/* ң������ť״̬ȷ�� */
  RC_thumbwheel_status_check(car);
}

/**
  * @brief  ң����s1״̬ȷ��
  * @param  
  * @retval 2022-5-8 13:53:21
  */
void RC_s1_status_check(car_t *car)
{
  if(car->move_mode_status == high_shoot_CAR)
  {
    switch(rc.base_info->s1.status)
    {
      case up_R:
        Launcher_Shoot = true;
        break;
      case mid_R:
        Launcher_Relax = true;
        break;
      case down_R:
        Launcher_Return = true;
        break;
      default:
        break;
    }
    if(rc.base_info->s1.value == 3)
    {
      Launcher_Relax = true;
    }
  }
}

/**
  * @brief  ң������ť״̬ȷ��
  * @param  
  * @retval 2022-5-8 13:53:28
  */
void RC_thumbwheel_status_check(car_t *car)
{
	static uint8_t flag = 0;
  if(car->move_mode_status == high_shoot_CAR)
  {
    switch(rc.base_info->thumbwheel.status)
    {
      case up_R:
        if(flag == 0)
        {
          dial_unload = true;
          flag = 1;
        }
        break;
      case mid_R:
        flag = 0;
        break;
      case down_R:
        if(flag == 0)
        {
          dial_reload = true;
          flag = 1;
        }
        break;
      default:
        break;
    }
    if(rc.base_info->thumbwheel.value == 0)
    {
      flag = 0;
    }
  }
}

void car_mode_status_update(car_t *car)
{
  /* �ж�������ʼ���Ƿ���� */
  car_init_judge();
	
  /* ��������״̬ */
  if(rc.info->status == DEV_OFFLINE)  //ң����ʧ��
  {
    car->move_mode_status = offline_CAR;
		car_init_Y_O_N = false;
		gimbal_init_Y_O_N = false;
    car_commond_init(car);
  }
  else if(car_init_Y_O_N == false)  //������ʼ��δ���
  {
    car->move_mode_status = init_CAR;
    car_commond_init(car);
  }
  else 
  {
    car->move_mode_status = car->move_mode_commond;
  }
}

/**
  * @brief  ��ʼ���ж�
  */
void car_init_judge(void)
{
  if(gimbal.work->init_Y_O_N == 1)
  {
    car_init_Y_O_N = true;
  }
}

/**
  * @brief  debug���ⰴ��ɨ��
  * @param  
  * @retval 2022-5-8 13:54:19
  */
void debug_special_scan(car_t *car)
{
  switch(rc.base_info->thumbwheel.status)
  {
    case up_R:
      Launcher_Init = true;
      break;
    case mid_R:
      break;
    case down_R:
      break;
    default:
      break;
  }
  switch(rc.base_info->s1.value)
  {
    case 1:
      HAL_GPIO_WritePin(REBOOT_RELAY_SET_GPIO_Port, REBOOT_RELAY_SET_Pin, GPIO_PIN_SET);
      break;
    case 3:
      break;
    case 2:
      HAL_GPIO_WritePin(REBOOT_RELAY_SET_GPIO_Port, REBOOT_RELAY_SET_Pin, GPIO_PIN_RESET);
      break;
    default:
      break;
  }
}

/** 
  * @brief  ���̰���ɨ��
  * @note   ����������״̬����
  */
void KEY_status_scan(car_t *car)
{
  KEY_mouse_l_status_check(car);
  KEY_mouse_r_status_check(car);
  KEY_Q_status_check(car);
  KEY_W_status_check(car);
  KEY_E_status_check(car);
  KEY_R_status_check(car);
  KEY_A_status_check(car);
  KEY_S_status_check(car);
  KEY_D_status_check(car);
  KEY_F_status_check(car);
  KEY_G_status_check(car);
  KEY_Z_status_check(car);
  KEY_X_status_check(car);
  KEY_C_status_check(car);
  KEY_V_status_check(car);
  KEY_B_status_check(car);
  KEY_Shift_status_check(car);
  KEY_Ctrl_status_check(car);
}

void KEY_mouse_l_status_check(car_t *car)
{
  switch(rc.base_info->mouse_btn_l.status)
  {
    case relax_K:
      Launcher_Relax = true;
      break;
    case down_K:
      Launcher_Shoot = true;
      break;
    case up_K:
      break;
    case short_press_K:
      break;
    case long_press_K:
      break;
    default:
      break;
  }
}

void KEY_mouse_r_status_check(car_t *car)
{
  switch(rc.base_info->mouse_btn_r.status)
  {
    case down_K:
			if(gimbal.info->shoot_watch_flag == 1)
			{
				gimbal.info->shoot_watch_flag = 0;
			}
			else if(gimbal.info->shoot_watch_flag == 0)
			{
				gimbal.info->shoot_watch_flag = 1;
			}
      break;
    case up_K:
      break;
    case short_press_K:
      break;
    case long_press_K:
      break;
    default:
      break;
  }
}

void KEY_Q_status_check(car_t *car)
{
  switch(rc.base_info->Q.status)
  {
    case down_K:
			if(car->move_mode_status == high_shoot_CAR)
      {
        gimbal_to_check = true;
      }
      break;
    case up_K:
      break;
    case short_press_K:
      break;
    case long_press_K:
      break;
    default:
      break;
  }
}

void KEY_W_status_check(car_t *car)
{
  switch(rc.base_info->W.status)
  {
    case down_K:
			if(car->move_mode_status == high_shoot_CAR)
      {
        launcher_u = true;
      }
      break;
    case up_K:
      break;
    case short_press_K:
      break;
    case long_press_K:
			if(car->move_mode_status == high_shoot_CAR)
      {
        launcher_u_s = true;
      }
      break;
    default:
      break;
  }
}

void KEY_E_status_check(car_t *car)
{
  switch(rc.base_info->E.status)
  {
    case down_K:
			if(car->move_mode_status == high_shoot_CAR)
      {
        gimbal_out_check = true;
      }
      break;
    case up_K:
      break;
    case short_press_K:
      break;
    case long_press_K:
      break;
    default:
      break;
  }
}

void KEY_R_status_check(car_t *car)
{
  switch(rc.base_info->R.status)
  {
    case down_K:
			if(car->move_mode_status == high_shoot_CAR)
      {
        dial_unload = true;
      }
      break;
    case up_K:
      break;
    case short_press_K:
      break;
    case long_press_K:
      break;
    default:
      break;
  }
}

void KEY_A_status_check(car_t *car)
{
  switch(rc.base_info->A.status)
  {
    case down_K:
			if(car->move_mode_status == high_shoot_CAR)
      {
        launcher_l = true;
      }
      break;
    case up_K:
      break;
    case short_press_K:
      break;
    case long_press_K:
			if(car->move_mode_status == high_shoot_CAR)
      {
        launcher_l_s = true;
      }
      break;
    default:
      break;
  }
}

void KEY_S_status_check(car_t *car)
{
  switch(rc.base_info->S.status)
  {
    case down_K:
			if(car->move_mode_status == high_shoot_CAR)
      {
        launcher_d = true;
      }
      break;
    case up_K:
      break;
    case short_press_K:
      break;
    case long_press_K:
			if(car->move_mode_status == high_shoot_CAR)
      {
        launcher_d_s = true;
      }
      break;
    default:
      break;
  }
}

void KEY_D_status_check(car_t *car)
{
  switch(rc.base_info->D.status)
  {
    case down_K:
			if(car->move_mode_status == high_shoot_CAR)
      {
        launcher_r = true;
      }
      break;
    case up_K:
      break;
    case short_press_K:
      break;
    case long_press_K:
			if(car->move_mode_status == high_shoot_CAR)
      {
        launcher_r_s = true;
      }
      break;
    default:
      break;
  }
}

void KEY_F_status_check(car_t *car)
{
  switch(rc.base_info->F.status)
  {
    case down_K:
      launcher_auto = true;
      break;
    case up_K:
      break;
    case short_press_K:
      break;
    case long_press_K:
      break;
    default:
      break;
  }
}

void KEY_G_status_check(car_t *car)
{
  switch(rc.base_info->G.status)
  {
    case down_K:
      car->move_mode_commond = high_shoot_CAR;
      break;
    case up_K:
      break;
    case short_press_K:
      break;
    case long_press_K:
      break;
    default:
      break;
  }
}

void KEY_Z_status_check(car_t *car)
{
  switch(rc.base_info->Z.status)
  {
		case relax_K:
			break;
    case down_K:
      break;
    case up_K:
      break;
    case short_press_K:
      break;
    case long_press_K:
      break;
    default:
      break;
  }
}

void KEY_X_status_check(car_t *car)
{
  switch(rc.base_info->X.status)
  {
		case relax_K:
			break;
    case down_K:
      break;
    case up_K:
      break;
    case short_press_K:
      break;
    case long_press_K:
      break;
    default:
      break;
  }
}

void KEY_C_status_check(car_t *car)
{
  switch(rc.base_info->C.status)
  {
		case relax_K:
			break;
    case down_K:
      break;
    case up_K:
      break;
    case short_press_K:
      break;
    case long_press_K:
      break;
    default:
      break;
  }
}

void KEY_V_status_check(car_t *car)
{
  switch(rc.base_info->V.status)
  {
    case down_K:
      break;
    case up_K:
      break;
    case short_press_K:
      break;
    case long_press_K:
      break;
    default:
      break;
  }
}

void KEY_B_status_check(car_t *car)
{
  switch(rc.base_info->B.status)
  {
		case relax_K:
			break;
    case down_K:
			Launcher_Return = true;
      break;
    case up_K:
      Launcher_Relax = true;
      break;
    case short_press_K:
      break;
    case long_press_K:
      break;
    default:
      break;
  }
}

void KEY_Shift_status_check(car_t *car)
{
  switch(rc.base_info->Shift.status)
  {
    case down_K:
      break;
    case up_K:
      break;
    case short_press_K:
      break;
    case long_press_K:
      break;
    default:
      break;
  }
}

void KEY_Ctrl_status_check(car_t *car)
{
  switch(rc.base_info->Ctrl.status)
  {
    case down_K:
      car->move_mode_commond = machine_CAR;
      break;
    case up_K:
      car->move_mode_commond = machine_CAR;
      break;
    case short_press_K:
      car->move_mode_commond = machine_CAR;
      break;
    case long_press_K:
      car->move_mode_commond = machine_CAR;
      break;
    default:
      break;
  }
}
