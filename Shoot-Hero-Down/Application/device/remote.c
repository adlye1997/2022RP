/**
  ******************************************************************************
  * @file           : remote.c/h
  * @brief          : 
  * @note           : ����ʱ�䣺2022��1��11��17:24:50
	*                   �����ж�->�������ݡ��жϱ�־λ��true
  *                   ��������->���ж�ʱ->����жϱ�־λ���������
  *                   ��������->���ж�ʱ->�������̰�����Ϣ
  ******************************************************************************
  */

#include "remote.h"
#include "string.h"
#include "stdbool.h"
#include "rp_math.h"
#include "config.h"
#include "math_support.h"

bool rc_interrupt = false;                  //ң���������ж�

rc_base_info_t rc_base_info;
rc_info_t rc_info;
rc_t rc = 
{
  .base_info = &rc_base_info,
  .info = &rc_info,
};

/**
  * @brief  ң������ʼ��
  */
void rc_init(rc_t *rc)
{
	/* ������Ϣ��ʼ�� */
  rc_base_info_init(rc->base_info);
	/* ��Ϣ��ʼ�� */
	rc_info_init(rc->info);
  /* �жϱ�־��� */
  rc_interrupt_init(rc->base_info);
}

/**
  * @brief  ������Ϣ��ʼ��
  */
void rc_base_info_init(rc_base_info_t *info)
{
	/* ������Ϣ���� */
  memset(info,0,sizeof(rc_base_info_t));
	/* ������ť״̬��ʼ�� */
  info->s1.status = keep_R;
  info->s2.status = keep_R;
  info->thumbwheel.status = keep_R;
	/* ���ó���ʱ�� */
  key_board_cnt_max_set(info);
}

/**
  * @brief  ��������ʱ������
  */
void key_board_cnt_max_set(rc_base_info_t *info)
{
  info->mouse_btn_l.cnt_max = MOUSE_BTN_L_CNT_MAX;
  info->mouse_btn_r.cnt_max = MOUSE_BTN_R_CNT_MAX;
  info->Q.cnt_max = KEY_Q_CNT_MAX;
  info->W.cnt_max = KEY_W_CNT_MAX;
  info->E.cnt_max = KEY_E_CNT_MAX;
  info->R.cnt_max = KEY_R_CNT_MAX;
  info->A.cnt_max = KEY_A_CNT_MAX;
  info->S.cnt_max = KEY_S_CNT_MAX;
  info->D.cnt_max = KEY_D_CNT_MAX;
  info->F.cnt_max = KEY_F_CNT_MAX;
  info->G.cnt_max = KEY_G_CNT_MAX;
  info->Z.cnt_max = KEY_Z_CNT_MAX;
  info->X.cnt_max = KEY_X_CNT_MAX;
  info->C.cnt_max = KEY_C_CNT_MAX;
  info->V.cnt_max = KEY_V_CNT_MAX;
  info->B.cnt_max = KEY_B_CNT_MAX;
  info->Shift.cnt_max = KEY_SHIFT_CNT_MAX;
  info->Ctrl.cnt_max = KEY_CTRL_CNT_MAX;
}

/**
  * @brief  ��Ϣ��ʼ��
  */
void rc_info_init(rc_info_t *info)
{
	/* ����ʧ������ */
  info->offline_cnt = REMOTE_OFFLINE_CNT_MAX;
  /* ����״̬ */
  info->status = DEV_OFFLINE;
}

/**
  * @brief  �жϱ�־���
  */
void rc_interrupt_init(rc_base_info_t *info)
{
  rc_interrupt = false;                  //ң���������ж�
}

/**
  * @brief  ң���������жϸ���
  * @note   ң���������жϲ���ʱ����������ִ��
  */
void rc_interrupt_update(rc_t *rc)
{
	/* ����ٶȾ�ֵ�˲� */
	static int16_t mouse_x[REMOTE_SMOOTH_TIMES], mouse_y[REMOTE_SMOOTH_TIMES];
	static int16_t index = 0;
	index++;
	if(index == REMOTE_SMOOTH_TIMES)
	{
		index = 0;
	}
	rc->info->mouse_x -= (float)mouse_x[index] / (float)REMOTE_SMOOTH_TIMES;
	rc->info->mouse_y -= (float)mouse_y[index] / (float)REMOTE_SMOOTH_TIMES;
	mouse_x[index] = rc->base_info->mouse_vx;
	mouse_y[index] = rc->base_info->mouse_vy;
	rc->info->mouse_x += (float)mouse_x[index] / (float)REMOTE_SMOOTH_TIMES;
	rc->info->mouse_y += (float)mouse_y[index] / (float)REMOTE_SMOOTH_TIMES;
	
	/* �жϸ��� */
  rc_interrupt = true;
  rc->info->offline_cnt = 0;
  rc->info->status = DEV_ONLINE;
}

/**
  * @brief  ң�����δ�����
  */
void rc_tick_task(rc_t *rc)
{
  rc->info->offline_cnt ++;
  if(rc->info->offline_cnt > REMOTE_OFFLINE_CNT_MAX)
  {
    rc_init(rc);
  }
	else 
	{
		remote_soft_reset_check(rc);
	}
}

/**
  * @brief  ��������
  */
void rc_ctrl(rc_t *rc)
{
  if(rc_interrupt == true)
  {
    rc_interrupt = false;
		/* ң��������״̬���� */
    rc_switch_status_interrupt_update(rc->base_info);
		/* ң������ť״̬���� */
    rc_wheel_status_interrupt_update(rc->base_info);
		/* ���̰���״̬���� */
    all_key_board_status_interrupt_update(rc->base_info);
  }
  else 
  {
		/* ���̰���״̬���� */
    all_key_board_status_update(rc->base_info);
  }
	
	/* ����ٶȵ�ͨ�˲� */
	rc->info->mouse_x_K = lowpass(rc->info->mouse_x_K,rc->info->mouse_x,0.1);
	rc->info->mouse_y_K = lowpass(rc->info->mouse_y_K,rc->info->mouse_y,0.1);
}

/**
  * @brief  ң��������״̬�����жϲ�����
  */
void rc_switch_status_interrupt_update(rc_base_info_t *info)
{
  /* �󲦸��ж� */
  if(info->s1.value != info->s1.value_last)
  {
    switch(info->s1.value)
    {
      case 1:
        info->s1.status = up_R;
        break;
      case 3:
        info->s1.status = mid_R;
        break;
      case 2:
        info->s1.status = down_R;
        break;
      default:
        break;
    }
    info->s1.value_last = info->s1.value;
  }
  else 
  {
    info->s1.status = keep_R;
  }
  /* �Ҳ����ж� */
  if(info->s2.value != info->s2.value_last)
  {
    switch(info->s2.value)
    {
      case 1:
        info->s2.status = up_R;
        break;
      case 3:
        info->s2.status = mid_R;
        break;
      case 2:
        info->s2.status = down_R;
        break;
      default:
        break;
    }
    info->s2.value_last = info->s2.value;
  }
  else 
  {
    info->s2.status = keep_R;
  }
}

/**
  * @brief  ң������ť״̬�����жϲ�����
  */
void rc_wheel_status_interrupt_update(rc_base_info_t *info)
{
  if(abs(info->thumbwheel.value_last) < WHEEL_JUMP_VALUE)
  {
    if(info->thumbwheel.value > WHEEL_JUMP_VALUE)
    {
      info->thumbwheel.status = up_R;
    }
    else if(info->thumbwheel.value < -WHEEL_JUMP_VALUE)
    {
      info->thumbwheel.status = down_R;
    }
    else 
    {
      info->thumbwheel.status = keep_R;
    }
  }
  else 
  {
    info->thumbwheel.status = keep_R;
  }
  info->thumbwheel.value_last = info->thumbwheel.value;
}

/**
  * @brief  ң�������ղ����ж�ʱ���м��̰���״̬�жϲ�����
  */
void all_key_board_status_interrupt_update(rc_base_info_t *info)
{
  key_board_status_interrupt_update(&info->mouse_btn_l);
  key_board_status_interrupt_update(&info->mouse_btn_r);
  key_board_status_interrupt_update(&info->Q);
  key_board_status_interrupt_update(&info->W);
  key_board_status_interrupt_update(&info->E);
  key_board_status_interrupt_update(&info->R);
  key_board_status_interrupt_update(&info->A);
  key_board_status_interrupt_update(&info->S);
  key_board_status_interrupt_update(&info->D);
  key_board_status_interrupt_update(&info->F);
  key_board_status_interrupt_update(&info->G);
  key_board_status_interrupt_update(&info->Z);
  key_board_status_interrupt_update(&info->X);
  key_board_status_interrupt_update(&info->C);
  key_board_status_interrupt_update(&info->V);
  key_board_status_interrupt_update(&info->B);
  key_board_status_interrupt_update(&info->Shift);
  key_board_status_interrupt_update(&info->Ctrl);
}

/**
  * @brief  ң�������ղ����ж�ʱ���̰���״̬�жϲ�����
  */
void key_board_status_interrupt_update(key_board_info_t *key)
{
  switch(key->status)
  {
    case relax_K:
      if(key->value == 1)
      {
        key->status = down_K;
        key->cnt = 0;
      }
      break;
    case short_press_K:
      if(key->value == 0)
      {
        key->status = up_K;
				key->cnt = 0;
      }
      else if(key->value == 1)
      {
        key->cnt++;
        if(key->cnt >= key->cnt_max)
        {
          key->status = long_press_K;
					key->cnt = key->cnt_max;
        }
      }
      break;
    case long_press_K:
      if(key->value == 0)
      {
        key->status = up_K;
				key->cnt = 0;
      }
      break;
    default:
      break;
  }
}

/**
  * @brief  ���м��̰���״̬�жϲ�����
  */
void all_key_board_status_update(rc_base_info_t *info)
{
  key_board_status_update(&info->mouse_btn_l);
  key_board_status_update(&info->mouse_btn_r);
  key_board_status_update(&info->Q);
  key_board_status_update(&info->W);
  key_board_status_update(&info->E);
  key_board_status_update(&info->R);
  key_board_status_update(&info->A);
  key_board_status_update(&info->S);
  key_board_status_update(&info->D);
  key_board_status_update(&info->F);
  key_board_status_update(&info->G);
  key_board_status_update(&info->Z);
  key_board_status_update(&info->X);
  key_board_status_update(&info->C);
  key_board_status_update(&info->V);
  key_board_status_update(&info->B);
  key_board_status_update(&info->Shift);
  key_board_status_update(&info->Ctrl);
}

/**
  * @brief  ���̰���״̬�жϲ�����
  */
void key_board_status_update(key_board_info_t *key)
{
  switch(key->status)
  {
    case down_K:
      key->status = short_press_K;
      key->cnt++;
      break;
    case up_K:
      key->status = relax_K;
			key->cnt = 0;
      break;
    case short_press_K:
      key->cnt++;
      if(key->cnt >= key->cnt_max)
      {
        key->value = long_press_K;
				key->cnt = key->cnt_max;
      }
      break;
    default:
      break;
  }
}

void remote_soft_reset_check(rc_t *rc)
{
		if((rc->base_info->Shift.status != relax_K)&&(rc->base_info->F.status != relax_K)&&(rc->base_info->V.status != relax_K))
		{
			HAL_NVIC_SystemReset();
		}
}
