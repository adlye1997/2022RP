/**
  ******************************************************************************
  * @file           : drv_can.c\h
  * @brief          : 
  * @note           : finish 2022-2-12 12:10:08
  ******************************************************************************
  */

#include "drv_can.h"
#include "string.h"
#include "config.h"
#include "config_can.h"
#include "config_car.h"
#include "remote.h"

/*
drv_can_t example = 
{
	.hcan = &hcan,
	.rx_id = example,
};
*/

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

uint8_t can1_tx_buf[16];//CAN1���ͻ��棨0x200 0x1FF��
uint8_t can2_tx_buf[16];//CAN2���ͻ��棨0x200 0x1FF��
can_rx_info_t CAN_RxInfo;

/**
  * @brief  can���巢��
  * @param  
  * @retval 
  */
void CAN_send_all(void)
{
	#ifdef REMOTE_OFFLINE_CHECK
	/* ң����ʧ�� */
	if(rc.info->status == DEV_OFFLINE)
	{
		memset(can1_tx_buf,0,16);
		memset(can2_tx_buf,0,16);
	}
	#endif
	
	/* CAN���ͺϼ� */
	CAN_SendData(&hcan1,0x200,can1_tx_buf);
	CAN_SendData(&hcan1,0x1FF,&can1_tx_buf[8]);
	CAN_SendData(&hcan2,0x200,can2_tx_buf);
	CAN_SendData(&hcan2,0x1FF,&can2_tx_buf[8]);
	
	//����
	memset(can1_tx_buf,0,16);
	memset(can2_tx_buf,0,16);
}

/**
  * @brief  CAN���ͺ���
  * @param  
  * @retval 
  */
uint8_t CAN_SendData(CAN_HandleTypeDef *hcan, uint32_t stdId, uint8_t *dat)
{
	CAN_TxHeaderTypeDef pHeader;
	uint32_t txMailBox;
	
	//�ж�can��Ч��
	if((hcan->Instance != CAN1)&&(hcan->Instance != CAN2))
	{
		return HAL_ERROR;
	}
	
	//����֡ͷ
	pHeader.StdId = stdId;
	pHeader.IDE = CAN_ID_STD;
	pHeader.RTR = CAN_RTR_DATA;
	pHeader.DLC = 8;
	
	//�жϷ��ͳɹ����
	if(HAL_CAN_AddTxMessage(hcan, &pHeader, dat, &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	
	return HAL_OK;
}

/**
  * @brief  CAN1��ʼ��
  * @param  
  * @retval 
  */
void CAN1_Init(void)
{
	CAN_FilterTypeDef sFilterConfig;
	
	// ����CAN��ʶ���˲���
	CAN_Filter_ParamsInit(&sFilterConfig);
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	
	// ʹ�ܽ����ж�
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	// ����CAN1
	HAL_CAN_Start(&hcan1);
}

/**
  * @brief  CAN2��ʼ��
  * @param  
  * @retval 
  */
void CAN2_Init(void)
{
	CAN_FilterTypeDef sFilterConfig;
	
	// ����CAN��ʶ���˲���
	CAN_Filter_ParamsInit(&sFilterConfig);
	HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
	
	// ʹ�ܽ����ж�
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	// ����CAN2
	HAL_CAN_Start(&hcan2);
}

/**
  * @brief  ����CAN��ʶ���˲���
  * @param  
  * @retval 
  */
void CAN_Filter_ParamsInit(CAN_FilterTypeDef *sFilterConfig)
{
	sFilterConfig->FilterIdHigh = 0;						
	sFilterConfig->FilterIdLow = 0;							
	sFilterConfig->FilterMaskIdHigh = 0;					// ������
	sFilterConfig->FilterMaskIdLow = 0;						// ������
	sFilterConfig->FilterFIFOAssignment = CAN_FILTER_FIFO0;	// ������������FIFO0
	sFilterConfig->FilterBank = 0;							// ���ù�����0
	sFilterConfig->FilterMode = CAN_FILTERMODE_IDMASK;		// ��ʶ������ģʽ
	sFilterConfig->FilterScale = CAN_FILTERSCALE_32BIT;		// 32λ��
	sFilterConfig->FilterActivation = ENABLE;				// �����˲���
	sFilterConfig->SlaveStartFilterBank = 0;
}

/**
  * @brief  can�����жϣ���stm32f4xx_hal_can.c��������
  * @param  
  * @retval 
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_RxInfo.Header, CAN_RxInfo.Data);
  if(hcan == &hcan1)
  {
    CAN1_Get_Data(CAN_RxInfo.Header.StdId, CAN_RxInfo.Data);
  }
  else if(hcan == &hcan2)
  {
    CAN2_Get_Data(CAN_RxInfo.Header.StdId, CAN_RxInfo.Data);
  }
  else 
  {
    return;
  }
}
