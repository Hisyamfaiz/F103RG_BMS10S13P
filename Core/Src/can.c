/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "stdbool.h"
#include "math.h"
#include "string.h"
#include "stdlib.h"
#include <ctype.h>
//Variabel transmit and receive CAN
CAN_TxHeaderTypeDef		Tx_Header;
CAN_RxHeaderTypeDef 	Rx_Header;

uint8_t     Tx_data[8],
			Rx_data[8];
uint32_t	TxMailbox;
uint8_t		dataRTR = 0;

//Variabel bantu
uint16_t		handshake_recognition;
float 			Tmax;
extern uint8_t	cycle;
extern float 	Pack_Cap;
uint8_t			SOH_batt=90;

union Float_byte {
	float    m_float;
	uint8_t  m_bytes[sizeof(float)];};

union uint16_byte {
	uint16_t m_uint16_t;
	uint8_t  m_bytes[sizeof(float)];};

union Float_byte 	data;
union uint16_byte 	Batt_voltage,
				 	Batt_current,
					Batt_SOC,
					Batt_temp,
					Batt_capacity,
					Batt_SOH,
					Batt_cycle,
					over_charge_current,
					over_discharge_current,
					over_temp_charge,
					over_temp_discharge,
					under_voltage,
					limit_capacity,
					over_charge,
					start_balancing,
					max_chargevoltage,
					max_chargecurrent,
					max_voltage;

union uint16_byte vcell_15databyte[15];
/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_CAN1_2();

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void CANTX_BattParameter() {
	int delay_mailboxcheck;

	Batt_voltage.m_uint16_t	= VBATT*100;
	Batt_current.m_uint16_t	= fabs(IBATT)*10;
	Batt_SOC.m_uint16_t		= (int)Pack_SOC;

	Tmax=Suhu_T1;
	if(Tmax < Suhu_T2) Tmax = Suhu_T2;
	if(Tmax < Suhu_T3) Tmax = Suhu_T3;
	if(Tmax < Suhu_T4) Tmax = Suhu_T4;

	Batt_temp.m_uint16_t		= Tmax+40;
	Batt_capacity.m_uint16_t	= Pack_Cap*10;
	Batt_SOH.m_uint16_t			= SOH_batt;
	Batt_cycle.m_uint16_t		= cycle;
	max_voltage.m_uint16_t 		= 63;

	max_chargevoltage.m_uint16_t = VBATT*1.3;
	max_chargecurrent.m_uint16_t = VBATT-20;

	// *********************** GENERAL CAN COMMUNICATION ******************************
	// CAN ID transmit #1
	Tx_Header.IDE = CAN_ID_EXT;
	Tx_Header.ExtId = (0x0B0<<20|UNIQUE_Code);
	//CAN Data #1
	Tx_data[0] = Batt_voltage.m_bytes[0];
	Tx_data[1] = Batt_voltage.m_bytes[1];
	Tx_data[2] = Batt_current.m_bytes[0];
	Tx_data[3] = Batt_current.m_bytes[1];
	Tx_data[4] = Batt_SOC.m_bytes[0];
	Tx_data[5] = Batt_SOC.m_bytes[1];
	Tx_data[6] = Tmax+40;
	Tx_data[7] = 1;
	//		Tx_data[6] = Batt_temp.m_bytes[1];
	//		Tx_data[7] = Batt_temp.m_bytes[0];

	//CAN Tx message #1
	Tx_Header.DLC = 8;
	while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan)){
		if(delay_mailboxcheck > 1000){
			HAL_CAN_AbortTxRequest(&hcan, TxMailbox);
			delay_mailboxcheck = 0;
			break;
		}
		delay_mailboxcheck++;
	}

	if(HAL_CAN_AddTxMessage(&hcan, &Tx_Header, Tx_data, &TxMailbox)!= HAL_OK) {
		HAL_CAN_AbortTxRequest(&hcan, TxMailbox);
		return;
//		Error_Handler();
	}

}

void CANTX_BattProtection() {
	int delay_mailboxcheck;
	// CAN ID transmit #2
	Tx_Header.IDE = CAN_ID_EXT;
	Tx_Header.ExtId = (0x0B1<<20|UNIQUE_Code);
	//CAN Data #2
	Tx_data[0] = Batt_capacity.m_bytes[0];
	Tx_data[1] = Batt_capacity.m_bytes[1];
	Tx_data[2] = Batt_SOH.m_bytes[0];
	Tx_data[3] = Batt_SOH.m_bytes[1];
	Tx_data[4] = Batt_cycle.m_bytes[0];
	Tx_data[5] = Batt_cycle.m_bytes[1];
	Tx_data[6] = flag_trip_overcurrentdischarge&0x01;
	Tx_data[6] |= (flag_trip_overcurrentcharge&0x01)<<1;
	Tx_data[6] |= (flag_trip_shortcircuit&0x01)<<2;
	Tx_data[6] |= (flag_trip_overtemperature&0x01)<<3;
	Tx_data[6] |= (flag_trip_undertemperature&0x01)<<4;
	Tx_data[6] |= (flag_trip_overtemperature&0x01)<<5;
	Tx_data[6] |= (flag_trip_undertemperature&0x01)<<6;
	Tx_data[6] |=  (flag_trip_undervoltage&0x01)<<7;

	Tx_data[7] = (flag_trip_overvoltage&0x01);
	Tx_data[7] |= (flag_trip_SOCOverDischarge&0x01)<<1;
	Tx_data[7] |= (flag_trip_unbalance&0x01)<<2;
	Tx_data[7] |= (flag_trip_systemfailure&0x01)<<3;
	Tx_data[7] |= (charge_state&0x01)<<4;
	Tx_data[7] |= (discharge_state&0x01)<<5;
	Tx_data[7] |= (sleep_state&0x01)<<6;

	//CAN Tx message #2
	Tx_Header.DLC = 8;
	while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan)){
		if(delay_mailboxcheck > 1000){
			HAL_CAN_AbortTxRequest(&hcan, TxMailbox);
			delay_mailboxcheck = 0;
			break;
		}
		delay_mailboxcheck++;
	}

	if(HAL_CAN_AddTxMessage(&hcan, &Tx_Header, Tx_data, &TxMailbox)!= HAL_OK) {
		HAL_CAN_AbortTxRequest(&hcan, TxMailbox);
		return;
//		Error_Handler();
	}
}

void CANTX_ReportToCharger() {
	// *********************** CHARGING CAN COMMUNICATION ******************************
	int delay_mailboxcheck;
	// CAN ID transmit #1
	Tx_Header.IDE = CAN_ID_EXT;
	Tx_Header.ExtId = (0x0E0<<20|UNIQUE_Code);
	//CAN Data #1
	Tx_data[0] = max_chargevoltage.m_bytes[0];
	Tx_data[1] = max_chargevoltage.m_bytes[1];
	Tx_data[2] = max_chargecurrent.m_bytes[0];
	Tx_data[3] = max_chargecurrent.m_bytes[1];
	Tx_data[4] = charge_state;
	Tx_data[5] = 0;

	if(BMS_mode == 0) handshake_recognition = 0x55;
	else if(BMS_mode == 2) handshake_recognition = 0xAA;

	Tx_data[6] = handshake_recognition;
	Tx_data[7] = 0;

	//CAN Tx message #1
	Tx_Header.DLC = 8;
	while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan)){
		if(delay_mailboxcheck > 1000){
			HAL_CAN_AbortTxRequest(&hcan, TxMailbox);
			delay_mailboxcheck = 0;
			break;
		}
		delay_mailboxcheck++;
	}

	if(HAL_CAN_AddTxMessage(&hcan, &Tx_Header, Tx_data, &TxMailbox)!= HAL_OK) {
		HAL_CAN_AbortTxRequest(&hcan, TxMailbox);
		return;
//		Error_Handler();
	}
}

void CANTX_BattId() {
	int delay_mailboxcheck;
	// CAN ID transmit #2
	Tx_Header.IDE = CAN_ID_EXT;
	Tx_Header.ExtId = (0x0E1<<20|UNIQUE_Code);
	//CAN Data #1
	Tx_data[0] = 0;
	Tx_data[1] = 0;
	Tx_data[2] = 0x21;
	Tx_data[3] = 0x04;
	Tx_data[4] = UNIQUE_Code >> 16;
	Tx_data[5] = UNIQUE_Code >> 8;
	Tx_data[6] = UNIQUE_Code;
	Tx_data[7] = 0;

	//CAN Tx message #2
	Tx_Header.DLC = 8;
	while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan)){
		if(delay_mailboxcheck > 1000){
			HAL_CAN_AbortTxRequest(&hcan, TxMailbox);
			delay_mailboxcheck = 0;
			break;
		}
		delay_mailboxcheck++;
	}

	if(HAL_CAN_AddTxMessage(&hcan, &Tx_Header, Tx_data, &TxMailbox)!= HAL_OK) {
		HAL_CAN_AbortTxRequest(&hcan, TxMailbox);
		return;
//		Error_Handler();
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rx_Header, Rx_data) == HAL_OK) {
		if(Rx_Header.ExtId == 0x1B2){
			flag_start_shutdown=Rx_data[0]&0x01;
			BMS_mode=(Rx_data[0]>>1)&0x03;
			Clear_Trip_overcurrentdischarge=(Rx_data[0]>>3)&&0x01;
			Clear_Trip_undervoltage=(Rx_data[0]>>4)&&0x01;

			Active_time_last=HAL_GetTick();	//counter kapan terakhir menerima data
			Shutdown_time_last = HAL_GetTick();
		}

		else if(((Rx_Header.ExtId & 0xFFF00000) == 0x0E300000) && (flag_get_UNIQUECODE < 3) ) {
			flag_start_shutdown = 1;
			BMS_mode = 0;
			flag_get_UNIQUECODE++;
			Active_time_last=HAL_GetTick();	//counter kapan terakhir menerima data
			Shutdown_time_last = HAL_GetTick();
		}

		else if(Rx_Header.ExtId == (0x0E3<<20|UNIQUE_Code)){
			if(Rx_data[5] == 0x55){
				BMS_mode = 0;
				charge_state = 0;
			}
			else if(Rx_data[5] == 0xAA){
				BMS_mode = 2;
				charge_state = 1;
			}
			Active_time_last=HAL_GetTick();	//counter kapan terakhir menerima data
			Shutdown_time_last = HAL_GetTick();
		}

		else if(Rx_Header.RTR == 2){
			if(Rx_Header.StdId == 0x0B4) dataRTR = 4;
			else if(Rx_Header.StdId == 0x0B5) dataRTR = 5;
			else if(Rx_Header.StdId == 0x0B6) dataRTR = 6;
			else if(Rx_Header.StdId == 0x0B7) dataRTR = 7;
		}

		Rx_Header.ExtId = 0;
		Rx_Header.StdId = 0;
		memset(Rx_data, 0, 8*sizeof(Rx_data[0]));
	}
}


void BMS_CAN_Config()
{
	/* Configure the CAN Filter */
	CAN_FilterTypeDef  sFilterConfig;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) Error_Handler();

	/* Start the CAN peripheral */
	if (HAL_CAN_Start(&hcan) != HAL_OK) Error_Handler();

	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

	/* Configure Transmission process */
	Tx_Header.TransmitGlobalTime = DISABLE;
	Tx_Header.RTR = CAN_RTR_DATA;
	Tx_Header.IDE = CAN_ID_STD;
}
/* USER CODE END 1 */
