/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//Pendefinisian kode unik baterai
//#define	UNIQUE_Code 0x00A21
#define	UNIQUE_Code 0x00002

#define ON 1
#define OFF 0
#define YES 1
#define NO 0

// Pendefinisan State BMS
#define STATE_STANDBY 0
#define STATE_CHARGE 1
#define STATE_DISCHARGE 2
#define STATE_FULL_CHARGE_DISCHARGE 3

#define LED_Toggle HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin)
#define BUZZ_Toggle HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin)
#define BUZZ_Write(X) HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, X)
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUZZER_Pin GPIO_PIN_13
#define BUZZER_GPIO_Port GPIOC
#define BMS_SHUTDOWN_Pin GPIO_PIN_1
#define BMS_SHUTDOWN_GPIO_Port GPIOC
#define LOWCELL_TEMP_Pin GPIO_PIN_1
#define LOWCELL_TEMP_GPIO_Port GPIOA
#define HIGHCELL_TEMP_Pin GPIO_PIN_2
#define HIGHCELL_TEMP_GPIO_Port GPIOA
#define MOSFET_TEMP_Pin GPIO_PIN_3
#define MOSFET_TEMP_GPIO_Port GPIOA
#define CURRENT_SENSE_TEMP_Pin GPIO_PIN_4
#define CURRENT_SENSE_TEMP_GPIO_Port GPIOA
#define CURRENT_SENSOR_Pin GPIO_PIN_5
#define CURRENT_SENSOR_GPIO_Port GPIOA
#define SPARE_TEMP_Pin GPIO_PIN_6
#define SPARE_TEMP_GPIO_Port GPIOA
#define EEPROM_WP_Pin GPIO_PIN_5
#define EEPROM_WP_GPIO_Port GPIOC
#define MIDCELL_TEMP_Pin GPIO_PIN_0
#define MIDCELL_TEMP_GPIO_Port GPIOB
#define EEPROM_SCL_Pin GPIO_PIN_10
#define EEPROM_SCL_GPIO_Port GPIOB
#define EEPROM_SDA_Pin GPIO_PIN_11
#define EEPROM_SDA_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_6
#define LED_GPIO_Port GPIOC
#define GATE_MOS_Pin GPIO_PIN_8
#define GATE_MOS_GPIO_Port GPIOA
#define CSBI_Pin GPIO_PIN_12
#define CSBI_GPIO_Port GPIOC
#define SCK_Pin GPIO_PIN_3
#define SCK_GPIO_Port GPIOB
#define SDO_Pin GPIO_PIN_4
#define SDO_GPIO_Port GPIOB
#define SDI_Pin GPIO_PIN_5
#define SDI_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_6
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_7
#define OLED_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

// Deklarasi Fungsi start-up dan tampilan
void BMS_Init(void);
void BMS_ScreenMode_Standby(void);
void BMS_ScreenMode_RUN(void);
void StartUp_Buzzer(void);
void ShutDown_Buzzer(void);
void isCell_UnderVoltage(float Cell_Voltage_10data[10]);
void isCell_OverVoltage(float Cell_Voltage_10data[10]);
void Reset_FlagProtection(void);

// Deklarasi variabel komunikasi LTC6804
uint8_t		CFGR4,
			CFGR5;
uint8_t		rd_config[1][8];
uint16_t	cell_voltage[3][12];
uint8_t		RDCVA[2], PEC[2];

// Variabel Read ADC
uint16_t 	adc_value[7];
float		i_datadigi;
float		VBATT,
			IBATT;
float		Isc,
			Vsc;

float		OFFSET_SENSOR_ARUS,
			IBATT_for_offset_cal;

float		Res_T1,
			Res_T2,
			Res_T3,
			Res_T4,
			Res_T5,
			Res_T6;

float		Suhu_T1,
			Suhu_T2,
			Suhu_T3,
			Suhu_T4,
			MOS_Temp,
			CurrentSensor_Temp;

// Variabel Bantu State BMS
uint8_t 	flag_start_shutdown,
			last_flag_start_shutdown,
			BMS_mode;
uint8_t		charge_state,
			discharge_state,
			sleep_state;

uint16_t 	balance_status;
uint16_t	unbalance_cell;
float 		persen_imbalance;

int 		Sleep_time,
			Active_time_last,
			Shutdown_time,
			Shutdown_time_last;

uint8_t		flag_get_UNIQUECODE;
uint8_t		state;

// Variabel kapasitas dan SOC baterai
float		batas_atas,batas_bawah,
			SOC_manipulasi,
			grad,
			constanta;
float 		AH_Consumption,
			AH_Total;

// PROTECTION VARIABLE
uint8_t	fault_code,
		last_fault_code;
float 	T_I_Over_trip,
		T_I_Over_trip_cycle;
int		test_tim2;

uint8_t flag_trip_overtemperature,
		flag_trip_undertemperature,
		flag_trip_SOCOverDischarge,
		flag_trip_SOCOverCharge,			//di tiada kan..!
		flag_trip_undervoltage,
		flag_trip_overvoltage,
		flag_trip_overcurrentdischarge,
		flag_trip_overcurrentcharge,
		flag_trip_shortcircuit,
		flag_trip_systemfailure,
		flag_trip_unbalance,
		flag_trip_cellundervoltage,
		flag_trip_cellovervoltage;

uint8_t Clear_Trip_undervoltage,
		Clear_Trip_overcurrentdischarge;
uint8_t Cell_OverVoltage,
		Cell_UnderVoltage;

//variable baterai dan SOC
float	Pack_SOC,
		SOC_manipulasi,
		Delta_VCell,
		Bat_Pow_Out;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
