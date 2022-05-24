/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "ssd1306.h"
#include "stdio.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "LTC68042.h"
#include "EEPROM.h"
#include "Battery_Charge_Discharge.h"
#include "ctype.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int test;
char 	buff_lcd[20];	//variable buff lcd-oled
uint8_t	BATT_State;
uint8_t BATT_Start_Up = 0;
char	lower_UNIQUE_Code[5],
		UPPER_UNIQUE_Code[5];

int Sleep_tick=10000,
	Shutdown_tick=15000;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  BMS_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //Read voltage per-cell and total voltage
	  ltc6804_CS_RESET(ltc6804_CS_PIN);
	  read_voltage_percell();
	  read_sumvoltage();
	  ltc6804_CS_SET(ltc6804_CS_PIN);

	  //comparing cell voltage to get
	  unbalance_cell = get_balance_status(cellvoltage_float);

	  // Balancing Process
	  if(BMS_mode == 2 && IBATT < -0.1 && (VBATT > VBATT_BALANCE_START))     //arus charging 0.1 tidak perlu di balancing
	  {
		  LTC681x_balance_cell(balance_status);
	  }
	  else
	  {
		  balance_status = 0;
		  LTC681x_balance_cell(0);
	  }

	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

//	  //test turn off system
//	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10))
//		  HAL_GPIO_WritePin(BMS_SHUTDOWN_GPIO_Port, BMS_SHUTDOWN_Pin, 1);

	  BMS_ScreenMode_RUN();
	  HAL_IWDG_Refresh(&hiwdg);
	  test++;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void BMS_Init(void)
{
	itoa(UNIQUE_Code, lower_UNIQUE_Code, 16);
	int ii=0;
	while(ii<6){
		UPPER_UNIQUE_Code[ii] = toupper(lower_UNIQUE_Code[ii]);
		ii++;
	}

	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
	HAL_Delay(500);
	HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
	HAL_Delay(500);
	HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
	HAL_Delay(100);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
	HAL_Delay(100);
/*
	SSD1306_Init();
	HAL_Delay(500);
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	SSD1306_UpdateScreen();

	SSD1306_GotoXY (30,10);
	SSD1306_Puts ("GEN-I BMS", &Font_7x10, 1);
	SSD1306_GotoXY (40, 30);
	SSD1306_Puts ("10S13P", &Font_7x10, 1);
	SSD1306_UpdateScreen(); //display
	SSD1306_Fill (0);
*/

	ltc6804_GPIO_Config();
	ltc6804_SPIInit();

	set_adc(MD_FILTERED, DCP_DISABLED, CELL_CH_ALL, AUX_CH_ALL); //ADC Setting
	HAL_Delay(10);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) &adc_value, 5);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);

	BATT_State=STATE_STANDBY;
	Batt_Open_Mode();
	BATT_Start_Up = 1;
	flag_start_shutdown = 0;
	HAL_Delay(50);
	EEPROM_isDeviceReady(0xA0);
	BMS_CAN_Config();
	HAL_Delay(100);
}

void BMS_ScreenMode_RUN(void)
{
	if(flag_start_shutdown == 0)
	{
		/*
		SSD1306_Fill(SSD1306_COLOR_BLACK);
		sprintf(buff_lcd,"RUNNING");
		SSD1306_GotoXY(40,18);
		SSD1306_Puts(buff_lcd, &Font_7x10, SSD1306_COLOR_WHITE);

		sprintf(buff_lcd,"SLEEP_STATE");
		SSD1306_GotoXY(25,38);
		SSD1306_Puts(buff_lcd, &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
		*/

		OFFSET_SENSOR_ARUS=IBATT_for_offset_cal;
		Batt_Open_Mode();

		flag_trip_overtemperature=OFF;
		flag_trip_undertemperature=OFF;
		flag_trip_SOCOverDischarge=OFF;
		flag_trip_SOCOverCharge=OFF;			//di tiada kan..!
		flag_trip_undervoltage=OFF;
		flag_trip_overvoltage=OFF;
		flag_trip_overcurrentdischarge=OFF;
		flag_trip_overcurrentcharge=OFF;
		flag_trip_shortcircuit=OFF;
		flag_trip_systemfailure=OFF;
		flag_trip_unbalance=OFF;
		flag_get_UNIQUECODE=OFF;
		OFFSET_SENSOR_ARUS=IBATT_for_offset_cal;

		if(last_flag_start_shutdown==1) Shutdown_time_last = HAL_GetTick();

		// Automatic sleep after 30s without receive CAN
		Shutdown_time=HAL_GetTick();
		if(Shutdown_time-Shutdown_time_last>Shutdown_tick)
		{
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
			HAL_Delay(100);
			HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
			HAL_Delay(100);
			HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
			HAL_Delay(100);
			HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
			HAL_Delay(750);
			HAL_GPIO_WritePin(BMS_SHUTDOWN_GPIO_Port, BMS_SHUTDOWN_Pin, 1);
		}

		last_flag_start_shutdown = 0;
	}
	else
	{
/*
		SSD1306_Fill(SSD1306_COLOR_BLACK);

		if(BATT_State==STATE_CHARGE)
		{
			sprintf(buff_lcd,"RUN (C) - %05s", UPPER_UNIQUE_Code);
			SSD1306_GotoXY(0,0);
			SSD1306_Puts(buff_lcd, &Font_7x10, SSD1306_COLOR_WHITE);
		}
		else if(BATT_State==STATE_DISCHARGE)
		{
			sprintf(buff_lcd,"RUN (D) - %05s", UPPER_UNIQUE_Code);
			SSD1306_GotoXY(0,0);
			SSD1306_Puts(buff_lcd, &Font_7x10, SSD1306_COLOR_WHITE);
		}
		else if(BATT_State==STATE_FULL_CHARGE_DISCHARGE)
		{
			sprintf(buff_lcd,"RUN (C/D) - %05s", UPPER_UNIQUE_Code);
			SSD1306_GotoXY(0,0);
			SSD1306_Puts(buff_lcd, &Font_7x10, SSD1306_COLOR_WHITE);
		}
		else if(BATT_State==STATE_STANDBY)
		{
			sprintf(buff_lcd,"RUN (Open) - %05s", UPPER_UNIQUE_Code);
			SSD1306_GotoXY(0,0);
			SSD1306_Puts(buff_lcd, &Font_7x10, SSD1306_COLOR_WHITE);
		}

		sprintf(buff_lcd,"V=%6.2f I=%6.2f",VBATT, IBATT);
		SSD1306_GotoXY(0,10);
		SSD1306_Puts(buff_lcd, &Font_7x10, SSD1306_COLOR_WHITE);
		sprintf(buff_lcd,"T=%3.0f|%3.0f|%3.0f|%3.0f", Suhu_T1, Suhu_T2, Suhu_T3, Suhu_T4);
		SSD1306_GotoXY(0,20);
		SSD1306_Puts(buff_lcd, &Font_7x10, SSD1306_COLOR_WHITE);
		sprintf(buff_lcd,"C=%5.1f%%--%5.1f%%",Pack_SOC,SOC_manipulasi);
		SSD1306_GotoXY(0,30);
		SSD1306_Puts(buff_lcd, &Font_7x10, SSD1306_COLOR_WHITE);
		sprintf(buff_lcd,"B=%5d, %4.2f-%4.2f",unbalance_cell, persen_imbalance, OFFSET_SENSOR_ARUS);
		SSD1306_GotoXY(0,40);
		SSD1306_Puts(buff_lcd, &Font_7x10, SSD1306_COLOR_WHITE);

		sprintf(buff_lcd,"%d-%d--%4.2f| %5.0f",fault_code,last_fault_code,Isc, AH_Total);
		SSD1306_GotoXY(0,50);
		SSD1306_Puts(buff_lcd, &Font_7x10, SSD1306_COLOR_WHITE);

		SSD1306_UpdateScreen();
*/
		if(BMS_mode==0) Batt_Open_Mode();
		else if(BMS_mode==1) Batt_Discharge_Mode();
		else if(BMS_mode==2) Batt_Charge_Mode();
		else if(BMS_mode==3) Batt_Full_CD_Mode();

	//  Automatically sleep after 30s without receive CAN
		Sleep_time=HAL_GetTick();
		if(Sleep_time-Active_time_last>Sleep_tick)
		{
			BMS_mode=0;
			flag_start_shutdown=0;
		}
		last_flag_start_shutdown = 1;
	}
	HAL_Delay(1);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

