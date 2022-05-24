/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "LTC68042.h"
#include "can.h"
#include "math.h"
#include "Battery_Charge_Discharge.h"

#define maxdata 200				//jumlah sampling rata-rata ADC
#define interval_hitungsuhu 200	//delay update nilai sensor suhu = 10 x periode timer 2

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
uint16_t testtim2, testtim3;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern CAN_HandleTypeDef hcan;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */
// Deklarasi fungsi proteksi
void Batt_Protection_when_discharge(void);
void Batt_Protection_when_charge(void);
void Batt_Protection_when_chargedischarge(void);

// Variabel bantu
uint16_t	hitung_suhu;
uint32_t	sum_datadigi_suhu1, sum_datadigi_suhu2, sum_datadigi_suhu3,sum_datadigi_suhu4;
uint16_t	datadigi_suhu1, datadigi_suhu2, datadigi_suhu3, datadigi_suhu4;
uint8_t 	Tick_BattId, Tick_33ms;
uint16_t	test;


//Variabel Kalkulasi sensor arus
int			sumI, i;
uint16_t	i_arrdata[maxdata];
float		OFFSET_SENSOR_ARUS,IBATT_for_offset_cal;
float		sum_current;

// Parameter tambahan baterai
float 		AH_Consumption, AH_Total=0;
uint16_t 	time_soc;
uint32_t 	cek_CC=0;
float 		Pack_Cap = 45;
uint8_t		BATT_State;
uint8_t 	BATT_Start_Up;

// Variable setting proteksi
float 	I_Over_Set=50,
		I_Over_Set_Charge=8,
		Temp_Over_Set=55,
		Temp_Under_Set=10,
		SOC_Under_Set=10,
		SOC_Over_Set=120,
		V_Under_Set=29,
		V_Over_Set=45,
		Persen_Imbalance_Set=20;

//Variable bantu proteksi
float	TMS=0.5;
float 	TMS_I_Over=120;
float 	T_Under_trip,
		T_trip_cycle;
uint8_t Clear_Trip_undervoltage,
		Clear_Trip_overcurrentdischarge;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  hitung_suhu++;
  test_tim2++;

  //Moving Average
  sumI=sumI-i_arrdata[i];		//menghapus sample data pertama/data lama
  i_arrdata[i]=adc_value[0];	//save nilai ADC dari variable DMA
  sumI=sumI+i_arrdata[i];		//menambahkan sample data paling baru
  i_datadigi=sumI/maxdata;	//menghitung rata-rata

  sum_datadigi_suhu1+=adc_value[1];
  sum_datadigi_suhu2+=adc_value[2];
  sum_datadigi_suhu3+=adc_value[3];
  sum_datadigi_suhu4+=adc_value[4];
  // *************PROSES Konversi dari DATA ADC ke Data Real *******************************/////

  VBATT = sum_voltage;
  if(VBATT<0) VBATT=-1;

  if(UNIQUE_Code == 0x00A21) {
	  IBATT = 0.062151574718308*i_datadigi - 121.796885042846 - OFFSET_SENSOR_ARUS; // Modul B fix
	  IBATT_for_offset_cal= 0.062151574718308*i_datadigi - 121.796885042846;
  }

  else if (UNIQUE_Code == 0x00A22) {
	  IBATT = 0.0635607965300084*i_datadigi - 125.923575896323 - OFFSET_SENSOR_ARUS; // Modul B fix
	  IBATT_for_offset_cal= 0.0635607965300084*i_datadigi - 125.923575896323;
  }

  if(hitung_suhu >= interval_hitungsuhu) {

	  Res_T1=sum_datadigi_suhu1/hitung_suhu*10000/(3900-adc_value[1]); 	// 10000 => R1 , 3900 => Vcc dalam nilai digital
	  Suhu_T1= -24.05*log(Res_T1) + 246.41;			//1 / a + b (Ln RT / R25) + c b (Ln RT / R25)2
	  Res_T2=sum_datadigi_suhu2/hitung_suhu*10000/(3900-adc_value[2]);
	  Suhu_T2= -24.05*log(Res_T2) + 246.41;			//1 / a + b (Ln RT / R25) + c b (Ln RT / R25)2
	  Res_T3=sum_datadigi_suhu3/hitung_suhu*10000/(3900-adc_value[3]);
	  Suhu_T3= -24.05*log(Res_T3) + 246.41;			//1 / a + b (Ln RT / R25) + c b (Ln RT / R25)2
	  Res_T4=sum_datadigi_suhu4/hitung_suhu*10000/(3900-adc_value[4]);
	  Suhu_T4= -24.05*log(Res_T4) + 246.41;			//1 / a + b (Ln RT / R25) + c b (Ln RT / R25)2

	  if(Suhu_T1>=130) Suhu_T1 = 130;
	  if(Suhu_T2>=130) Suhu_T2 = 130;
	  if(Suhu_T3>=130) Suhu_T3 = 130;
	  if(Suhu_T4>=130) Suhu_T4 = 130;

	  sum_datadigi_suhu1=0;
	  sum_datadigi_suhu2=0;
	  sum_datadigi_suhu3=0;
	  sum_datadigi_suhu4=0;
	  hitung_suhu=0;
  }

  if(BATT_Start_Up==1)
  {
	  if(BATT_State == STATE_DISCHARGE){
		  Batt_Protection_when_discharge();
	  }
	  else if(BATT_State == STATE_CHARGE){
		  Batt_Protection_when_charge();
	  }
	  if(BATT_State == STATE_FULL_CHARGE_DISCHARGE){
		  Batt_Protection_when_chargedischarge();
	  }

	  //********************* Clearing protection status *****************************////
	  // ---> Clearing UnderVoltage
	  if(((Clear_Trip_undervoltage==1)||(VBATT>54))&&flag_trip_undervoltage==ON){
		  flag_trip_undervoltage=OFF;
		  Clear_Trip_undervoltage=0;
	  }
	  // ---> Clearing OverCurrent Discharge
	  if(flag_trip_overcurrentdischarge==ON && Clear_Trip_overcurrentdischarge==1){
		  flag_trip_overcurrentdischarge=OFF;
		  Clear_Trip_overcurrentdischarge=0;
	  }
	  // ---> Clearing OverTemperature
	  if(flag_trip_overtemperature==ON && (Suhu_T1<40)&&(Suhu_T2<50)&&(Suhu_T3<40)&&(Suhu_T4<50)){
		  flag_trip_overtemperature=OFF;
	  }
	  // ---> Clearing UnderTemperature
	  if(flag_trip_undertemperature==ON && (Suhu_T1>20)&&(Suhu_T2>20)&&(Suhu_T3>20)&&(Suhu_T4>20)){
		  flag_trip_undertemperature=OFF;
	  }
	  // ---> Clearing OverDischarge
	  if(flag_trip_SOCOverDischarge==ON && Pack_SOC>20){
		  flag_trip_SOCOverDischarge=OFF;
	  }
	  // ---> Clearing OverCharge
	  if(flag_trip_SOCOverCharge==ON && Pack_SOC<70){
		  flag_trip_SOCOverCharge=OFF;
	  }
  }

  i++;
  i=i%maxdata;

  //////////// Bagian Hitung SOC /////// SOC akan dihitung berdasarkan state baterai (Jika charge maupun discharge)
  if(BATT_State==STATE_CHARGE||BATT_State==STATE_DISCHARGE||BATT_State==STATE_FULL_CHARGE_DISCHARGE)
  {
	  time_soc++;
	  sum_current+=IBATT;
	  if(time_soc>99)
	  {
		  AH_Consumption = (-1*sum_current/100*(1.0/3600.0))/Pack_Cap*100-(4e-5); //Konsumsi System 4e-5
		  Pack_SOC=Pack_SOC+AH_Consumption;
		  time_soc=0;
		  sum_current=0;

		  grad=(100-0)/(batas_atas-batas_bawah);
		  constanta=grad*batas_bawah*(-1);
		  SOC_manipulasi=grad*Pack_SOC+constanta;
	  }
  }
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
  testtim3++;
  if(flag_start_shutdown==1){
	  if(Tick_33ms == 1) CANTX_BattParameter();
	  else if(Tick_33ms == 2) CANTX_BattProtection();
	  else if(Tick_33ms == 3) {
		  CANTX_ReportToCharger();
		  Tick_33ms = 0;
	  }
	  if(Tick_BattId > 30) {
		  CANTX_BattId();
		  Tick_BattId = 0;
	  }
	  Tick_33ms++;
	  Tick_BattId++;
  }
  /* USER CODE END TIM3_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void Batt_Protection_when_discharge(void) {
	//***************** Short Circuit Protection ***********************************//
	if(IBATT > (VBATT/0.5)) {
		Isc = IBATT;
		Vsc = VBATT;
		fault_code = 12;
		Batt_Open_Mode();
		flag_trip_shortcircuit = ON;
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
	}

	//***************** Undervoltage Protection ***********************************//
	else if(VBATT<V_Under_Set && flag_trip_undervoltage==OFF ) {   //Indikasi terjadi Undervoltage
		fault_code=1;
		T_Under_trip=TMS/(1-(VBATT/V_Under_Set));
		T_trip_cycle+=0.001;

		if(T_trip_cycle>T_Under_trip && flag_trip_undervoltage==OFF) {
			Batt_Open_Mode();
			T_trip_cycle=T_Under_trip;
			flag_trip_undervoltage=ON;
			HAL_GPIO_WritePin(BATT_CUTL_GPIO_Port, BATT_CUTL_Pin, GPIO_PIN_RESET);
		}

		if(flag_trip_undervoltage==OFF) {
			if(T_Under_trip-T_trip_cycle>15) {
				if((test_tim2%1000)==0) {
					BUZZ_Toggle;
					test_tim2=0;
				}
			}
			else if(T_Under_trip-T_trip_cycle>10) {
				if((test_tim2%100)==0) {
					BUZZ_Toggle;
					test_tim2=0;
				}
			}
			else if(T_Under_trip-T_trip_cycle>1) {
				HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
			}
		}
	}

	//************** OverCurrent Discharge **********************//
	else if((IBATT-I_Over_Set)>0 && flag_trip_overcurrentdischarge==OFF) {   //Indikasi terjadi Over Current
		fault_code=2;
		T_I_Over_trip=50/(((IBATT/6.9)*(IBATT/6.9))-1);
//		T_I_Over_trip=TMS_I_Over/((IBATT/I_Over_Set)-1);
		T_I_Over_trip_cycle+=0.01;

		if(T_I_Over_trip_cycle>T_I_Over_trip && flag_trip_overcurrentdischarge==OFF) {
			Batt_Open_Mode();
			T_I_Over_trip_cycle=T_I_Over_trip;
			flag_trip_overcurrentdischarge=ON;
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		}
		if(flag_trip_overcurrentdischarge==OFF) {
			if(T_I_Over_trip-T_I_Over_trip_cycle>15) {
				if((test_tim2%1000)==0) {
					BUZZ_Toggle;
					test_tim2=0;
				}
			}
			else if(T_I_Over_trip-T_I_Over_trip_cycle>10){
				if((test_tim2%100)==0){
					BUZZ_Toggle;
					test_tim2=0;
				}
			}
			else if(T_I_Over_trip-T_I_Over_trip_cycle>3){
				if((test_tim2%10)==0){
					BUZZ_Toggle;
					test_tim2=0;
				}
			}
			else if(T_I_Over_trip-T_I_Over_trip_cycle>1){
				HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
			}
		}
	}

	//**************Pengecekan OverTemperature ****************************//
	else if(((70-Suhu_T1<10)||(50-Suhu_T2<10)||(50-Suhu_T3<10)||(50-Suhu_T4<10)) && flag_trip_overtemperature==OFF) {
		fault_code=3;
		if(Suhu_T1>Temp_Over_Set-10 && Suhu_T1<=Temp_Over_Set-5) {
			if((test_tim2%1000)==0) {
				BUZZ_Toggle;
				test_tim2=0;
			}
		}
		else if(Suhu_T1>Temp_Over_Set-5 && Suhu_T1<=Temp_Over_Set-2){
			if((test_tim2%500)==0){
				BUZZ_Toggle;
				test_tim2=0;
			}
		}
		else if(Suhu_T1>Temp_Over_Set-2 && Suhu_T1<=Temp_Over_Set){
			if((test_tim2%500)==0){
				BUZZ_Toggle;
				test_tim2=0;
			}
		}
		else if(Suhu_T1>50||Suhu_T2>85||Suhu_T3>50||Suhu_T4>85){
			Batt_Open_Mode();
			flag_trip_overtemperature=ON;
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		}
	}

	//**************Pengecekan UnderTemperature ****************************//
	else if((Suhu_T1-Temp_Under_Set<=10||Suhu_T2-Temp_Under_Set<=10||Suhu_T3-Temp_Under_Set<=10||Suhu_T4-Temp_Under_Set<=10) && flag_trip_undertemperature==OFF) {
		fault_code=4;
		if(Suhu_T1<=Temp_Under_Set+10 && Suhu_T1>Temp_Under_Set+5){
			if((test_tim2%1000)==0){
				BUZZ_Toggle;
				test_tim2=0;
			}
		}
		else if(Suhu_T1<=Temp_Under_Set+5 && Suhu_T1>Temp_Under_Set+2){
			if((test_tim2%500)==0){
				BUZZ_Toggle;
				test_tim2=0;
			}
		}
		else if(Suhu_T1<Temp_Under_Set+2 && Suhu_T1>=Temp_Under_Set){
			if((test_tim2%500)==0){
				BUZZ_Toggle;
				test_tim2=0;
			}
		}
		else if(Suhu_T1<Temp_Under_Set||Suhu_T2<Temp_Under_Set||Suhu_T3<Temp_Under_Set||Suhu_T4<Temp_Under_Set){
			Batt_Open_Mode();
			flag_trip_undertemperature=ON;
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		}
	}

	//********************** SOC_OverDischarge****************************//
	else if(Pack_SOC-SOC_Under_Set<=10 && flag_trip_SOCOverDischarge==OFF && BATT_State==STATE_DISCHARGE) {
		fault_code=5;
		if(Pack_SOC<=SOC_Under_Set+10 && Pack_SOC>SOC_Under_Set+5){
			if((test_tim2%1000)==0){
				BUZZ_Toggle;
				test_tim2=0;
			}
		}
		else if(Pack_SOC<=SOC_Under_Set+5 && Pack_SOC>SOC_Under_Set+2){
			if((test_tim2%500)==0){
				BUZZ_Toggle;
				test_tim2=0;
			}
		}
		else if(Pack_SOC<SOC_Under_Set+2 && Pack_SOC>=SOC_Under_Set){
			if((test_tim2%500)==0){
				BUZZ_Toggle;
				test_tim2=0;
			}
		}
		else if(Pack_SOC<SOC_Under_Set){
			Batt_Open_Mode();
			flag_trip_SOCOverDischarge=ON;
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		}
	}

	//********************** Imbalance Checking Status ****************************//
	else if(Persen_Imbalance_Set-persen_imbalance<10)
	{
		fault_code=6;
		if(persen_imbalance>Persen_Imbalance_Set)
		{
			flag_trip_unbalance=ON;
			Batt_Open_Mode();
		}
	}

	//Clearing when data status is normal before system trip
	else {
		if(fault_code!=0)
			last_fault_code=fault_code;
		fault_code=0;
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		T_Under_trip=0;
		T_trip_cycle=T_trip_cycle-0.001;
		T_I_Over_trip_cycle-=0.001;
		if(T_trip_cycle<0)
			T_trip_cycle=0;
		if(T_I_Over_trip_cycle<0)
			T_I_Over_trip_cycle=0;
	}
}

void Batt_Protection_when_charge(void){
	//***************** Short Circuit Protection ***********************************//
	if(IBATT > (VBATT)) {
		Isc = IBATT;
		Vsc = VBATT;
		fault_code = 12;
		Batt_Open_Mode();
		flag_trip_shortcircuit = ON;
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
	}

	//**************Pengecekan OverCharge****************************//
	else if(SOC_Over_Set-Pack_SOC<=10 && flag_trip_SOCOverCharge==OFF) {
		fault_code=7;
		if(Pack_SOC>SOC_Over_Set){
			Batt_Open_Mode();
			flag_trip_SOCOverCharge=ON;
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		}
	}

	//**************Pengecekan OverTemperature ****************************//
	else if(((65-Suhu_T1<10)||(50-Suhu_T2<10)||(50-Suhu_T3<10)||(50-Suhu_T4<10)) && (flag_trip_overtemperature==OFF)) { // Warning Over Temperature Charge 40 65 40 65
		fault_code=8;
		if((Suhu_T1>45)||(Suhu_T2>80)||(Suhu_T3>45)||(Suhu_T4>80)) {
			Batt_Open_Mode();
			flag_trip_overtemperature=ON;
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		}
	}

	//**************Pengecekan UnderTemperature ****************************//
	else if((Suhu_T1-Temp_Under_Set<=10||Suhu_T2-Temp_Under_Set<=10||Suhu_T3-Temp_Under_Set<=10||Suhu_T4-Temp_Under_Set<=10) && flag_trip_undertemperature==OFF) {
		fault_code=9;
		if(Suhu_T1<=Temp_Under_Set+10 && Suhu_T1>Temp_Under_Set+5) {
			if((test_tim2%1000)==0) {
				BUZZ_Toggle;
				test_tim2=0;
			}
		}
		else if(Suhu_T1<=Temp_Under_Set+5 && Suhu_T1>Temp_Under_Set+2) {
			if((test_tim2%500)==0) {
				BUZZ_Toggle;
				test_tim2=0;
			}
		}
		else if(Suhu_T1<Temp_Under_Set+2 && Suhu_T1>=Temp_Under_Set) {
			if((test_tim2%500)==0) {
				BUZZ_Toggle;
				test_tim2=0;
			}
		}
		else if(Suhu_T1<Temp_Under_Set||Suhu_T2<Temp_Under_Set||Suhu_T3<Temp_Under_Set||Suhu_T4<Temp_Under_Set) {
			Batt_Open_Mode();
			flag_trip_undertemperature=ON;
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		}
	}

	//**************Pengecekan OverCurrent Charge **********************//
	else if((fabs(IBATT)-I_Over_Set_Charge)>0 && flag_trip_overcurrentcharge==OFF) {  //Indikasi terjadi Over Current
		fault_code=10;
		T_I_Over_trip=8/(((IBATT/6.9)*(IBATT/6.9))-1);
		T_I_Over_trip_cycle+=0.01;

		if(T_I_Over_trip_cycle>T_I_Over_trip && flag_trip_overcurrentcharge==OFF) {
			Batt_Open_Mode();
			T_I_Over_trip_cycle=T_I_Over_trip;
			flag_trip_overcurrentcharge=ON;
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		}

		if(flag_trip_overcurrentcharge==OFF) {
			if(T_I_Over_trip-T_I_Over_trip_cycle>15) {
				if((test_tim2%1000)==0) {
					BUZZ_Toggle;
					test_tim2=0;
				}
			}
			else if(T_I_Over_trip-T_I_Over_trip_cycle>10) {
				if((test_tim2%100)==0) {
					BUZZ_Toggle;
					test_tim2=0;
				}
			}
			else if(T_I_Over_trip-T_I_Over_trip_cycle>3) {
				if((test_tim2%10)==0) {
					BUZZ_Toggle;
					test_tim2=0;
				}
			}
			else if(T_I_Over_trip-T_I_Over_trip_cycle>1) {
				HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
			}
		}
	}

	///////////////////// Overvoltage Protection//////////////////////////////
	else if(VBATT>V_Over_Set) {
		fault_code=11;
		flag_trip_overvoltage=ON;
		Batt_Open_Mode();
	}

	//Clearing when data status is normal before system trip
	else {
		if(fault_code!=0)
			last_fault_code=fault_code;
		fault_code=0;
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		T_Under_trip=0;
		T_trip_cycle=T_trip_cycle-0.001;
		T_I_Over_trip_cycle-=0.001;
		if(T_trip_cycle<0)
			T_trip_cycle=0;
		if(T_I_Over_trip_cycle<0)
			T_I_Over_trip_cycle=0;
	}
}

void Batt_Protection_when_chargedischarge(void) {
	// Short circuit protection
	if(IBATT > (VBATT/0.9)) {
		Isc=IBATT;
		Vsc=VBATT;
		fault_code=12;
		Batt_Open_Mode();
		flag_trip_shortcircuit=ON;
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
	}

	// Under Voltage protection
	else if(VBATT < V_Under_Set && flag_trip_undervoltage == OFF ) { //Indikasi terjadi Undervoltage
		fault_code=1;
		T_Under_trip=TMS/(1-(VBATT/V_Under_Set));
		T_trip_cycle+=0.001;

		if(T_trip_cycle>T_Under_trip && flag_trip_undervoltage==OFF) {
			Batt_Open_Mode();
			T_trip_cycle=T_Under_trip;
			flag_trip_undervoltage=ON;
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		}

		if(flag_trip_undervoltage==OFF) {
			if(T_Under_trip-T_trip_cycle>15) {
				if((test_tim2%1000)==0) {
					BUZZ_Toggle;
					test_tim2=0;
				}
			}
			else if(T_Under_trip-T_trip_cycle>10) {
				if((test_tim2%100)==0) {
					BUZZ_Toggle;
					test_tim2=0;
				}
			}
			else if(T_Under_trip-T_trip_cycle>1) {
				HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
			}
		}
	}

	// Over Current Protection
	if((IBATT-I_Over_Set)>0 && flag_trip_overcurrentdischarge==OFF) {  //Indikasi terjadi Over Current
		fault_code=2;
		T_I_Over_trip=TMS_I_Over/((IBATT/I_Over_Set)-1);
		T_I_Over_trip_cycle+=0.001;

		if(T_I_Over_trip_cycle>T_I_Over_trip && flag_trip_overcurrentdischarge==OFF) {
			T_I_Over_trip_cycle=T_I_Over_trip;
			flag_trip_overcurrentdischarge=ON;
			Batt_Open_Mode();
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		}
		if(flag_trip_overcurrentdischarge==OFF) {
			if(T_I_Over_trip-T_I_Over_trip_cycle>15) {
				if((test_tim2%1000)==0) {
					BUZZ_Toggle;
					test_tim2=0;
				}
			}
			else if(T_I_Over_trip-T_I_Over_trip_cycle>10) {
				if((test_tim2%100)==0) {
					BUZZ_Toggle;
					test_tim2=0;
				}
			}
			else if(T_I_Over_trip-T_I_Over_trip_cycle>1) {
				HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
			}
		}
	}

	// Over Temperature protection
	else if(((50-Suhu_T1 < 10)||(85-Suhu_T2 < 10)||(50-Suhu_T3 < 10)||(85-Suhu_T4 < 10)) && flag_trip_overtemperature==OFF) {
		fault_code=3;
		if(Suhu_T1>Temp_Over_Set-10 && Suhu_T1<=Temp_Over_Set-5) {
			if((test_tim2%1000)==0) {
				BUZZ_Toggle;
				test_tim2=0;
			}
		}
		else if(Suhu_T1>Temp_Over_Set-5 && Suhu_T1<=Temp_Over_Set-2){
			if((test_tim2%500)==0) {
				BUZZ_Toggle;
				test_tim2=0;
			}
		}
		else if(Suhu_T1>Temp_Over_Set-2 && Suhu_T1<=Temp_Over_Set) {
			if((test_tim2%500)==0)
			{
				BUZZ_Toggle;
				test_tim2=0;
			}
		}
		else if(Suhu_T1>50||Suhu_T2>85||Suhu_T3>50||Suhu_T4>85) {
			Batt_Open_Mode();
			flag_trip_overtemperature=ON;
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		}
	}

	// Under Temperature protection
	else if((Suhu_T1-Temp_Under_Set<=10||Suhu_T2-Temp_Under_Set<=10||Suhu_T3-Temp_Under_Set<=10||Suhu_T4-Temp_Under_Set<=10) && flag_trip_undertemperature==OFF) {
		fault_code=4;
		if(Suhu_T1<=Temp_Under_Set+10 && Suhu_T1>Temp_Under_Set+5) {
			if((test_tim2%1000)==0) {
				BUZZ_Toggle;
				test_tim2=0;
			}
		}
		else if(Suhu_T1<=Temp_Under_Set+5 && Suhu_T1>Temp_Under_Set+2) {
			if((test_tim2%500)==0) {
				BUZZ_Toggle;
				test_tim2=0;
			}
		}
		else if(Suhu_T1<Temp_Under_Set+2 && Suhu_T1>=Temp_Under_Set) {
			if((test_tim2%500)==0) {
				BUZZ_Toggle;
				test_tim2=0;
			}
		}
		else if(Suhu_T1<Temp_Under_Set||Suhu_T2<Temp_Under_Set||Suhu_T3<Temp_Under_Set||Suhu_T4<Temp_Under_Set) {
			Batt_Open_Mode();
			flag_trip_undertemperature=ON;
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		}
	}

	// SOC OverDischarge protection
	else if(Pack_SOC-SOC_Under_Set<=10 && flag_trip_SOCOverDischarge==OFF && BATT_State==STATE_DISCHARGE) {
		fault_code=5;
		if(Pack_SOC<=SOC_Under_Set+10 && Pack_SOC>SOC_Under_Set+5) {
			if((test_tim2%1000)==0) {
				BUZZ_Toggle;
				test_tim2=0;
			}
		}
		else if(Pack_SOC<=SOC_Under_Set+5 && Pack_SOC>SOC_Under_Set+2) {
			if((test_tim2%500)==0) {
				BUZZ_Toggle;
				test_tim2=0;
			}
		}
		else if(Pack_SOC<SOC_Under_Set+2 && Pack_SOC>=SOC_Under_Set) {
			if((test_tim2%500)==0){
				BUZZ_Toggle;
				test_tim2=0;
			}
		}
		else if(Pack_SOC<SOC_Under_Set) {
			Batt_Open_Mode();
			flag_trip_SOCOverDischarge=ON;
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		}
	}

	// Imbalance Protection
	else if(Persen_Imbalance_Set-persen_imbalance<10)
	{
		fault_code=6;
		if(persen_imbalance>Persen_Imbalance_Set)
		{
			flag_trip_unbalance=ON;
			Batt_Open_Mode();
		}
	}

	//Clearing when data status is normal before system trip
	else {
		if(fault_code!=0)
			last_fault_code=fault_code;
		fault_code=0;
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		T_Under_trip=0;
		T_trip_cycle=T_trip_cycle-0.001;
		T_I_Over_trip_cycle-=0.001;
		if(T_trip_cycle < 0)
			T_trip_cycle=0;
		if(T_I_Over_trip_cycle < 0)
			T_I_Over_trip_cycle=0;
	}
}
/* USER CODE END 1 */

