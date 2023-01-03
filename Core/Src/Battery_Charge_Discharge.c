#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "gpio.h"

#include "Battery_Charge_Discharge.h"
#include "EEPROM.h"

uint8_t	cycle, flag_write_cycle;
extern float VBATT, IBATT;

extern float Pack_SOC, Delta_VCell,Bat_Pow_Out, Pack_Cap;
extern uint16_t LifeTime;
extern uint8_t BATT_State;

void Batt_Discharge_Mode(void)
{
	if(flag_trip_undervoltage==ON||
			flag_trip_overtemperature==ON||
			flag_trip_undertemperature==ON||
			flag_trip_overcurrentdischarge==ON||
			flag_trip_SOCOverDischarge==ON||
			flag_trip_shortcircuit==ON||
			flag_trip_unbalance==ON||
			flag_trip_systemfailure==ON||
			VBATT < 30)
	{
		Batt_Open_Mode();
	}
	else
	{
		HAL_GPIO_WritePin(GATE_MOS_GPIO_Port, GATE_MOS_Pin, GPIO_PIN_SET);
		BATT_State=STATE_DISCHARGE;

		charge_state=0;
		discharge_state=1;
		sleep_state=0;
		flag_write_cycle = 0;
	}


}

void Batt_Charge_Mode(void)
{
	if(flag_trip_overvoltage==ON			||
			flag_trip_overtemperature==ON	||
			flag_trip_undertemperature==ON	||
			flag_trip_overcurrentcharge==ON	||
			flag_trip_SOCOverCharge==ON		||
			flag_trip_shortcircuit==ON		||
			flag_trip_systemfailure==ON		)
	{
		Batt_Open_Mode();
	}
	else
	{
		HAL_GPIO_WritePin(GATE_MOS_GPIO_Port, GATE_MOS_Pin, GPIO_PIN_SET);
		BATT_State=STATE_CHARGE;
		charge_state=1;
		discharge_state=0;
		sleep_state=0;

		if (flag_write_cycle == 0){
			cycle = EEPROM_ReadData(11) + 1;
			EEPROM_WriteData(11, cycle);
			flag_write_cycle = 1;
		}
	}

}

void Batt_Full_CD_Mode(void)
{
	if(flag_trip_undervoltage==ON			||
			flag_trip_overvoltage==ON		||
			flag_trip_overtemperature==ON	||
			flag_trip_undertemperature==ON	||
			flag_trip_overcurrentdischarge==ON||
			flag_trip_overcurrentcharge==ON	||
			flag_trip_SOCOverDischarge==ON	||
			flag_trip_SOCOverCharge==ON		||
			flag_trip_shortcircuit==ON		||
			flag_trip_unbalance==ON			||
			flag_trip_systemfailure==ON		)
	{
		Batt_Open_Mode();
	}
	else
	{
		HAL_GPIO_WritePin(GATE_MOS_GPIO_Port, GATE_MOS_Pin, GPIO_PIN_SET);
		BATT_State=STATE_FULL_CHARGE_DISCHARGE;
		charge_state=1;
		discharge_state=1;
		sleep_state=0;
		flag_write_cycle = 0;
	}
}

void Batt_Open_Mode(void)
{
	HAL_GPIO_WritePin(GATE_MOS_GPIO_Port, GATE_MOS_Pin, GPIO_PIN_RESET);
	BATT_State=STATE_STANDBY;
	charge_state=0;
	discharge_state=0;
	sleep_state=1;

	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);

	check_SOC_Based_OCV();
}

void check_SOC_Based_OCV(void)
{
	if(VBATT <= 51.3){
		Pack_SOC = 24.03846153846*(VBATT/15.0) - 77.18750000000;
	}
	else if(VBATT > 51.3 &&  VBATT <= 53.5){
		Pack_SOC = 135.26698598540*(VBATT/15.0) - 458.27213056570;
	}
	else if(VBATT > 53.5 &&  VBATT <= 54.7){
		Pack_SOC = 332.88158563421*(VBATT/15.0) - 1161.98331356855;
	}
	else if(VBATT > 54.7){
		Pack_SOC = 111.42655038475*(VBATT/15.0) - 353.86053305809;
	}

	Pack_SOC=(0.4884934490 * VBATT * VBATT) - (26.2875616013 * VBATT) + 348.6849534722;   //Persamaan Baterai INR 21700
//	Pack_SOC=(147.471026094008*(VBATT/15.0) - 494.687746093127);  // Persamaan Baterai EVE ICR18650/26V

	grad=(100-0)/(batas_atas-batas_bawah);
	constanta=grad*batas_bawah*(-1);
	SOC_manipulasi=grad*Pack_SOC+constanta;

	if(Pack_SOC>130) Pack_SOC=100;
	else if(Pack_SOC>100) Pack_SOC=100;
	else if(Pack_SOC<0) Pack_SOC=0;
}
