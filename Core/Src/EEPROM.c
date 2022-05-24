/*
 * EEPROM.c
 *
 *  Created on: Apr 6, 2020
 *      Author: faiz
 */
#include "EEPROM.h"

void EEPROM_isDeviceReady(uint16_t addr)
{
	if (HAL_I2C_IsDeviceReady(&hi2c2, addr, 3, 100)!=HAL_OK){
		while(1){
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			HAL_Delay(100);
		}
	}
}

void EEPROM_WriteData(uint16_t addr, uint8_t data)
{
	HAL_I2C_Mem_Write(&hi2c2,EEPROM_ADDRESS,addr,64,&data,1,10);
	HAL_Delay(100);
}

uint8_t EEPROM_ReadData(uint16_t addr)
{
	uint8_t EEPROM_data;
	HAL_I2C_Mem_Read(&hi2c2,0xA0,addr,64,&EEPROM_data,1,10);
	HAL_Delay(1);
	return EEPROM_data;
}

void EEPROM_WriteChar(uint16_t addr, char data_char[])
{
	int len=0,j;
	len=strlen(data_char);
	for(j=0;j<=len;j++)
	{
		uint8_t data_write=data_char[j];
		HAL_I2C_Mem_Write(&hi2c2,EEPROM_ADDRESS,addr+j,64,&data_write,1,10);
		HAL_Delay(100);
	}
}

void EEPROM_ReadChar(uint16_t addr0, uint16_t addrn, uint8_t data_read_char[])
{
	uint8_t i;
	for(i=0;i<=(addrn-addr0);i++)
	{
		data_read_char[i]=EEPROM_ReadData(addr0+i);
	}
}


void EEPROM_WritemByte(uint16_t addr0, uint8_t mByte[])
{
	int j;
	for(j=0;j<=4;j++)
	{
		uint8_t data_write=mByte[j];
		HAL_I2C_Mem_Write(&hi2c2,EEPROM_ADDRESS,addr0+j,64,&data_write,1,10);
		HAL_Delay(1);
	}
}

void ReadmByte_FRAM(uint16_t addr0, uint8_t mByte[])
{
	int j;
	for(j=0;j<=4;j++)
	{
		mByte[j]=EEPROM_ReadData(addr0+j);
	}
}
