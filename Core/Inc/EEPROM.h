/*
 * EEPROM.h
 *
 *  Created on: Apr 6, 2020
 *      Author: faiz
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "i2c.h"
#include "ssd1306.h"
#include "stdio.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#define EEPROM_ADDRESS 0xA0

void EEPROM_WriteData(uint16_t addr, uint8_t data);
uint8_t EEPROM_ReadData(uint16_t addr);
void EEPROM_isDeviceReady(uint16_t addr);
void EEPROM_WriteChar(uint16_t addr, char data_char[]);
void EEPROM_ReadChar(uint16_t addr0, uint16_t addrn, uint8_t data_read_char[]);
void EEPROM_WritemByte(uint16_t addr0, uint8_t mByte[]);
void ReadmByte_FRAM(uint16_t addr0, uint8_t mByte[]);


#endif /* INC_EEPROM_H_ */
