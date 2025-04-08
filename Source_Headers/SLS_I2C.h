/*
 * PE_I2C.h
 *
 *  Created on: 20th August 2024
 *      Author: IainMosely
 */

#ifndef HEADERS_SLS_I2C_H_
#define HEADERS_SLS_I2C_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>


static uint16_t WaitForI2CStopConditionStatus(uint32_t base, bool status, uint16_t timeout_ms);
static uint16_t WaitForI2CStatus(uint32_t base, uint16_t status, uint16_t timeout_ms);
void I2CA_Init(void);
void I2CB_Init(void);
uint16_t I2CWriteSingleByte(uint32_t Base, uint16_t Address, uint16_t I2CAddress, uint16_t Byte);
uint16_t I2CWriteTwoBytes(uint32_t Base, uint16_t Address, uint16_t I2CAddress, uint16_t TwoBytes);
uint16_t I2CReadSingleByte(uint32_t Base, uint16_t Address, uint16_t I2CAddress);
uint16_t I2CReadTwoBytes(uint32_t Base, uint16_t Address, uint16_t I2CAddress);


#endif /* HEADERS_PE_I2C_H_ */
