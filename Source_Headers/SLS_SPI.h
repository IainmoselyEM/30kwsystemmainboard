/*
 * PE_SPI.h
 *
 *  Created on: 30 Aug 2024
 *      Author: IainMosely
 */

#ifndef BSW_PE_SPI_H_
#define BSW_PE_SPI_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

void IncrementPowerCycles(void);
uint16_t GetPowerCycles(void);
void FM25V02A_Write_Data(uint16_t, uint16_t);
uint16_t FM25V02A_Read_Data(uint16_t);
void SetNVMAllZero(void);


#endif /* BSW_PE_SPI_H_ */
