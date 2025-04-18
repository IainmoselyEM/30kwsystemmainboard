/*
 * PE_I2C.h
 *
 *  Created on: 20th August 2024
 *      Author: IainMosely
 */

#ifndef HEADERS_SLS_THERMAL_H_
#define HEADERS_SLS_THERMAL_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

void TMP75C_Read_Temp(void);
float GetCONTTEMP(void);
int16_t FANTEMPLOCAL(uint16_t I2CAddress);
void SetFanDutyCycle(float DutyCycle, uint16_t I2CAddress);
void FANinit(void);
float GetFan1Speed(void);
void FanControl(void);
uint16_t ReadFanSpeed(uint16_t I2CAddress);
int16_t FANTEMPLOCAL(uint16_t I2CAddress);
float GetFan1LocalTemp();
float GetFan2LocalTemp();
void ProgramFanDutyCycle(float DutyCycle, uint16_t I2CAddress);
void SetFan1DutyCycle(float DutyCycle);
void SetFan2DutyCycle(float DutyCycle);
float GetFan2Speed(void);

#endif /* HEADERS_PE_I2C_H_ */
