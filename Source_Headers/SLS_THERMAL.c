/*
 * PE_I2C.c
 *
 *  Created on: 20th August 2024
 *      Author: IainMosely
 */

#include "device.h"
#include <board.h>
#include <SLS_I2C.h>
#include <SLS_THERMAL.h>
#include <SLS_Timers.h>
#include "driverlib.h"


float ContTemp = 0;                         // Control board temperature in Deg C
float FAN1TEMP = 0;                       // Power Stage #1 Transformer Temperature
float FAN2TEMP = 0;                       // Power Stage #2 Transformer Temperature
uint16_t FAN1SPEED = 0;
uint16_t FAN2SPEED = 0;
float FAN1DUTYCYCLE = 0.0f;
float FAN2DUTYCYCLE = 0.0f;

uint16_t FAN1StatusReg1=0;
uint16_t FAN1StatusReg2=0;
uint16_t FAN2StatusReg1=0;
uint16_t FAN2StatusReg2=0;

#define TMP75C_BASE         I2CA_BASE
#define POWERBOARD_BASE     I2CB_BASE
#define FAN1ADD             0x18
#define FAN2ADD             0x4D
#define TMPScalingFactor    256.0f      // Scaling Factor Used with TMP75C temp sensor to scale value read to degrees C
#define CONFIG1             0x00
#define CONFIG2             0x01
#define CONFIG3             0x3F
#define CONFIG4             0x04
#define LocalTempHighReg    0x0A
#define LocalTempLowReg     0x06
#define TachoSetLowByteReg  0x1E
#define TachoSetHighByteReg 0x1F
#define TachoLowByteReg     0x08
#define TachoHighByteReg    0x09
#define FanCharReg          0x20
#define DCYRampReg          0x23
#define DCYReg              0x22

float GetCONTTEMP(void)
{
    float Value=(float)ContTemp;
    return(Value);
}

void SetFan1DutyCycle(float DutyCycle)
{
    FAN1DUTYCYCLE=DutyCycle;
}

void SetFan2DutyCycle(float DutyCycle)
{
    FAN2DUTYCYCLE=DutyCycle;
}

void FanControl(void)
{
    FAN1TEMP=((float)FANTEMPLOCAL(FAN1ADD))/TMPScalingFactor;
    FAN2TEMP=((float)FANTEMPLOCAL(FAN2ADD))/TMPScalingFactor;
    FAN1SPEED=ReadFanSpeed(FAN1ADD);
    FAN2SPEED=ReadFanSpeed(FAN2ADD);
    FAN1StatusReg1=I2CReadSingleByte(POWERBOARD_BASE, 0x02, FAN1ADD);
    FAN1StatusReg2=I2CReadSingleByte(POWERBOARD_BASE, 0x03, FAN1ADD);
    FAN2StatusReg1=I2CReadSingleByte(POWERBOARD_BASE, 0x02, FAN2ADD);
    FAN2StatusReg2=I2CReadSingleByte(POWERBOARD_BASE, 0x03, FAN2ADD);
    ProgramFanDutyCycle(FAN1DUTYCYCLE, FAN1ADD);
    ProgramFanDutyCycle(FAN2DUTYCYCLE, FAN2ADD);
}

float GetFan1LocalTemp()
{
    return ((float)FAN1TEMP);
}

float GetFan2LocalTemp()
{
    return ((float)FAN2TEMP);
}

float GetFan1Speed(void)
{
    if(FAN1SPEED==0xFFFF)
    {
        return 0;
    }
    else
    {
        return(6000000.0f/((float)FAN1SPEED));
    }
}

float GetFan2Speed(void)
{
    if(FAN2SPEED==0xFFFF)
    {
        return 0;
    }
    else
    {
        return(6000000.0f/((float)FAN2SPEED));
    }
}

void FANinit(void)
{
    I2CWriteSingleByte(POWERBOARD_BASE,CONFIG1,FAN1ADD,0b10010101);    // Set FAN1 to direct duty cycle control and enable ADCs
    I2CWriteSingleByte(POWERBOARD_BASE,CONFIG1,FAN2ADD,0b10010101);    // Set FAN2 to direct duty cycle control and enable ADCs
    I2CWriteSingleByte(POWERBOARD_BASE,CONFIG2,FAN1ADD,0b00111111);    // Set FAN1 TACH Mode to 1
    I2CWriteSingleByte(POWERBOARD_BASE,CONFIG2,FAN2ADD,0b00111111);    // Set FAN2 TACH Mode to 1
    I2CWriteSingleByte(POWERBOARD_BASE,CONFIG4,FAN1ADD,0b10001000);    // Set FAN1 Configure bit to 1 as required
    I2CWriteSingleByte(POWERBOARD_BASE,CONFIG4,FAN2ADD,0b10001000);    // Set FAN1 Configure bit to 1 as required
    I2CWriteSingleByte(POWERBOARD_BASE,FanCharReg,FAN1ADD,0b00001101); // Enable Fan Spin process for FAN1, 10kHz PWM
    I2CWriteSingleByte(POWERBOARD_BASE,FanCharReg,FAN2ADD,0b00001101); // Enable Fan Spin process for FAN2, 10kHz PWM
    I2CWriteSingleByte(POWERBOARD_BASE,CONFIG3,FAN1ADD,0b00000010);    // Disable Fan full speed when THERM goes low for FAN1
    I2CWriteSingleByte(POWERBOARD_BASE,CONFIG3,FAN2ADD,0b00000010);    // Disable Fan full speed when THERM goes low for FAN2
}

int16_t FANTEMPLOCAL(uint16_t I2CAddress)
{
    uint16_t Value;
    Value=I2CReadSingleByte(POWERBOARD_BASE, LocalTempHighReg, I2CAddress)<<8;  // Read High byte of Fan Local Temperature and shift left by three bits
    Value=Value | I2CReadSingleByte(POWERBOARD_BASE, LocalTempLowReg, I2CAddress); // Read low byte of Fan local temperature and OR together with upper three bits to form 11 bit value
    return (int16_t)Value;
}

void ProgramFanDutyCycle(float DutyCycle, uint16_t I2CAddress)
{
    I2CWriteSingleByte(POWERBOARD_BASE, DCYReg, I2CAddress, (uint16_t)(2.55f*DutyCycle));
}

uint16_t ReadFanSpeed(uint16_t I2CAddress)
{
    uint16_t Value;
    Value=I2CReadSingleByte(POWERBOARD_BASE, TachoLowByteReg, I2CAddress);  // Read Tacho Speed Register Low Byte
    Value=Value | (I2CReadSingleByte(POWERBOARD_BASE, TachoHighByteReg, I2CAddress)<<8);  // Read Tacho Speed Register Low Byte
    return Value;
}

void TMP75C_Read_Temp(void)
{
    // The TMP75C temperature is accessed in register 0b0 and occupies the top 12 bits of the register. The top 8 bits are used to return the whole number and the remaining 4 bits are the fractional part.
    ContTemp = (float)(I2CReadTwoBytes(TMP75C_BASE,0b0,0b1001111)/TMPScalingFactor);       // Read Temperature Register of TMP75C (0b0) and divide by 256 to extract temperature in Deg C
}


