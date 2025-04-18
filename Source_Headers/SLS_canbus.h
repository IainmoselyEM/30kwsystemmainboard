/*
 * EM_CANBUS.h
 *
 *  Created on: 3 Feb 2022
 *      Author: Iainmosely
 */

#ifndef HEADERS_NB_CANBUS_H_
#define HEADERS_NB_CANBUS_H_


void Init_CANA(void);
void TransmitCANMessage(void);
void PackageCANData(void);
void CANMessengerTick(void);
//PowerContState GetCANBUSCommandState(void);
uint16_t WaitForCANStatus(uint32_t base, uint16_t status, uint16_t timeout_ms);
void CheckForCANBUSMessage6Timeout();

#define CANWAIT_SUCCESS             0U //Success return value from the WaitForCAN.. functions
#define CANWAIT_ERROR_TIMEOUT       1U //Timeout return value from the WaitForCAN.. functions

#define CANWAIT_DEFAULT_TIMEOUT_ms  10 //A default timeout that can be passed to the WaitForCAN.. functions

typedef struct __attribute__((aligned(4)))
{
    uint32_t IA_L : 16;
    uint32_t IA_IN : 16;
    uint32_t IA_OUT : 16;
    uint32_t VA_OUT : 16;
}SLSMsg1;

typedef struct __attribute__((aligned(4)))
{
    uint32_t IB_L : 16;
    uint32_t IB_IN : 16;
    uint32_t IB_OUT : 16;
    uint32_t VB_OUT : 16;
}SLSMsg2;

typedef struct __attribute__((aligned(4)))
{
    uint32_t VDC : 16;
    uint32_t IDC : 16;
    uint32_t THERMA : 16;
    uint32_t THERMB : 16;
}SLSMsg3;

typedef struct __attribute__((aligned(4)))
{
    uint32_t P3V3 : 16;
    uint32_t P5V : 16;
    uint32_t P24V : 16;
    uint32_t P1V25 : 16;
}SLSMsg4;

typedef struct __attribute__((aligned(4)))
{
    uint32_t P625MV : 16;
    uint32_t CONTTEMP : 16;
    uint32_t FAN1DUTY : 16;
    uint32_t FAN2DUTY : 16;
}SLSMsg5;

typedef struct __attribute ((aligned(4)))
{
    uint32_t FAN1TEMP : 16;
    uint32_t FAN2TEMP : 16;
    uint32_t FAN1SPEED : 16;
    uint32_t FAN2SPEED : 16;
}SLSMsg6;

typedef struct __attribute ((aligned(4)))
{
    uint32_t FAULTSTATUS : 16;
    uint32_t STATE : 8;
    uint32_t POWERCYCLES : 16;
}SLSMsg7;

typedef struct __attribute ((aligned(4)))
{
    uint32_t REQUESTSTATE : 8;
    uint32_t FAN1DUTY : 8;
}SLSMsg8;

typedef struct __attribute__((aligned(4)))
{
    SLSMsg1 SLSMsg1;
    SLSMsg2 SLSMsg2;
    SLSMsg3 SLSMsg3;
    SLSMsg4 SLSMsg4;
    SLSMsg5 SLSMsg5;
    SLSMsg6 SLSMsg6;
    SLSMsg7 SLSMsg7;
    SLSMsg8 SLSMsg8;
}PwrCAN;



#endif /* HEADERS_NB_CANBUS_H_ */
