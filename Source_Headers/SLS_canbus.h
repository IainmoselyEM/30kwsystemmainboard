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
    uint32_t PwrContHwId : 18;
    uint32_t PwrContSwIdDay : 5;
    uint32_t PwrContSwIdMinute : 5;
    uint32_t PwrContSwIdMonth : 4;
    uint32_t PwrContSwIdSecond : 6;
    uint32_t PwrContSwIdYear : 4;
    uint32_t PwrContTimeSincePowerOn : 16;
    uint32_t PwrContCommandCurrIdem : 16;

}PwrContMsg1;

typedef struct __attribute__((aligned(4)))
{
    uint32_t PwrContCommandCurrIdem : 16;
    uint32_t PwrContInductorCurrIind : 16;
    uint32_t PwrContInputCurrIin : 16;
    uint32_t PwrContOutputCurrIout : 16;
}PwrContMsg2;

typedef struct __attribute__((aligned(4)))
{
    uint32_t PwrContInputVoltageVin : 13;
    uint32_t PwrContMaxPwrLimitPmax : 16;
    uint32_t PwrContMaxVoltageVmax : 12;
    uint32_t PwrContOutputVoltageVout : 16;
}PwrContMsg3;

typedef struct __attribute__((aligned(4)))
{
    uint32_t PwrCont12VBattCurrIbatt : 16;
    uint32_t PwrCont12VBattSoc : 12;
    uint32_t PwrCont12VSupplyVbatt : 12;
    uint32_t PwrContBiasSupplyVbias : 12;
    uint32_t PwrContStatus : 8;
}PwrContMsg4;

typedef struct __attribute__((aligned(4)))
{
    uint32_t PwrContCmdStructure : 8;
    uint32_t PwrContControlBrdTemp : 8;
    uint32_t PwrContModATemp : 8;
    uint32_t PwrContModBTemp : 8;
}PwrContMsg5;

typedef struct __attribute ((aligned(4)))
{
    uint32_t PwrContInputCommand : 8;
}PwrConMsg6;

typedef struct __attribute__((aligned(4)))
{
    PwrContMsg1 PwrContMsg1;
    PwrContMsg2 PwrContMsg2;
    PwrContMsg3 PwrContMsg3;
    PwrContMsg4 PwrContMsg4;
    PwrContMsg5 PwrContMsg5;
}PwrCAN;



#endif /* HEADERS_NB_CANBUS_H_ */
