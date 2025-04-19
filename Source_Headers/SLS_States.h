/*
 * DCB_States.h
 *
 *  Created on: 8 Nov 2024
 *      Author: IainMosely
 */

#ifndef SOURCE_HEADERS_SLS_STATES_H_
#define SOURCE_HEADERS_SLS_STATES_H_

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

typedef enum
{
    Standby,
    Running,
    Fault,
    Stop,

}PowerContState;

void StandbyCode(void);
//uint16_t StartSessionCode(void);
bool ChargingCode(void);
void StopSessionCode(void);
void FaultCode_CalledFromErrorsModule(void);
uint16_t PreChargeCode(void);
void CheckTwoHz(void);
void SetCurrentState(PowerContState);
PowerContState GetCurrentState(void);

#endif /* SOURCE_HEADERS_SLS_STATES_H_ */
