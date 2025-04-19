/*
 * DCB_States.c
 *
 *  Created on: 8 Nov 2024
 *      Author: IainMosely
 */

#include <SLS_States.h>

PowerContState SystemState=Standby;

PowerContState GetCurrentState(void)
{
    return(SystemState);
}

void SetCurrentState(PowerContState State)
{
    SystemState = State;
}

void StandbyCode(void)
{
    // Code to run when we enter standby state
}

void FaultCode(void)
{
    // Code to run when we enter Fault State
}


bool RunningCode(void)
{
    // Code to run when we enter running state
    return(true);
}

void StopCode(void)
{
    // Code to run when we enter stop state
}

