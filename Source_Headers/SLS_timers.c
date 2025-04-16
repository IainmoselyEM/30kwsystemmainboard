/*
 * EM_Timers.c
 *
 *  Created on: 11 Jan 2022
 *      Author: IainMosely
 */

#include <SLS_states.h>
#include <SLS_SPI.h>
#include <SLS_I2C.h>
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include <stdbool.h>
#include <stdio.h>
#include <SLS_Timers.h>
#include <SLS_UART.h>
#include <SLS_THERMAL.h>
#include <SLS_CANBUS.h>



volatile uint16_t UART_Flag = 0;
volatile uint16_t UART_ISR_Counter = 0;

static uint64_t BgTimerCounter = 0; //A background timer counter that can be used for general timing operations
#define BG_TIMER_PRD_US   1000 //Period for BgTimerCounter in us

void CheckTwoHz(void)
{
    if(UART_Flag >= 1)
    {
        TwoHzUpdate();
        UART_Flag = 0;
        UART_ISR_Counter--;
    }
}

__interrupt void INT_Heartbeat2Hz_ISR(void)
{
    UART_Flag=1;
    UART_ISR_Counter++;
    //
    // The CPU acknowledges the interrupt.
    //
}

__interrupt void INT_myCPUTIMER0_ISR(void)
{
    //Increment the background timer counter
    BgTimerCounter++;
    // Acknowledge this interrupt to receive more interrupts from group 1
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

//! \brief      Gets the current value of the background timer
//!
uint64_t BGTimer_getCurrentTime(){
    return BgTimerCounter;
}

//! \brief                  Computes the elapsed time (in ms) between the current time and a given start time
//! \param[in] startTime    The start time (e.g. from BGTimer_getCurrentTime())
//!
uint64_t BGTimer_getElapsedTime_ms(uint64_t startTime){
    uint64_t diffTime;
    if (BgTimerCounter < startTime){
        //Counter has looped, work out considering this
        diffTime = BgTimerCounter + (UINT64_MAX - startTime);
    } else {
        //Counter has not looped, simple subtraction
        diffTime = BgTimerCounter - startTime;
    }
    //work out the time difference in ms
    return diffTime * BG_TIMER_PRD_US/1000;
}

void TwoHzUpdate(void)
{
    TransmitUartMessage();
    HeartbeatToggle();
    TMP75C_Read_Temp();
    FanControl();
    PackageCANData();
    TransmitCANMessage();
}

void HeartbeatToggle(void)
{
    GPIO_togglePin(Heartbeat);
}





