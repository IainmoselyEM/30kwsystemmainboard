/*
 * DCB_UART.c
 *
 *  Created on: 8 Nov 2024
 *      Author: IainMosely
 */

#include <SLS_ADC.h>
#include <SLS_I2C.h>
#include "driverlib.h"
#include "device.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "board.h"
#include <SLS_SPI.h>
#include <SLS_UART.h>
#include <SLS_THERMAL.h>

//
// DMA data sections
//

//
// Map the TX data buffer to DMA accessible global shared memory.
//
#pragma DATA_SECTION(txData, "ramgs0");

//
// Defines
//

//
// Amount of data words that will trigger UART FIFOs.
// For the 16 byte deep UART FIFOs: set to 2, 4, 8, 12 or 14.
// !IMPORTANT! Set "Burst Size" in Sysconfig myDMA0 and myDMA1 instances to the
// chosen UART_BUFFER_SIZE.
//
#define UART_BUFFER_SIZE  2

//
// Set total amount of data words to transfer from TxData to RxData.
// Must be divisible by UART_BUFFER_SIZE and even numbered.
// !IMPORTANT! Set "Transfer Size" in Sysconfig myDMA0 and myDMA1 instances
// to the BUFFER_SIZE / UART_BUFFER_SIZE.
//
#define BUFFER_SIZE      1024

//
// Globals
//

//
// BUFFER_SIZE elements stored in memory to be transmitted by UART.
// Each element is stored as a 16-bit value, however only the lower 8 bits will
// be transmitted since only 8 bits can be written to the UART data register.
//
uint16_t txData[BUFFER_SIZE];
int16_t Length = 0;

//
// Set up pointers to txData/rxData buffers, and UART data register.
//
const void *txAddr = (const void *)txData;
const void *drAddr = (const void *)(DCB_UART_BASE + UART_O_DR);

//
// TX DMA Channel 5 end of transfer ISR.
// Called when all data in txData is copied to the UART data register.
// Sets txDone flag which is checked in the main.
//

__interrupt void INT_UARTATX_DMA_ISR(void)
{
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP7);
    //txDone = 1;
    return;
}



void TransmitUartMessage(void)
{
    Length = 0;
    uint16_t i=0;
    int16_t Value = 0;
    Length=Length+sprintf((char *)(txData+Length),"\x1b[2J\x1b[H");
    Length=Length+sprintf((char *)(txData+Length),"\rDigital Control Board");
    Length=Length+sprintf((char *)(txData+Length),"\r\nCompile Date and Time ");
    Length=Length+sprintf((char *)(txData+Length),TIMESTAMP);
    Value = (int16_t)GetCONTTEMP();
    Length=Length+sprintf((char *)(txData+Length),"\r\nControl Board Temperature = %d degC", Value);
    Value= (int16_t)GetMidscaleVoltage();
    Length=Length+sprintf((char *)(txData+Length),"\r\nMidscale Voltage =  %4d mV", Value);
    Value= (int16_t)GetQuarterScaleVoltage();
    Length=Length+sprintf((char *)(txData+Length),"\r\nQuarterscale Voltage =  %4d mV", Value);
    Value= (int16_t)GetIA_IN();
    Length=Length+sprintf((char *)(txData+Length),"\r\nSide A Input Current =  %4d A", Value);
    Value= (int16_t)GetIA_L();
    Length=Length+sprintf((char *)(txData+Length),"\r\nSide A Inductor Current =  %4d A", Value);
    Value= (int16_t)GetIA_OUT();
    Length=Length+sprintf((char *)(txData+Length),"\r\nSide A Output Current =  %4d A", Value);
    Value= (int16_t)GetVA_OUT();
    Length=Length+sprintf((char *)(txData+Length),"\r\nSide A Output Voltage =  %4d V", Value);
    Value= (int16_t)GetIB_IN();
    Length=Length+sprintf((char *)(txData+Length),"\r\nSide B Input Current =  %4d A", Value);
    Value= (int16_t)GetIB_L();
    Length=Length+sprintf((char *)(txData+Length),"\r\nSide B Inductor Current =  %4d A", Value);
    Value= (int16_t)GetIB_OUT();
    Length=Length+sprintf((char *)(txData+Length),"\r\nSide B Output Current =  %4d A", Value);
    Value= (int16_t)GetVB_OUT();
    Length=Length+sprintf((char *)(txData+Length),"\r\nSide A Output Voltage =  %4d V", Value);
    Value= (int16_t)GetIDC();
    Length=Length+sprintf((char *)(txData+Length),"\r\nCommon DC Link Current =  %4d A", Value);
    Value= (int16_t)GetVDC();
    Length=Length+sprintf((char *)(txData+Length),"\r\nCommon DC Link Voltage =  %4d V", Value);
    Value= (int16_t)Get3V3();
    Length=Length+sprintf((char *)(txData+Length),"\r\n3V3 Rail Voltage =  %4d mV", Value);
    Value= (int16_t)Get5V();
    Length=Length+sprintf((char *)(txData+Length),"\r\n5V Rail Voltage =  %4d mV", Value);
    Value= (int16_t)Get24V();
    Length=Length+sprintf((char *)(txData+Length),"\r\n24V Rail Voltage =  %4d mV", Value);
    Value= (int16_t)GetFan1Speed();
    Length=Length+sprintf((char *)(txData+Length),"\r\nFan 1 Speed =  %5d RPM", Value);
    Value= (int16_t)GetFan1LocalTemp();
    Length=Length+sprintf((char *)(txData+Length),"\r\nFan 1 Local Temperature =  %d degC", Value);
    Value= (int16_t)GetPowerCycles();
    Length=Length+sprintf((char *)(txData+Length),"\r\nPower Cycles = %4d ", Value);
    Value= (int16_t)EQEP_getPosition(ENC1_BASE);
    Length=Length+sprintf((char *)(txData+Length),"\r\nENC1 = %4d ", Value);
    Value= (int16_t)EQEP_getPosition(ENC2_BASE);
    Length=Length+sprintf((char *)(txData+Length),"\r\nENC2 = %4d ", Value);
    for (i=Length;i<BUFFER_SIZE;i++)
    {
        txData[i]=0x20;
    }
    DMA_startChannel(UARTATX_DMA_BASE);

}

void ConfigureUART(void)
{
    //
    // Valid FIFO level triggers:
    // UART_BUFFER_SIZE = 2: TX/RX FIFOs will trigger DMA when they are 1/8
    //                       empty/full (2 available/written spots)
    // UART_BUFFER_SIZE = 4: TX/RX FIFOs will trigger DMA when they are 1/4
    //                       empty/full (4 available/written spots)
    // UART_BUFFER_SIZE = 8: TX/RX FIFOs will trigger DMA when they are 1/2
    //                       empty/full (8 available/written spots)
    //
    // UART_BUFFER_SIZE = 12: TX/RX FIFOs will trigger DMA when they are 3/4
    //                       empty/full (12 available/written spots)
    //
    // UART_BUFFER_SIZE = 14: TX/RX FIFOs will trigger DMA when they are 7/8
    //                       empty/full (14 available/written spots)
    //

    //
    // FIFO interrupt levels are set to generate an interrupt
    // when the TX FIFO is less than or equal to 16-burstSize elements full
    // and the RX FIFO is greater than or equal to burstSize elements full.
    //
    UART_setFIFOLevel(DCB_UART_BASE, UART_FIFO_TX7_8, UART_FIFO_RX1_8);


    //
    // Enable DMA for TX and RX events
    //
    UART_enableDMA(DCB_UART_BASE, UART_DMA_TX);

}
