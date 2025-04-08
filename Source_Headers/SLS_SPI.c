/*
 * PE_SPI.c
 *
 *  Created on: 30 Aug 2024
 *      Author: IainMosely
 */

#include "device.h"
#include <board.h>
#include <SLS_SPI.h>

#define PowerCycleAddress   0x0010
uint16_t PowerCycles;

void FM25V02A_Write_Data(uint16_t address, uint16_t data)
{
    SPI_resetTxFIFO(FM25V02A_BASE);
    SPI_resetRxFIFO(FM25V02A_BASE);
    SPI_writeDataBlockingFIFO(FM25V02A_BASE, 0b00000110<<8);   // Send 0b00000110 opcode to enable WREN bit in status register
    while(!(SPI_getRxFIFOStatus(FM25V02A_BASE)==SPI_FIFO_RX1));
    SPI_resetRxFIFO(FM25V02A_BASE);
    SPI_writeDataBlockingFIFO(FM25V02A_BASE, 0b00000010<<8);   // Send 0b00000010 opcode to enable write to NVM
    SPI_writeDataBlockingFIFO(FM25V02A_BASE, address);         // Send MSB of address to write to
    SPI_writeDataBlockingFIFO(FM25V02A_BASE, address<<8);      // Send LSB of address to write to
    SPI_writeDataBlockingFIFO(FM25V02A_BASE, data);            // Send MSB of data to write
    SPI_writeDataBlockingFIFO(FM25V02A_BASE, data<<8);              // Send LSB of data to write
    while(!(SPI_getRxFIFOStatus(FM25V02A_BASE)==SPI_FIFO_RX5));
}

uint16_t FM25V02A_Read_Data(uint16_t address)
{
    uint16_t Byte1,Byte2;
    SPI_resetRxFIFO(FM25V02A_BASE);
    SPI_resetTxFIFO(FM25V02A_BASE);
    SPI_writeDataBlockingFIFO(FM25V02A_BASE, 0b00000011<<8);   // Send 0b00000011 opcode to read memory data
    SPI_writeDataBlockingFIFO(FM25V02A_BASE, address);         // Send MSB of address to read from
    SPI_writeDataBlockingFIFO(FM25V02A_BASE, address<<8);      // Send LSB of address to read from
    SPI_writeDataBlockingFIFO(FM25V02A_BASE, 0b0);             // Send dummy data to toggle clock
    SPI_writeDataBlockingFIFO(FM25V02A_BASE, 0b0);             // Send dummy data to toggle clock
    while(!(SPI_getRxFIFOStatus(FM25V02A_BASE)==SPI_FIFO_RX5));
    SPI_readDataBlockingFIFO(FM25V02A_BASE);
    SPI_readDataBlockingFIFO(FM25V02A_BASE);
    SPI_readDataBlockingFIFO(FM25V02A_BASE);
    Byte1=SPI_readDataBlockingFIFO(FM25V02A_BASE);
    Byte2=SPI_readDataBlockingFIFO(FM25V02A_BASE);
    return ((Byte1<<8)+Byte2);
}

void SetNVMAllZero(void)
{
    uint16_t i=0;
    SPI_resetTxFIFO(FM25V02A_BASE);
    SPI_resetRxFIFO(FM25V02A_BASE);
    SPI_writeDataBlockingFIFO(FM25V02A_BASE, 0b00000110<<8);     // Send 0b00000110 opcode to enable WREN bit in status register
    while(!(SPI_getRxFIFOStatus(FM25V02A_BASE)==SPI_FIFO_RX1));
    SPI_writeDataBlockingFIFO(FM25V02A_BASE, 0b00000010<<8);     // Send 0b00000010 opcode to enable write to NVM
    while(i<32768)
    {
        SPI_writeDataBlockingFIFO(FM25V02A_BASE,i);
        i++;
    }

}

void IncrementPowerCycles(void)
{
    PowerCycles=(FM25V02A_Read_Data(PowerCycleAddress))+1;
    FM25V02A_Write_Data(PowerCycleAddress,PowerCycles);
}

uint16_t GetPowerCycles(void)
{
    return(PowerCycles);
}
