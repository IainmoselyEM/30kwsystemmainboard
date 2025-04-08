/*
 * PE_I2C.c
 *
 *  Created on: 20th August 2024
 *      Author: IainMosely
 */

#include "device.h"
#include <board.h>
#include <SLS_I2C.h>
#include <SLS_Timers.h>
#include "driverlib.h"

#define I2CWAIT_SUCCESS             0U      //Success return value from the WaitForI2C.. functions
#define I2CWAIT_ERROR_TIMEOUT       1U      //Timeout return value from the WaitForI2C.. functions
#define I2CWAIT_DEFAULT_TIMEOUT_ms  10      //A default timeout that can be passed to the WaitForI2C.. functions
#define TMP75C_BASE                 I2CA_BASE
#define POWERBOARD_BASE             I2CB_BASE

void I2CA_Init(void)
{

    GPIO_setPinConfig(GPIO_56_I2CA_SDA);
    GPIO_setPadConfig(56, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(56, GPIO_QUAL_ASYNC);

    //
    // Configuration for the I2CA SCL Pin
    //
    GPIO_setPinConfig(GPIO_57_I2CA_SCL);
    GPIO_setPadConfig(75, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(75, GPIO_QUAL_ASYNC);

    I2C_disableModule(TMP75C_BASE);
    I2C_initController(TMP75C_BASE, DEVICE_SYSCLK_FREQ, 100000, I2C_DUTYCYCLE_50);
    I2C_setConfig(TMP75C_BASE, I2C_CONTROLLER_SEND_MODE);
    I2C_disableLoopback(TMP75C_BASE);
    I2C_setOwnAddress(TMP75C_BASE, 0x0);
    I2C_setTargetAddress(TMP75C_BASE, 0x4F);
    I2C_setBitCount(TMP75C_BASE,I2C_BITCOUNT_8);
    I2C_setDataCount(TMP75C_BASE,1);
    I2C_setAddressMode(TMP75C_BASE,I2C_ADDR_MODE_7BITS);
    I2C_disableFIFO(TMP75C_BASE);
    I2C_setEmulationMode(TMP75C_BASE,I2C_EMULATION_FREE_RUN);
    I2C_enableModule(TMP75C_BASE);
}

void I2CB_Init(void)
{

    GPIO_setPinConfig(GPIO_40_I2CB_SDA);
    GPIO_setPadConfig(40, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(40, GPIO_QUAL_ASYNC);

    //
    // Configuration for the I2CB SCL Pin
    //
    GPIO_setPinConfig(GPIO_41_I2CB_SCL);
    GPIO_setPadConfig(41, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(41, GPIO_QUAL_ASYNC);

    I2C_disableModule(POWERBOARD_BASE);
    I2C_initController(POWERBOARD_BASE, DEVICE_SYSCLK_FREQ, 100000, I2C_DUTYCYCLE_50);
    I2C_setConfig(POWERBOARD_BASE, I2C_CONTROLLER_SEND_MODE);
    I2C_disableLoopback(POWERBOARD_BASE);
    I2C_setOwnAddress(POWERBOARD_BASE, 0x0);
    I2C_setTargetAddress(POWERBOARD_BASE, 0x4F);
    I2C_setBitCount(POWERBOARD_BASE,I2C_BITCOUNT_8);
    I2C_setDataCount(POWERBOARD_BASE,1);
    I2C_setAddressMode(POWERBOARD_BASE,I2C_ADDR_MODE_7BITS);
    I2C_disableFIFO(POWERBOARD_BASE);
    I2C_setEmulationMode(POWERBOARD_BASE,I2C_EMULATION_FREE_RUN);
    I2C_enableModule(POWERBOARD_BASE);
}

uint16_t I2CReadTwoBytes(uint32_t Base, uint16_t Address, uint16_t I2CAddress)
{
    uint16_t Value=0;

    if(WaitForI2CStopConditionStatus(Base, false, I2CWAIT_DEFAULT_TIMEOUT_ms) != I2CWAIT_SUCCESS) // Wait until Stop condition is cleared, with timeout
    {
        return 1*256;
    }

    I2C_setConfig(Base, I2C_CONTROLLER_SEND_MODE);              // Set up as master transmitter
    I2C_setTargetAddress(Base, I2CAddress);                   // Slave address of TMP75C is 0b1001111
    I2C_setDataCount(Base, 1);                             // Send Single Byte
    I2C_putData(Base, Address);                             // Transmit 0b0 to specify read of temperature register
    I2C_sendStartCondition(Base);                          // Send start condition to I2CB to begin transfer

    if(WaitForI2CStatus(Base, I2C_STS_TX_DATA_RDY, I2CWAIT_DEFAULT_TIMEOUT_ms) != I2CWAIT_SUCCESS) //Wait for data to go, with timeout
    {
        I2C_sendStopCondition(Base);                            // Send stop condition
        return 2*256;
    }

    I2C_setConfig(Base, I2C_CONTROLLER_RECEIVE_MODE);           // Set up as master receiver
    I2C_setDataCount(Base, 2);                             // Set to receive two bytes
    I2C_sendStartCondition(Base);                          // Send start condition to I2CB to begin transfer


    if(WaitForI2CStatus(Base, I2C_STS_RX_DATA_RDY, I2CWAIT_DEFAULT_TIMEOUT_ms) != I2CWAIT_SUCCESS) // Wait for byte to arrive, with timeout
    {
        I2C_sendStopCondition(Base);                            // Send stop condition
        return 3*256;
    }

    Value = I2C_getData(Base);                             // Read MSB into TempValue

    if(WaitForI2CStatus(Base, I2C_STS_RX_DATA_RDY, I2CWAIT_DEFAULT_TIMEOUT_ms) != I2CWAIT_SUCCESS) // Wait for byte to arrive, with timeout
    {
        I2C_sendStopCondition(Base);                            // Send stop condition
        return 4*256;
    }

    Value = (I2C_getData(Base) + (Value << 8));            // Read LSB into TempValue, left shift MSB
    I2C_sendStopCondition(Base);                            // Send stop condition
    return (Value);                                             // Return Value
}

uint16_t I2CReadSingleByte(uint32_t Base, uint16_t Address, uint16_t I2CAddress)
{
    uint16_t Value=0;

    if(WaitForI2CStopConditionStatus(Base, false, I2CWAIT_DEFAULT_TIMEOUT_ms) != I2CWAIT_SUCCESS) // Wait until Stop condition is cleared, with timeout
    {
        return 1*256;
    }

    I2C_setConfig(Base, I2C_CONTROLLER_SEND_MODE);         // Set up as master transmitter
    I2C_setTargetAddress(Base, I2CAddress);                // Set Slave Address
    I2C_setDataCount(Base, 1);                             // Send Single Byte
    I2C_putData(Base, Address);                            // Send Address to read
    I2C_sendStartCondition(Base);                          // Send start condition to I2CB to begin transfer

    if(WaitForI2CStatus(Base, I2C_STS_TX_DATA_RDY, I2CWAIT_DEFAULT_TIMEOUT_ms) != I2CWAIT_SUCCESS) //Wait for data to go, with timeout
    {
        I2C_sendStopCondition(Base);                       // Send stop condition
        return 2*256;
    }

    I2C_setConfig(Base, I2C_CONTROLLER_RECEIVE_MODE);      // Set up as master receiver
    I2C_setDataCount(Base, 1);                             // Set to receive a single byte
    I2C_sendStartCondition(Base);                          // Send start condition to I2CB to begin transfer


    if(WaitForI2CStatus(Base, I2C_STS_RX_DATA_RDY, I2CWAIT_DEFAULT_TIMEOUT_ms) != I2CWAIT_SUCCESS) // Wait for byte to arrive, with timeout
    {
        I2C_sendStopCondition(Base);                            // Send stop condition
        return 3*256;
    }

    Value = I2C_getData(Base);                             // Read MSB into TempValue

    I2C_sendStopCondition(Base);                            // Send stop condition
    return (Value);                                         // Return Value
}

uint16_t I2CWriteSingleByte(uint32_t Base, uint16_t Address, uint16_t I2CAddress, uint16_t Byte)
{
    if(WaitForI2CStopConditionStatus(Base, false, I2CWAIT_DEFAULT_TIMEOUT_ms) != I2CWAIT_SUCCESS) // Wait until Stop condition is cleared, with timeout
    {
        return 1*256;
    }

    I2C_setConfig(Base, I2C_CONTROLLER_SEND_MODE);          // Set up as master transmitter
    I2C_setTargetAddress(Base, I2CAddress);                 // Set Slave Address
    I2C_setDataCount(Base,2);                              // Send One Byte
    I2C_putData(Base, Address);                             // Transmit register address to I2C base address
    I2C_sendStartCondition(Base);                           // Send start condition to I2CB to begin transfer

    if(WaitForI2CStatus(Base, I2C_STS_TX_DATA_RDY, I2CWAIT_DEFAULT_TIMEOUT_ms) != I2CWAIT_SUCCESS) //Wait for data to go, with timeout
    {
        I2C_sendStopCondition(Base);                            // Send stop condition
        return 2*256;
    }

    I2C_putData(Base, Byte);                             // Transmit register address to I2C base address
    //I2C_sendStartCondition(Base);                           // Send start condition to I2CB to begin transfer

    if(WaitForI2CStatus(Base, I2C_STS_TX_DATA_RDY, I2CWAIT_DEFAULT_TIMEOUT_ms) != I2CWAIT_SUCCESS) //Wait for data to go, with timeout
    {
        I2C_sendStopCondition(Base);                            // Send stop condition
        return 3*256;
    }

    I2C_sendStopCondition(Base);                            // Send stop condition
    return 0;

}

uint16_t I2CWriteTwoBytes(uint32_t Base, uint16_t Address, uint16_t I2CAddress, uint16_t TwoBytes)
{
    if(WaitForI2CStopConditionStatus(Base, false, I2CWAIT_DEFAULT_TIMEOUT_ms) != I2CWAIT_SUCCESS) // Wait until Stop condition is cleared, with timeout
    {
        return 1*256;
    }

    I2C_setConfig(Base, I2C_CONTROLLER_SEND_MODE);          // Set up as master transmitter
    I2C_setTargetAddress(Base, I2CAddress);                 // Set Slave Address
    I2C_setDataCount(Base,1);                              // Send One Byte
    I2C_putData(Base, Address);                             // Transmit register address to I2C base address
    I2C_sendStartCondition(Base);                           // Send start condition to I2CB to begin transfer

    if(WaitForI2CStatus(Base, I2C_STS_TX_DATA_RDY, I2CWAIT_DEFAULT_TIMEOUT_ms) != I2CWAIT_SUCCESS) //Wait for data to go, with timeout
    {
        I2C_sendStopCondition(Base);                            // Send stop condition
        return 2*256;
    }

    I2C_putData(Base, TwoBytes>>8);                             // Transmit register address to I2C base address
    I2C_sendStartCondition(Base);                           // Send start condition to I2CB to begin transfer

    if(WaitForI2CStatus(Base, I2C_STS_TX_DATA_RDY, I2CWAIT_DEFAULT_TIMEOUT_ms) != I2CWAIT_SUCCESS) //Wait for data to go, with timeout
    {
        I2C_sendStopCondition(Base);                            // Send stop condition
        return 3*256;
    }

    I2C_putData(Base, TwoBytes);                             // Transmit register address to I2C base address
    I2C_sendStartCondition(Base);                           // Send start condition to I2CB to begin transfer
    if(WaitForI2CStatus(Base, I2C_STS_TX_DATA_RDY, I2CWAIT_DEFAULT_TIMEOUT_ms) != I2CWAIT_SUCCESS) //Wait for data to go, with timeout
    {
        I2C_sendStopCondition(Base);                            // Send stop condition
        return 4*256;
    }

    I2C_sendStopCondition(Base);                            // Send stop condition
    return 0;

}

//! \brief                  Waits until the I2C peripheral has a particular stop condition status, with a timeout
//! \param[in] base         The base address of the I2C instance used.
//! \param[in] status       The status to look for
//! \return                 I2CWAIT_SUCCESS if there was no timeout, I2CWAIT_ERROR_TIMEOUT in the case of a timeout
//!
static uint16_t WaitForI2CStopConditionStatus(uint32_t base, bool status, uint16_t timeout_ms){
    uint64_t startTime;
    uint16_t elapsedTime_ms;

    startTime = BGTimer_getCurrentTime();
    elapsedTime_ms = 0;
    while(elapsedTime_ms < timeout_ms)
    {
        if(I2C_getStopConditionStatus(base) == status)
        {
            return I2CWAIT_SUCCESS;
        }
        elapsedTime_ms = (uint16_t)BGTimer_getElapsedTime_ms(startTime);
    }
    //If we get here the timeout has expired
    return I2CWAIT_ERROR_TIMEOUT;
}

//! \brief                  Waits until the I2C peripheral has a particular status, with a timeout
//! \param[in] base         The base address of the I2C instance used.
//! \param[in] status       The status to look for (as a bit mask), e.g. I2C_STS_TX_DATA_RDY
//! \return                 I2CWAIT_SUCCESS if there was no timeout, I2CWAIT_ERROR_TIMEOUT in the case of a timeout
//!
static uint16_t WaitForI2CStatus(uint32_t base, uint16_t status, uint16_t timeout_ms){
    uint64_t startTime;
    uint16_t elapsedTime_ms;

    startTime = BGTimer_getCurrentTime();
    elapsedTime_ms = 0;
    while(elapsedTime_ms < timeout_ms)
    {
        if(I2C_getStatus(base) & status)
        {
            return I2CWAIT_SUCCESS;
        }
        elapsedTime_ms = (uint16_t)BGTimer_getElapsedTime_ms(startTime);
    }
    //If we get here the timeout has expired
    return I2CWAIT_ERROR_TIMEOUT;
}
