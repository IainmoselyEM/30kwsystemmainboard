/*
 * SLS_CANBUS.c
 *
 *  Created on: 4th April 2025
 *      Author: Iainmosely
 */


#include "device.h"
#include <board.h>
#include <SLS_CANBUS.h>

__interrupt void canaISR(void);

#define MSG_DATA_LENGTH     8                   // All CAN messages are 8 bytes long
#define Message1            1
#define Message2            2
#define Message3            3
#define Message4            4
#define Message5            5
#define Message6            6

#define CANCLOCKRATE        500000              // CANBUS Clock Rate in Hz
#define CAN_ABO_Time_ms     50U                 // Time to wait after Bus off before executing auto-bus-on feature
#define CAN_Msg_Period_ms   50U                 // Time between sending of successive CAN message batches

#define Message6RxTimeout_ms    500U            // Max allowable time since last Message6 receipt before an error is flagged
#define CAN_BASE_ADDRESS 0x361                  // CANBUS Base Address for Data Transmission


volatile uint32_t errorFlag = 0;
uint16_t rxMsgData[8];
PwrCAN CAN1;
//PowerContState CANBUSCommandState;
//static uint64_t Message6LastRxd;
//const errors_union CANTimeoutErr = {.as_struct.CANTimeout_Message6 = true};

//PowerContState GetCANBUSCommandState(void)
//{
//    return(CANBUSCommandState);
//}

void Init_CANA(void)
{
    CAN_enableInterrupt(myCAN0_BASE, CAN_INT_IE0 | CAN_INT_ERROR | CAN_INT_STATUS);
    CAN_setupMessageObject(myCAN0_BASE, Message1, 0x380, CAN_MSG_FRAME_EXT, CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH);
    CAN_setupMessageObject(myCAN0_BASE, Message2, (CAN_BASE_ADDRESS + 0x1), CAN_MSG_FRAME_EXT, CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH);
    CAN_setupMessageObject(myCAN0_BASE, Message3, (CAN_BASE_ADDRESS + 0x2), CAN_MSG_FRAME_EXT, CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH);
    CAN_setupMessageObject(myCAN0_BASE, Message4, (CAN_BASE_ADDRESS + 0x3), CAN_MSG_FRAME_EXT, CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH);
    CAN_setupMessageObject(myCAN0_BASE, Message5, (CAN_BASE_ADDRESS + 0x4), CAN_MSG_FRAME_EXT, CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH);
    CAN_setupMessageObject(myCAN0_BASE, Message6, CAN_BASE_ADDRESS, CAN_MSG_FRAME_EXT, CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE, MSG_DATA_LENGTH);
    Interrupt_register(INT_CANA0, &canaISR);
}

void PackageCANData(void)
{
//    float CANValue=0;
//
//    CANValue=(uint16_t)(GetICommand()*IOUTScaling)+IDEMANDOffset;
//    CAN1.PwrContMsg2.PwrContCommandCurrIdem = (uint16_t)fmaxf(0.0f,fminf(CANValue,powf(2.0f,IDEMANDbits)));         // Package IDEMAND with offset, scaling and limit checking
//    CANValue=(GetIIND()*IINDScaling)+IINDOffset;
//    CAN1.PwrContMsg2.PwrContInductorCurrIind = (uint16_t)fmaxf(0.0f,fminf(CANValue,powf(2.0f,IINDbits)));          // Package IIND with offset, scaling and limit checking
//    CANValue=(GetIIN()*IINScaling)+IINOffset;
//    CAN1.PwrContMsg2.PwrContInputCurrIin = (uint16_t)fmaxf(0.0f,fminf(CANValue,powf(2.0f,IINbits)));                // Package IIN with offset, scaling and limit checking
//    CANValue=(GetIOUT()*IOUTScaling)+IOUTOffset;
//    CAN1.PwrContMsg2.PwrContOutputCurrIout = (uint16_t)fmaxf(0.0f,fminf(CANValue,powf(2.0f,IOUTbits)));            // Package IOUT with offset, scaling and limit checking
//    CANValue=(GetVIN()*VINScaling)+VINOffset;
//    CAN1.PwrContMsg3.PwrContInputVoltageVin = (uint16_t)fmaxf(0.0f,fminf(CANValue,powf(2.0f,VINbits)));             // Package VIN with offset, scaling and limit checking
//    CANValue=(GetVOUT()*VOUTScaling)+VOUTOffset;
//    CAN1.PwrContMsg3.PwrContOutputVoltageVout = (uint16_t)fmaxf(0.0f,fminf(CANValue,powf(2.0f,VOUTbits)));         // Package VOUT with offset, scaling and limit checking
//    CANValue=(GetIBATTCharge()*IBATTScaling)+IBATTOffset;
//    CAN1.PwrContMsg4.PwrCont12VBattCurrIbatt = (uint16_t)fmaxf(0.0f,fminf(CANValue,powf(2.0f,IBATTbits)));        // Package IBATT with offset, scaling and limit checking
//    CANValue=(GetVBATT()*VBATTScaling)+VBATTOffset;
//    CAN1.PwrContMsg4.PwrCont12VSupplyVbatt = (uint16_t)fmaxf(0.0f,fminf(CANValue,powf(2.0f,VBATTbits)));          // Package VBATT with offset, scaling and limit checking
//    CANValue=(GetSOCBATT()*BATTSOCScaling)+BATTSOCOffset;
//    CAN1.PwrContMsg4.PwrCont12VBattSoc = (uint16_t)fmaxf(0.0f,fminf(CANValue,powf(2.0f,BATTSOCbits)));          // Package BATTSOC with offset, scaling and limit checking
//    CANValue=(GetVBIAS()*VBIASScaling)+VBIASOffset;
//    CAN1.PwrContMsg4.PwrContBiasSupplyVbias = (uint16_t)fmaxf(0.0f,fminf(CANValue,powf(2.0f,VBIASbits)));         // Package VBIAS with offset, scaling and limit checking
//    CAN1.PwrContMsg4.PwrContStatus = (uint8_t)GetCurrentState();
//    CANValue=(GetCONTTEMP()*CONTTEMPScaling)+CONTTEMPOffset;
//    CAN1.PwrContMsg5.PwrContControlBrdTemp = (uint8_t)fmaxf(0.0f,fminf(CANValue,powf(2.0f,CONTTEMPbits)));     // Package CONTTEMP with offset, scaling and limit checking
//    CANValue=(GetMODTEMPA()*MODTEMPAScaling)+MODTEMPAOffset;
//    CAN1.PwrContMsg5.PwrContModATemp = (uint8_t)fmaxf(0.0f,fminf(CANValue,powf(2.0f,MODTEMPAbits)));           // Package MODTEMPA with offset, scaling and limit checking
//    CANValue=(GetMODTEMPB()*MODTEMPBScaling)+MODTEMPBOffset;
//    CAN1.PwrContMsg5.PwrContModBTemp = (uint8_t)fmaxf(0.0f,fminf(CANValue,powf(2.0f,MODTEMPBbits)));           // Package MODTEMPB with offset, scaling and limit checking
}

void TransmitCANMessage(void)
{
    uint16_t txMsgData[8]={0,0,0,0,0,0,0,0};

    // Form data for Message 1 and Transmit(0x361?)
    //CAN_sendMessage(CANA_BASE, Message1, MSG_DATA_LENGTH,txMsgData);

    // Form data for Message 2 and Transmit(0x362)
    txMsgData[7] = CAN1.PwrContMsg2.PwrContCommandCurrIdem & 0xFF;
    txMsgData[6] = (CAN1.PwrContMsg2.PwrContCommandCurrIdem & 0xFF00)>>8;
    txMsgData[5] = CAN1.PwrContMsg2.PwrContInductorCurrIind & 0xFF;
    txMsgData[4] = (CAN1.PwrContMsg2.PwrContInductorCurrIind & 0xFF00)>>8;
    txMsgData[3] = CAN1.PwrContMsg2.PwrContInputCurrIin & 0xFF;
    txMsgData[2] = (CAN1.PwrContMsg2.PwrContInputCurrIin & 0xFF00)>>8;
    txMsgData[1] = CAN1.PwrContMsg2.PwrContOutputCurrIout & 0xFF;
    txMsgData[0] = (CAN1.PwrContMsg2.PwrContOutputCurrIout & 0xFF00)>>8;
    CAN_sendMessage(myCAN0_BASE, Message2, MSG_DATA_LENGTH,txMsgData);

    // Form data for Message 3 and Transmit(0x363)
    txMsgData[7] = CAN1.PwrContMsg3.PwrContInputVoltageVin & 0xFF;
    txMsgData[6] = (CAN1.PwrContMsg3.PwrContInputVoltageVin & 0xFF00)>>8;
    txMsgData[5] = CAN1.PwrContMsg3.PwrContMaxPwrLimitPmax & 0xFF;
    txMsgData[4] = (CAN1.PwrContMsg3.PwrContMaxPwrLimitPmax & 0xFF00)>>8;
    txMsgData[3] = CAN1.PwrContMsg3.PwrContMaxVoltageVmax & 0xFF;
    txMsgData[2] = (CAN1.PwrContMsg3.PwrContMaxVoltageVmax & 0xFF00)>>8;
    txMsgData[1] = CAN1.PwrContMsg3.PwrContOutputVoltageVout & 0xFF;
    txMsgData[0] = (CAN1.PwrContMsg3.PwrContOutputVoltageVout & 0xFF00)>>8;

    CAN_sendMessage(myCAN0_BASE, Message3, MSG_DATA_LENGTH,txMsgData);

    // Form data for Message 4 and Transmit(0x364)
    txMsgData[7] = CAN1.PwrContMsg4.PwrContStatus & 0xFF;
    txMsgData[6] = CAN1.PwrContMsg4.PwrCont12VBattCurrIbatt & 0xFF;
    txMsgData[5] = (CAN1.PwrContMsg4.PwrCont12VBattCurrIbatt & 0xFF00)>>8;
    txMsgData[4] = CAN1.PwrContMsg4.PwrCont12VSupplyVbatt & 0xFF;
    txMsgData[3] = (((CAN1.PwrContMsg4.PwrContBiasSupplyVbias & 0xF)<<4) | ((CAN1.PwrContMsg4.PwrCont12VSupplyVbatt & 0xF00)>>8));
    txMsgData[2] = (CAN1.PwrContMsg4.PwrContBiasSupplyVbias & 0xFF0)>>4;
    txMsgData[1] = CAN1.PwrContMsg4.PwrCont12VBattSoc & 0xFF;
    txMsgData[0] = (CAN1.PwrContMsg4.PwrCont12VBattSoc & 0xF00)>>8;
    CAN_sendMessage(CANA_BASE, Message4, MSG_DATA_LENGTH,txMsgData);

    // Form data for Message 5 and Transmit(0x365)
    txMsgData[7] = CAN1.PwrContMsg5.PwrContCmdStructure & 0xFF;
    txMsgData[6] = CAN1.PwrContMsg5.PwrContControlBrdTemp & 0xFF;
    txMsgData[5] = CAN1.PwrContMsg5.PwrContModATemp & 0xFF;
    txMsgData[4] = CAN1.PwrContMsg5.PwrContModBTemp & 0xFF;
    txMsgData[3] = 0;
    txMsgData[2] = 0;
    txMsgData[1] = 0;
    txMsgData[0] = 0;
    CAN_sendMessage(myCAN0_BASE, Message5, MSG_DATA_LENGTH,txMsgData);
}

//! \brief  Call this function repeatedly to gather and send main CAN data
//!
//void CANMessengerTick(void)
//{
//    uint64_t currentElapsedTime_ms;
//
//    static uint64_t lastMessageTime = 0; //declared static so it persists between function calls
//
//    currentElapsedTime_ms = BGTimer_getElapsedTime_ms(lastMessageTime);
//    if(currentElapsedTime_ms >=  CAN_Msg_Period_ms) //if it is time for a new message transmission
//    {
//        //record the current time
//        lastMessageTime = BGTimer_getCurrentTime();
//
//        //package and transmit CAN data
//        PackageCANData();
//        TransmitCANMessage();
//    }
//}


//TODO: Make a way to check for a specific message having been Tx'd successfully.
//      The function below checks register for the whole CAN module. The field TXOK for example can be set by any message having been successfully tx'd.
//      The flags in the message object can be used to determine if a particular message has been successfully sent.
//      (Note: see section 21.11.3 Transmission of Messages in Event Driven CAN Communication in SPRUHX5G - TMS320F2837xS Microcontrollers Technical Reference Manual for details)
//      To access them one needs to go via an interface register, similar to the CAN_readMessage(..) implementation. An interrupt that fires on successful TX may be another approach.

//! \brief                  Waits until the CAN peripheral has a particular status, with a timeout
//! \param[in] base         The base address of the CAN instance used.
//! \param[in] status       The status to look for (as a bit mask), e.g. CAN_ES_TXOK
//! \return                 CANWAIT_SUCCESS if there was no timeout, CANWAIT_ERROR_TIMEOUT in the case of a timeout
//!
//uint16_t WaitForCANStatus(uint32_t base, uint16_t status, uint16_t timeout_ms){
//    uint64_t startTime;
//    uint16_t elapsedTime_ms;
//
//    startTime = BGTimer_getCurrentTime();
//    elapsedTime_ms = 0;
//    while(elapsedTime_ms < timeout_ms)
//    {
//        if(CAN_getStatus(base) & status)
//        {
//            return CANWAIT_SUCCESS;
//        }
//        elapsedTime_ms = (uint16_t)BGTimer_getElapsedTime_ms(startTime);
//    }
//    //If we get here the timeout has expired
//    return CANWAIT_ERROR_TIMEOUT;
//}


// Call this function periodically when required to monitor message6 for timeouts
//void CheckForCANBUSMessage6Timeout()
//{
//        uint64_t timeSinceLastMessage6;
//
//        // work out time since last Message6 was received
//        timeSinceLastMessage6 = BGTimer_getElapsedTime_ms(Message6LastRxd);
//        // set timeout error appropriately (N.B. CAN rx interrupt code will clear the error)
//        if(timeSinceLastMessage6 > Message6RxTimeout_ms)
//        {
//            #ifndef DISABLE_CAN_MESSAGE6_TIMEOUT
//                SetErrors(CANTimeoutErr);
//            #endif
//        }
//}


__interrupt void canaISR(void)
{
    uint32_t status=0;


    //
    // Read the CAN-A interrupt status (in the CAN_INT register) to find the
    // cause of the interrupt
    //
    status = CAN_getInterruptCause(myCAN0_BASE);

    //
    // If the cause is a controller status interrupt, then get the status.
    // During first iteration of every ISR execution, status = 0x8000,
    // which simply means CAN_ES != 0x07.
    //
    if(status == CAN_INT_INT0ID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.
        //
        status = CAN_getStatus(myCAN0_BASE);  // Return CAN_ES value.
        //
        // Now status = 0x00000010, indicating RxOK.
        //

        //
        // Check to see if an error occurred.
        //
        if(((status  & ~(CAN_STATUS_RXOK)) != CAN_STATUS_LEC_MSK) &&
           ((status  & ~(CAN_STATUS_RXOK)) != CAN_STATUS_LEC_NONE))
        {
            //
            // Set a flag to indicate some errors may have occurred.
            //
            errorFlag = 1;
        }
    }
    //
    // Check if the cause is the CAN-A receive message object IDEMAND_MSG_OBJ_ID . Will be skipped
    // in the first iteration of every ISR execution
    //
    else if(status == Message6)
    {
        //
        // Get the received message
        //
        CAN_readMessage(myCAN0_BASE, Message6, rxMsgData);

        //Set demand current and voltage
       // SetDemandCurrent(((((float)((rxMsgData[1]<<8)|rxMsgData[0]))-IDemandOffset)/IDemandScaling));
       // SetVoltage((float)((rxMsgData[5]<<8)|rxMsgData[4]) / VDemandScaling);
        //Set requested state
       // CANBUSCommandState = (PowerContState)rxMsgData[7];


        // Record the time at which Message6 was last received
        //Message6LastRxd = BGTimer_getCurrentTime();
        // Clear any active timeout error
        //ClearErrors(CANTimeoutErr);

        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message RX is complete.  Clear the
        // message object interrupt.
        //
        CAN_clearInterruptStatus(myCAN0_BASE, Message6);

        //
        // Since the message was received, clear any error flags.
        //
        errorFlag = 0;
    }
    //
    // If something unexpected caused the interrupt, this would handle it.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
    }

    //
    // Clear the global interrupt flag for the CAN interrupt line
    //
    CAN_clearGlobalInterruptStatus(myCAN0_BASE, CAN_GLOBAL_INT_CANINT0);

    //
    // Acknowledge this interrupt located in group 9
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

