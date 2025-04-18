/*
 * SLS_CANBUS.c
 *
 *  Created on: 4th April 2025
 *      Author: Iainmosely
 */


#include "device.h"
#include <board.h>
#include <SLS_CANBUS.h>
#include <SLS_ADC.h>
#include <SLS_THERMAL.h>
#include <SLS_SPI.h>

__interrupt void INT_SLS_CANA_0_ISR(void);
__interrupt void INT_SLS_CANA_1_ISR(void);

#define MSG_DATA_LENGTH     8                   // All CAN messages are 8 bytes long
#define Message1            1
#define Message2            2
#define Message3            3
#define Message4            4
#define Message5            5
#define Message6            6
#define Message7            7
#define Message8            8

#define CANCLOCKRATE        500000              // CANBUS Clock Rate in Hz
#define CAN_ABO_Time_ms     50U                 // Time to wait after Bus off before executing auto-bus-on feature
#define CAN_Msg_Period_ms   50U                 // Time between sending of successive CAN message batches

#define Message6RxTimeout_ms    500U            // Max allowable time since last Message6 receipt before an error is flagged
#define CAN_BASE_ADDRESS 0x100                  // CANBUS Base Address for Data Transmission

//DBC Definitions

#define IALGain         256.0f
#define IALOffset       128.0f
#define IAINGain        256.0f
#define IAINOffset      128.0f
#define IAOUTGain       256.0f
#define IAOUTOffset     128.0f
#define VAOUTGain       64.0f
#define VAOUTOffset     0.0f

#define IBLGain         256.0f
#define IBLOffset       128.0f
#define IBINGain        256.0f
#define IBINOffset      128.0f
#define IBOUTGain       256.0f
#define IBOUTOffset     128.0f
#define VBOUTGain       64.0f
#define VBOUTOffset     0.0f

#define VDCGain         64.0f
#define VDCOffset       0.0f
#define IDCGain         256.0f
#define IDCOffset       128.0f

#define P3V3Gain        2048.0f
#define P3V3Offset      0.0f
#define P5VGain         2048.0f
#define P5VOffset       0.0f
#define P24VGain        2048.0f
#define P24VOffset      0.0f
#define P1V25Gain       2048.0f
#define P1V25Offset     0.0f
#define P625mVGain      2048.0f
#define P625mVOffset    0.0f

#define CONTTEMPGain    256.0f
#define CONTTEMPOffset  128.0f
#define FAN1TEMPGain    256.0f
#define FAN1TEMPOffset  128.0f
#define FAN2TEMPGain    256.0f
#define FAN2TEMPOffset  128.0f
#define FAN1DutyGain    512.0f
#define FAN1DutyOffset  0.0f
#define FAN2DutyGain    512.0f
#define FAN2DutyOffset  0.0f
#define FAN1SpeedGain   2.0f
#define FAN1SpeedOffset 0.0f
#define FAN2SpeedGain   2.0f
#define FAN2SpeedOffset 0.0f

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
    CAN_enableInterrupt(SLS_CANA_BASE, CAN_INT_IE0 | CAN_INT_ERROR | CAN_INT_STATUS);
    CAN_setupMessageObject(SLS_CANA_BASE, Message1, (CAN_BASE_ADDRESS + 0x1), CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH);
    CAN_setupMessageObject(SLS_CANA_BASE, Message2, (CAN_BASE_ADDRESS + 0x2), CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH);
    CAN_setupMessageObject(SLS_CANA_BASE, Message3, (CAN_BASE_ADDRESS + 0x3), CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH);
    CAN_setupMessageObject(SLS_CANA_BASE, Message4, (CAN_BASE_ADDRESS + 0x4), CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH);
    CAN_setupMessageObject(SLS_CANA_BASE, Message5, (CAN_BASE_ADDRESS + 0x5), CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH);
    CAN_setupMessageObject(SLS_CANA_BASE, Message6, (CAN_BASE_ADDRESS + 0x6), CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH);
    CAN_setupMessageObject(SLS_CANA_BASE, Message7, (CAN_BASE_ADDRESS + 0x7), CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH);
    CAN_setupMessageObject(SLS_CANA_BASE, Message8, (CAN_BASE_ADDRESS + 0x8), CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE, MSG_DATA_LENGTH);
    Interrupt_register(INT_CANA0, &INT_SLS_CANA_0_ISR);
}

void PackageCANData(void)
{

    CAN1.SLSMsg1.IA_L = (uint16_t)((GetIA_L()+IALOffset)*IALGain);
    CAN1.SLSMsg1.IA_IN = (uint16_t)((GetIA_IN()+IAINOffset)*IAINGain);
    CAN1.SLSMsg1.IA_OUT = (uint16_t)((GetIA_OUT()+IAOUTOffset)*IAOUTGain);
    CAN1.SLSMsg1.VA_OUT = (uint16_t)((GetVA_OUT()+VAOUTOffset)*VAOUTGain);

    CAN1.SLSMsg2.IB_L = (uint16_t)((GetIB_L()+IBLOffset)*IBLGain);
    CAN1.SLSMsg2.IB_IN = (uint16_t)((GetIB_IN()+IBINOffset)*IBINGain);
    CAN1.SLSMsg2.IB_OUT = (uint16_t)((GetIB_OUT()+IBOUTOffset)*IBOUTGain);
    CAN1.SLSMsg2.VB_OUT = (uint16_t)((GetVB_OUT()+VBOUTOffset)*VBOUTGain);

    CAN1.SLSMsg3.VDC = (uint16_t)((GetVDC()+VDCOffset)*VDCGain);
    CAN1.SLSMsg3.IDC = (uint16_t)((GetIDC()+IDCOffset)*IDCGain);
    CAN1.SLSMsg3.THERMA = 0;
    CAN1.SLSMsg3.THERMB = 0;

    CAN1.SLSMsg4.P3V3 = (uint16_t)((Get3V3()+P3V3Offset)*P3V3Gain);
    CAN1.SLSMsg4.P5V = (uint16_t)((Get5V()+P5VOffset)*P5VGain);
    CAN1.SLSMsg4.P24V = (uint16_t)((Get24V()+P24VOffset)*P24VGain);
    CAN1.SLSMsg4.P1V25 = (uint16_t)((GetMidscaleVoltage()+P1V25Offset)*P1V25Gain);

    CAN1.SLSMsg5.P625MV = (uint16_t)((GetQuarterScaleVoltage()+P625mVOffset)*P625mVGain);
    CAN1.SLSMsg5.CONTTEMP = (uint16_t)((GetCONTTEMP()+CONTTEMPOffset)*CONTTEMPGain);
    CAN1.SLSMsg5.FAN1DUTY = 0;
    CAN1.SLSMsg5.FAN2DUTY = 0;

    CAN1.SLSMsg6.FAN1TEMP = (uint16_t)((GetFan1LocalTemp()+FAN1TEMPOffset)*FAN1TEMPGain);
    CAN1.SLSMsg6.FAN2TEMP = 0;
    CAN1.SLSMsg6.FAN1SPEED = (uint16_t)((GetFan1Speed()+FAN1SpeedOffset)*FAN1SpeedGain);
    CAN1.SLSMsg6.FAN2SPEED = 0;

    CAN1.SLSMsg7.FAULTSTATUS = 0;
    CAN1.SLSMsg7.POWERCYCLES = GetPowerCycles();
    CAN1.SLSMsg7.STATE=0;



}

void TransmitCANMessage(void)
{
    uint16_t txMsgData[8]={0,0,0,0,0,0,0,0};

    // Form data for Message 1 and Transmit
    txMsgData[7] = CAN1.SLSMsg1.IA_L & 0xFF;
    txMsgData[6] = (CAN1.SLSMsg1.IA_L & 0xFF00)>>8;
    txMsgData[5] = CAN1.SLSMsg1.IA_IN & 0xFF;
    txMsgData[4] = (CAN1.SLSMsg1.IA_IN & 0xFF00)>>8;
    txMsgData[3] = CAN1.SLSMsg1.IA_OUT & 0xFF;
    txMsgData[2] = (CAN1.SLSMsg1.IA_OUT & 0xFF00)>>8;
    txMsgData[1] = CAN1.SLSMsg1.VA_OUT & 0xFF;
    txMsgData[0] = (CAN1.SLSMsg1.VA_OUT & 0xFF00)>>8;
    CAN_sendMessage(SLS_CANA_BASE, Message1, MSG_DATA_LENGTH,txMsgData);

    // Form data for Message 2 and Transmit
    txMsgData[7] = CAN1.SLSMsg2.IB_L & 0xFF;
    txMsgData[6] = (CAN1.SLSMsg2.IB_L & 0xFF00)>>8;
    txMsgData[5] = CAN1.SLSMsg2.IB_IN & 0xFF;
    txMsgData[4] = (CAN1.SLSMsg2.IB_IN & 0xFF00)>>8;
    txMsgData[3] = CAN1.SLSMsg2.IB_OUT & 0xFF;
    txMsgData[2] = (CAN1.SLSMsg2.IB_OUT & 0xFF00)>>8;
    txMsgData[1] = CAN1.SLSMsg2.VB_OUT & 0xFF;
    txMsgData[0] = (CAN1.SLSMsg2.VB_OUT & 0xFF00)>>8;
    CAN_sendMessage(SLS_CANA_BASE, Message2, MSG_DATA_LENGTH,txMsgData);

    // Form data for Message 3 and Transmit
    txMsgData[7] = CAN1.SLSMsg3.VDC & 0xFF;
    txMsgData[6] = (CAN1.SLSMsg3.VDC & 0xFF00)>>8;
    txMsgData[5] = CAN1.SLSMsg3.IDC & 0xFF;
    txMsgData[4] = (CAN1.SLSMsg3.IDC & 0xFF00)>>8;
    txMsgData[3] = CAN1.SLSMsg3.THERMA & 0xFF;
    txMsgData[2] = (CAN1.SLSMsg3.THERMA & 0xFF00)>>8;
    txMsgData[1] = CAN1.SLSMsg3.THERMB & 0xFF;
    txMsgData[0] = (CAN1.SLSMsg3.THERMB & 0xFF00)>>8;
    CAN_sendMessage(SLS_CANA_BASE, Message3, MSG_DATA_LENGTH,txMsgData);

    // Form data for Message 4 and Transmit
    txMsgData[7] = CAN1.SLSMsg4.P3V3 & 0xFF;
    txMsgData[6] = (CAN1.SLSMsg4.P3V3 & 0xFF00)>>8;
    txMsgData[5] = CAN1.SLSMsg4.P5V & 0xFF;
    txMsgData[4] = (CAN1.SLSMsg4.P5V & 0xFF00)>>8;
    txMsgData[3] = CAN1.SLSMsg4.P24V & 0XFF;
    txMsgData[2] = (CAN1.SLSMsg4.P24V & 0xFF00)>>8;
    txMsgData[1] = CAN1.SLSMsg4.P1V25 & 0xFF;
    txMsgData[0] = (CAN1.SLSMsg4.P1V25 & 0xFF00)>>8;
    CAN_sendMessage(SLS_CANA_BASE, Message4, MSG_DATA_LENGTH,txMsgData);

    // Form data for Message 5 and Transmit
    txMsgData[7] = CAN1.SLSMsg5.P625MV & 0xFF;
    txMsgData[6] = (CAN1.SLSMsg5.P625MV & 0xFF00)>>8;
    txMsgData[5] = CAN1.SLSMsg5.CONTTEMP & 0xFF;
    txMsgData[4] = (CAN1.SLSMsg5.CONTTEMP & 0xFF00)>>8;
    txMsgData[3] = CAN1.SLSMsg5.FAN1DUTY & 0XFF;
    txMsgData[2] = (CAN1.SLSMsg5.FAN1DUTY & 0xFF00)>>8;
    txMsgData[1] = CAN1.SLSMsg5.FAN2DUTY & 0xFF;
    txMsgData[0] = (CAN1.SLSMsg5.FAN2DUTY & 0xFF00)>>8;
    CAN_sendMessage(SLS_CANA_BASE, Message5, MSG_DATA_LENGTH,txMsgData);

    // Form data for Message 6 and Transmit
    txMsgData[7] = CAN1.SLSMsg6.FAN1TEMP & 0xFF;
    txMsgData[6] = (CAN1.SLSMsg6.FAN1TEMP & 0xFF00)>>8;
    txMsgData[5] = CAN1.SLSMsg6.FAN2TEMP & 0xFF;
    txMsgData[4] = (CAN1.SLSMsg6.FAN2TEMP & 0xFF00)>>8;
    txMsgData[3] = CAN1.SLSMsg6.FAN1SPEED & 0XFF;
    txMsgData[2] = (CAN1.SLSMsg6.FAN1SPEED & 0xFF00)>>8;
    txMsgData[1] = CAN1.SLSMsg6.FAN2SPEED & 0xFF;
    txMsgData[0] = (CAN1.SLSMsg6.FAN2SPEED & 0xFF00)>>8;
    CAN_sendMessage(SLS_CANA_BASE, Message6, MSG_DATA_LENGTH,txMsgData);

    // Form data for Message 7 and Transmit
    txMsgData[7] = CAN1.SLSMsg7.FAULTSTATUS & 0xFF;
    txMsgData[6] = (CAN1.SLSMsg7.FAULTSTATUS & 0xFF00)>>8;
    txMsgData[5] = CAN1.SLSMsg7.STATE & 0xFF;
    txMsgData[4] = CAN1.SLSMsg7.POWERCYCLES & 0xFF;
    txMsgData[3] = (CAN1.SLSMsg7.POWERCYCLES & 0xFF00)>>8;
    txMsgData[2] = 0;
    txMsgData[1] = 0;
    txMsgData[0] = 0;
    CAN_sendMessage(SLS_CANA_BASE, Message7, MSG_DATA_LENGTH,txMsgData);
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


__interrupt void INT_SLS_CANA_0_ISR(void)
{
    uint32_t status=0;


    //
    // Read the CAN-A interrupt status (in the CAN_INT register) to find the
    // cause of the interrupt
    //
    status = CAN_getInterruptCause(SLS_CANA_BASE);

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
        status = CAN_getStatus(SLS_CANA_BASE);  // Return CAN_ES value.
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
    else if(status == Message8)
    {
        //
        // Get the received message
        //
        CAN_readMessage(SLS_CANA_BASE, Message8, rxMsgData);

        CAN1.SLSMsg8.REQUESTSTATE=rxMsgData[0];
        CAN1.SLSMsg8.FAN1DUTY=rxMsgData[1];
        SetFan1DutyCycle((float)CAN1.SLSMsg8.FAN1DUTY);
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
        CAN_clearInterruptStatus(SLS_CANA_BASE, Message8);

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
    CAN_clearGlobalInterruptStatus(SLS_CANA_BASE, CAN_GLOBAL_INT_CANINT0);

    //
    // Acknowledge this interrupt located in group 9
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

__interrupt void INT_SLS_CANA_1_ISR(void)
{

}

