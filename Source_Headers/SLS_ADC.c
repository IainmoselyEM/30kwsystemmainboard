/*
 * PE_ADC.c
 *
 *  Created on: 30 Aug 2024
 *      Author: IainMosely
 */

#include "device.h"
#include <board.h>
#include <SLS_ADC.h>
#include "driverlib.h"

#define MIDSCALE_FS    2.5f
#define QUARTERSCALE_FS 2.5f
#define IA_L_FS 407.6086957f
#define IA_OUT_FS 407.6086957f
#define IA_IN_FS 407.6086957f
#define VA_OUT_FS 1251.25f
#define VDC_FS 1251.25f
#define IDC_FS 407.6086957f
#define RAIL3V3_FS 3.775f
#define RAIL5V_FS 7.5f
#define IB_L_FS 407.6086957f
#define IB_OUT_FS 407.6086957f
#define IB_IN_FS 407.6086957f
#define VB_OUT_FS 1251.25f
#define RAIL24V_FS 27.5f

float REF1V25_BUF_pu=0;
float REF625mV_pu=0;
float IA_L_pu=0;
float IA_OUT_pu=0;
float IA_IN_pu=0;
float VA_OUT_pu=0;
float VDC_pu=0;
float IDC_pu=0;
float RAIL3V3_pu=0;
float RAIL5V_pu=0;
float IB_L_pu=0;
float IB_OUT_pu=0;
float IB_IN_pu=0;
float VB_OUT_pu=0;
float RAIL24V_pu=0;
float THERMA_pu=0;
float THERMB_pu=0;


// Midscale voltage in mV
float GetMidscaleVoltage(void)
{
    return(REF1V25_BUF_pu*QUARTERSCALE_FS*1000.0f);
}

// Quarterscale voltage in mV
float GetQuarterScaleVoltage(void)
{
    return(REF625mV_pu*MIDSCALE_FS*1000.0f);
}

// Power Stage A Inductor Current in A
float GetIA_L(void)
{
    return(IA_L_pu*IA_L_FS);
}

// Power Stage A Output Current in A
float GetIA_OUT(void)
{
    return(IA_OUT_pu*IA_OUT_FS);
}

// Power Stage A Input Current in A
float GetIA_IN(void)
{
    return(IA_IN_pu*IA_IN_FS);
}

// Power Stage A Output Voltage V
float GetVA_OUT(void)
{
    return(VA_OUT_pu*VA_OUT_FS);
}

// Common DC Link Voltage V
float GetVDC(void)
{
    return(VDC_pu*VDC_FS);
}

// Common DC Link Supply Current A
float GetIDC(void)
{
    return(IDC_pu*IDC_FS);
}

// 3V3 Rail Voltage mV
float Get3V3(void)
{
    return(RAIL3V3_pu*RAIL3V3_FS*1000.0f);
}

// 5V Rail Voltage mV
float Get5V(void)
{
    return(RAIL5V_pu*RAIL5V_FS*1000.0f);
}

// Power Stage B Inductor Current in A
float GetIB_L(void)
{
    return(IB_L_pu*IB_L_FS);
}

// Power Stage B Output Current in A
float GetIB_OUT(void)
{
    return(IB_OUT_pu*IB_OUT_FS);
}

// Power Stage B Input Current in A
float GetIB_IN(void)
{
    return(IB_IN_pu*IB_IN_FS);
}

// Power Stage B Output Voltage V
float GetVB_OUT(void)
{
    return(VB_OUT_pu*VB_OUT_FS);
}

// 24V Rail Voltage mV
float Get24V(void)
{
    return(RAIL24V_pu*RAIL24V_FS*1000.0f);
}


__interrupt void INT_myADCA_1_ISR(void)
{
    //
    // Add the latest result to the buffer
    //
    REF1V25_BUF_pu = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER11)/65536.0f;
    REF625mV_pu = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER7)/65536.0f;
    IA_L_pu = (ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0)/65536.0f)-REF1V25_BUF_pu;
    IA_OUT_pu = (ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1)/65536.0f)-REF1V25_BUF_pu;
    IA_IN_pu = (ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER2)/65536.0f)-REF1V25_BUF_pu;
    VA_OUT_pu = (ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER3)/65536.0f)-REF625mV_pu;
    VDC_pu = (ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER4)/65536.0f)-REF625mV_pu;
    IDC_pu = (ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER5)/65536.0f)-REF1V25_BUF_pu;
    //VAUX_pu = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER6)/65536.0f;
    RAIL3V3_pu = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER8)/65536.0f;
    RAIL5V_pu = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER10)/65536.0f;

    IB_L_pu = (ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0)/65536.0f)-REF1V25_BUF_pu;
    IB_OUT_pu = (ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER1)/65536.0f)-REF1V25_BUF_pu;
    IB_IN_pu = (ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER2)/65536.0f)-REF1V25_BUF_pu;
    VB_OUT_pu = (ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER3)/65536.0f)-REF625mV_pu;
    RAIL24V_pu = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER5)/65536.0f;

    THERMA_pu = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0)/65536.0f;
    THERMB_pu = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER1)/65536.0f;

    //
    // Clear the interrupt flag
    //
    ADC_clearInterruptStatus(myADCA_BASE, ADC_INT_NUMBER1);

    //
    // Check if overflow has occurred
    //
    if(true == ADC_getInterruptOverflowStatus(myADCA_BASE, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(myADCA_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(myADCA_BASE, ADC_INT_NUMBER1);
    }

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}


