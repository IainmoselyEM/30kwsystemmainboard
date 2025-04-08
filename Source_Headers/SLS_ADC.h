/*
 * PE_ADC.h
 *
 *  Created on: 30 Aug 2024
 *      Author: IainMosely
 */

#ifndef BSW_PE_ADC_H_
#define BSW_PE_ADC_H_

__interrupt void INT_myADCA_1_ISR(void);

float GetMidscaleVoltage(void);
float GetQuarterScaleVoltage(void);
float GetIA_L(void);
float GetIA_OUT(void);
float GetIA_IN(void);
float GetVA_OUT(void);
float GetVDC(void);
float GetIDC(void);
float Get3V3(void);
float Get5V(void);
float GetIB_L(void);
float GetIB_OUT(void);
float GetIB_IN(void);
float GetVB_OUT(void);
float Get24V(void);

#endif /* BSW_PE_ADC_H_ */
