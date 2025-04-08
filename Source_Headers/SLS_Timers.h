/*
 * DCB_Timers.h
 *
 *  Created on: 8 Nov 2024
 *      Author: IainMosely
 */

#ifndef CPU1_FLASH_SOURCE_HEADERS_DCB_TIMERS_H_
#define CPU1_FLASH_SOURCE_HEADERS_DCB_TIMERS_H_

#include "stdint.h"

__interrupt void INT_myCPUTIMER0_ISR(void);
__interrupt void INT_Heartbeat2Hz_ISR(void);
void HeartbeatToggle(void);
void TwoHzUpdate(void);
void CheckTwoHz(void);

uint64_t BGTimer_getCurrentTime();
uint64_t BGTimer_getElapsedTime_ms(uint64_t startTime);



#endif /* CPU1_FLASH_SOURCE_HEADERS_DCB_TIMERS_H_ */
