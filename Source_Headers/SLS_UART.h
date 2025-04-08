/*
 * DCB_UART.h
 *
 *  Created on: 8 Nov 2024
 *      Author: IainMosely
 */

#ifndef SOURCE_HEADERS_SLS_UART_H_
#define SOURCE_HEADERS_SLS_UART_H_

#include <stdint.h>

__interrupt void INT_UARTATX_DMA_ISR(void);
void TransmitUartMessage(void);
void ConfigureUART(void);

#endif /* SOURCE_HEADERS_SLS_UART_H_ */
