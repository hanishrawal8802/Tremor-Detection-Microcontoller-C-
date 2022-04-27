/*
 * uartTX.h
 *
 *  Created on: 11.01.2019
 *      Author: jr
 */

#ifndef UARTTX_H_
#define UARTTX_H_

#define BITTIME 52 // (19200 bps -> 52.08 µs/bit)
#define TXD BIT4

#define MODE_IDLE 0
#define MODE_TX 1

void TX_Byte(unsigned char b);
void TX_String(char* str);
void TX_Init(void);


#endif /* UARTTX_H_ */
