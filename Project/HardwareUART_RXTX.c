
#include <msp430fr6989.h>
#include <string.h>
#include "uart.h"


unsigned char txBuf[BUF_LEN]; // TX buffer
unsigned char rxBuf[BUF_LEN]; // RX buffer
unsigned char bMode = UART_IDLE;
unsigned char txIndex = 0xFF;
unsigned char rxIndex = 0xFF;
unsigned char txLen;
unsigned char bEcho=0;
unsigned char bByteRcvd;

#define USE_TX_INTERRUPT

void UART_Init(unsigned char spd) {
  P3SEL0 |= (BIT4|BIT5); // P3.4: TXD, P3.5: RXD, f¸r eUSCI verwendet
  P3DIR |= BIT4; // TXD
  P3DIR &= ~BIT5; // RXD
  PM5CTL0 &= ~LOCKLPM5;

  switch(spd) {
  case SPEED_38400_SMCLK:
        UCA1CTLW0 = UCSWRST|UCSSEL__SMCLK; // UART mode, asynch.,
         // 8 databits, 1 stopbit, no parity, LSB first BRCLK: SMCLK (1 MHz)
         // BEISPIEL: 38400 bps @1MHz: gem‰ﬂ Table 30-5 (User's guide)
         // UCOS16=1, UCBRx = 1, UCBRFx = 10, UCBRSx = 0x00
      UCA1BRW = 1;
      UCA1MCTLW = UCBRF_10 |UCOS16 ;
      break;
  case SPEED_38400_SMCLK_8MHZ:
      UCA1CTLW0 = UCSWRST|UCSSEL__SMCLK; // UART mode, asynch.,
         // 8 databits, 1 stopbit, no parity, LSB first BRCLK: SMCLK (1 MHz)
         // BEISPIEL: 38400 bps @8MHz: gem‰ﬂ Table 30-5 (User's guide)
         // UCOS16=1, UCBRx = 13, UCBRFx = 0, UCBRSx = 0x84
      UCA1BRW = 13;
      UCA1MCTLW = (0x84 << 8)|UCBRF_0 |UCOS16 ;
      break;
  case SPEED_4800_ACLK:
      UCA1CTLW0 = UCSWRST|UCSSEL__ACLK; // UART mode, asynch.,
      UCA1BRW = 6;
      UCA1MCTLW = (0xEE<<8)|UCOS16 ;
      break;
  case SPEED_4800_SMCLK:
      UCA1CTLW0 = UCSWRST|UCSSEL__SMCLK; // UART mode, asynch.,
      UCA1BRW = 6;
      UCA1MCTLW = (0xEE<<8)|UCOS16 ;
      break;
  case SPEED_9600_SMCLK_8MHZ:
      // UCOS16=1, UCBRx=52, UCBRF=1, UCBRS=0x49
      UCA1CTLW0 = UCSWRST|UCSSEL__SMCLK; // UART mode, asynch.,
      UCA1BRW = 52;
      UCA1MCTLW = (0x49<<8)|UCBRF_1|UCOS16 ;
      break;
  case SPEED_9600_SMCLK:
      UCA1CTLW0 = UCSWRST|UCSSEL__SMCLK; // UART mode, asynch.,
      UCA1BRW = 6;
      UCA1MCTLW = (0x20<<8)|UCBRF_8|UCOS16 ;
      break;
  case SPEED_19200_SMCLK:
      UCA1CTLW0 = UCSWRST|UCSSEL__SMCLK; // UART mode, asynch.,
      UCA1BRW = 3;
      UCA1MCTLW = (0x02<<8)|UCBRF_4|UCOS16 ;
      break;
  case SPEED_57600_SMCLK:
      UCA1CTLW0 = UCSWRST|UCSSEL__SMCLK; // UART mode, asynch.,
      UCA1BRW = 17;
      UCA1MCTLW = (0x4A<<8);
      break;
  }
  UCA1CTLW0 &= ~UCSWRST;
#ifdef USE_TX_INTERRUPT
  UCA1IE |= (UCRXIE+UCTXIE); // enable RX+TX interrupt.
#else
  UCA1IE |= UCRXIE); // enable RX
#endif
}

void TX_Byte(unsigned char c) {
#ifndef USE_TX_INTERRUPT
    UCA1TXBUF = c;
  while(UCA1STATW & UCBUSY);
#else
  if (bMode != UART_IDLE)
      LPM0;
  UCA1TXBUF = c;
  bMode = UART_TX_BYTE;
  LPM0;
#endif
}

void TX_String(unsigned char* str) {
    unsigned int len=0;
    while(str[len] != '\0') len++;
    TX_ByteArray(str,len);
}

void TX_ShortByteArray(unsigned char* ba, int len) {
#ifndef USE_TX_INTERRUPT
  int i;
  for (i=0;i<len;i++)
    TX_Byte(ba[i]);
#else
  if (bMode!=UART_IDLE)
      LPM0;
    txLen = 0;
    for(txLen=0;txLen<len;txLen++) {
        txBuf[txLen]=ba[txLen];
    }
    txIndex = 0;
    UCA1TXBUF = txBuf[txIndex++];
    bMode = UART_TX;
#endif
}

void TX_ByteArray(unsigned char* ba, int len) {
#ifndef USE_TX_INTERRUPT
  int i;
  for (i=0;i<len;i++)
    TX_Byte(ba[i]);
#else
  if (bMode!=UART_IDLE)
      LPM0;

  unsigned int index = 0;
  while(len > BUF_LEN) {
     TX_ShortByteArray(&ba[index],BUF_LEN);
     index += BUF_LEN;
     len -= BUF_LEN;
  }
  if (len > 0)
      TX_ShortByteArray(&ba[index],len);
#endif
}

void waitUntilTxDone(void) {
    while(bMode !=UART_IDLE)
        _NOP();
}

unsigned char RX_Byte(unsigned char fEcho) {
    if (bMode!=UART_IDLE)
        LPM0;
    bMode = UART_RX_BYTE;
    bEcho = fEcho;
    LPM0;
    return bByteRcvd;
}

void RX_String(unsigned char* pb, int iMax, unsigned char fEcho) {
    unsigned char i;
  if (bMode != UART_IDLE)
      LPM0;
  rxIndex = 0;
  bEcho = fEcho;
  bMode = UART_RX;
  LPM0;
  for (i=0;rxBuf[i]!='\0';i++)
      pb[i]=rxBuf[i];
  pb[i] = rxBuf[i]; // abschlieﬂende 0
}

#pragma vector = USCI_A1_VECTOR
__interrupt void uart_isr(void) {
    if (UCA1IFG & UCRXIFG) {
          unsigned char c=UCA1RXBUF;
          UCA1IFG &= ~BIT0;
          if (bMode == UART_RX) {
              if (bEcho)
                  UCA1TXBUF = c;
               if (c=='\n' || c=='\r') {
                  rxBuf[rxIndex] = '\0'; // String-Ende markieren
                  bMode = UART_IDLE;
                  LPM0_EXIT;
                  return;
              }
              else {
                  rxBuf[rxIndex++]=c;
                  if (rxIndex==BUF_LEN)
                      rxIndex=0;
              }
          }
          else if (bMode==UART_RX_BYTE){
              if (bEcho)
                  UCA1TXBUF = c;
              bByteRcvd = c;
              bMode = UART_IDLE;
              LPM0_EXIT;
          }
    }
    if (UCA1IFG & UCTXIFG) { // TX buffer empty
        UCA1IFG &= ~BIT1;
        if (bMode == UART_TX)
            if (txIndex < txLen) {
                UCA1TXBUF = txBuf[txIndex++];
            }
            else {
                bMode = UART_IDLE;
                LPM0_EXIT;
            }
        else if (bMode == UART_TX_BYTE) {
            bMode = UART_IDLE;
            LPM0_EXIT;
        }
    }
}


int fputc(int _c, register FILE *_fp) {
  TX_Byte((unsigned char)_c);
  return((unsigned char)_c);
}

int fputs(const char * __restrict _ptr, FILE * _fp) {
    TX_String(_ptr);
    return strlen(_ptr);
}

#if 0
// Problem: bei scanf geht erstes Zeichen verloren
int fgetc(FILE *_fp) {
    return RX_Byte(1);
}

char   *fgets(char *_ptr, int _size, FILE * _fp) {
    RX_String(_ptr, _size, 1);
    return _ptr;
}
#endif
