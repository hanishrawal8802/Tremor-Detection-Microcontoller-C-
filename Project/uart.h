#include <stdio.h>

#define MAX_RCV 50

void UART_Init(unsigned char speed);
void TX_Byte(unsigned char c);
void TX_String(unsigned char* pszStr);
unsigned char RX_Byte(unsigned char fEcho);
void RX_String(unsigned char* pb, int iMax, unsigned char bEcho);
void TX_ByteArray(unsigned char* ba, int len);
void waitUntilTxDone(void);


#define BUF_LEN 100

#define UART_IDLE 0
#define UART_TX 1
#define UART_TX_BYTE 2
#define UART_RX 3
#define UART_RX_BYTE 4

#define SPEED_9600_SMCLK 1
#define SPEED_9600_SMCLK_8MHZ 8
#define SPEED_19200_SMCLK 2
#define SPEED_38400_SMCLK 3
#define SPEED_57600_SMCLK 4
#define SPEED_4800_ACLK 5
#define SPEED_4800_SMCLK 6
#define SPEED_38400_SMCLK_8MHZ 7
