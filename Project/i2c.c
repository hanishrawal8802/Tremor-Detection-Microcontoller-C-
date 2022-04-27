#include "i2c.h"

#define TRACE
#ifdef TRACE

#pragma PERSISTENT(trace)
unsigned char trace[TRACE_LEN]={0};
unsigned int trace_position = 0;
#endif

#include <msp430fr6989.h>

unsigned char *pI2cRcvBuffer, *pI2cSndBuffer;
unsigned gBytesRcvd, gBytesSent;
unsigned char gBytesToRcv, gBytesToSnd, gLocPtr;

#ifndef SW_I2C

volatile unsigned char gI2cState = 0xFF;
unsigned char gbSlvAdd;

void I2C_Init(void) {
    // Configure I2C pins
//    P5SEL0 |= (BIT2 | BIT3);  // select I2C pins

    unsigned long uL = 1000;
    P1SEL0 &= ~(BIT6+BIT7);
    P1DIR |= BIT6;
    P1OUT |= BIT7; // -> SCL -> 1
    while (uL-- >0);
    P1OUT |= BIT6; // -> SDA -> 1
    uL = 1000;
    while (uL-- >0);

    P1SEL0 |= (BIT6+BIT7); // select I2C pins
    P1SEL1 &= ~(BIT6+BIT7);
#if 0
    P1DIR &= ~(BIT6+BIT7);
    P1REN |= (BIT6+BIT7);
    P1OUT |= (BIT6+BIT7);
#endif
    PM5CTL0 &= ~LOCKLPM5;
     
    UCB0CTLW0 = UCSWRST;                   // Software reset enabled, 7-bit address mode
    UCB0CTLW0 |= (UCMODE_3|UCSYNC | UCMST | UCSSEL_3); // I2C mode, Master mode, BRCLK : SMCLK
    UCB0CTLW1 &= ~0xFF;                  // no automatic stop generated
//    UCB0CTLW1 |= UCGLIT0; // |UCGLIT0);
    UCB0BRW = 0x008;                       // baudrate = SMCLK / UCB0BRW
    UCB0CTLW0 &= ~UCSWRST;
    UCB0IE = 0x7FFF; // UCTXIE0 | UCRXIE0 | UCNACKIE;// enable I2C RCV/TX(/NACK interrupts
    UCB0IFG = 0;

    gI2cState = 0xFF;
}




void I2C_Error(void) {
#ifdef TRACE
    unsigned int uI = UCB0STATW;
  trace[trace_position++ % TRACE_LEN] = 0xE1;
  trace[trace_position++ % TRACE_LEN] = (unsigned char)uI;
  trace[trace_position++ % TRACE_LEN] = (unsigned char)(uI>>8);
  trace[trace_position++ % TRACE_LEN] = gLocPtr;
  trace[trace_position++ % TRACE_LEN] = gI2cState;
#endif
  I2C_Init();
#ifdef TRACE
  uI = UCB0STATW;
  trace[trace_position++ % TRACE_LEN] = 0xE2;
  trace[trace_position++ % TRACE_LEN] = (unsigned char)uI;
  trace[trace_position++ % TRACE_LEN] = (unsigned char)(uI>>8);
  uI = UCB0CTLW0;
  trace[trace_position++ % TRACE_LEN] = (unsigned char)uI;
  trace[trace_position++ % TRACE_LEN] = (unsigned char)(uI>>8);
#endif

}

int I2C_WriteBlock(unsigned char slvAdd,unsigned char* pData, unsigned char length, 
                   unsigned char location_pointer) {
    char done = 0;
    gbSlvAdd = slvAdd;
    pI2cSndBuffer = pData;
    gBytesToSnd = length;
    gLocPtr = location_pointer;

    while (UCB0CTLW0 & UCTXSTP);         // Ensure stop condition got sent
    
    gI2cState = WRITE_BASE;
    UCB0CTLW0 |= UCTR;          // select transmit mode
    UCB0I2CSA = slvAdd;          
    UCB0CTLW0 |= UCTXSTT;        // transmit I2C start condition
    gI2cState = WRITE_START_SENT;

#ifdef TRACE
  trace[trace_position++ % TRACE_LEN] = 0xCC;
  trace[trace_position++ % TRACE_LEN] = gI2cState;
  trace[trace_position++ % TRACE_LEN] = gLocPtr;
#endif


    unsigned long lCount = 0;
 
    while (!done) {
      if (lCount++ == 100000)
        return TIMEOUT_BITS|gI2cState;
      switch(gI2cState) {
          case WRITE_SUCCESS:
            done = 1;
            break;
          case WRITE_ADD_NACK: 
          case WRITE_WRITEADD_NACK:
            return gI2cState;
      }
    }
    return I2C_SUCCESS;
                   
}

// read a single byte from a slave; sequential read, i.e. no location pointer used
int I2C_ReadBytes(unsigned char slvAdd,unsigned char* pData, unsigned char length) {
       char done = 0;
       gbSlvAdd = slvAdd;
       pI2cRcvBuffer = pData;
       gBytesToRcv = length;

       while (UCB0CTLW0 & UCTXSTP);         // Ensure stop condition got sent

       gI2cState = READ_BASE;
       UCB0CTLW0 |= UCTR;          // select transmit mode
       UCB0I2CSA = slvAdd;
       UCB0CTLW0 |= UCTXSTT;        // transmit I2C start condition
       gI2cState=READ_START_SINGLE_SENT;

       unsigned long lCount = 0;

       while (!done) {
         if (lCount++ == 100000)
           return TIMEOUT_BITS|gI2cState;
         switch(gI2cState) {
             case READ_SUCCESS:
               done = 1;
               break;
             case READ_ADD_NACK:
             case READ_READADD_NACK:
               return gI2cState;
         }
       }
       return I2C_SUCCESS;
}

int I2C_ReadBlock(unsigned char slvAdd,unsigned char* pData, unsigned char length, 
                  unsigned char location_pointer) {

    char done = 0;
    gbSlvAdd = slvAdd;
    pI2cRcvBuffer = pData;
    gBytesToRcv = length;
    gLocPtr = location_pointer;

    while (UCB0CTLW0 & UCTXSTP);         // Ensure stop condition got sent
    
    gI2cState = READ_BASE;
    UCB0CTLW0 |= UCTR;          // select transmit mode
    UCB0I2CSA = slvAdd;          
    UCB0CTLW0 |= UCTXSTT;        // transmit I2C start condition
    gI2cState=READ_START_SENT;

#ifdef TRACE
  trace[trace_position++ % TRACE_LEN] = 0xDD;
  trace[trace_position++ % TRACE_LEN] = gI2cState;
  trace[trace_position++ % TRACE_LEN] = gLocPtr;
#endif


    unsigned long lCount = 0;
 
    while (!done) {
      if (lCount++ == 100000)
        return TIMEOUT_BITS|gI2cState;
      switch(gI2cState) {
          case READ_SUCCESS:
            done = 1;
            break;
          case READ_ADD_NACK: 
          case READ_READADD_NACK:
            return gI2cState;
      }
    }
    return I2C_SUCCESS;
}


unsigned char gbaLocPtr[2];

int I2C_ReadBlockEx(unsigned char slvAdd,unsigned char* pData, unsigned char length,
                  unsigned char lpHi, unsigned char lpLo) {

    char done = 0;

    gbSlvAdd = slvAdd;
    pI2cRcvBuffer = pData;
    gBytesToRcv = length;

    gbaLocPtr[0] = lpHi;
    gbaLocPtr[1] = lpLo;

    gLocPtr = gbaLocPtr[0];

    while (UCB0CTLW0 & UCTXSTP);         // Ensure stop condition got sent

    gI2cState = READ_BASE;
    UCB0CTLW0 |= UCTR;          // select transmit mode
    UCB0I2CSA = slvAdd;
    UCB0CTLW0 |= UCTXSTT;        // transmit I2C start condition
    gI2cState=READ_START_SENT1;

    unsigned long lCount = 0;

    while (!done) {
      if (lCount++ == 100000)
        return TIMEOUT_BITS|gI2cState;
      switch(gI2cState) {
          case READ_SUCCESS:
            done = 1;
            break;
          case READ_ADD_NACK:
          case READ_READADD_NACK:
            return gI2cState;
      }
    }
    return I2C_SUCCESS;
}


#pragma vector = USCI_B0_VECTOR
__interrupt void USCIB0_ISR(void) {
  volatile unsigned char rx;
  
  unsigned int result = __even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG);

#ifdef TRACE
  trace[trace_position++ % TRACE_LEN] = 0xAA;
  trace[trace_position++ % TRACE_LEN] = result;
  trace[trace_position++ % TRACE_LEN] = gI2cState;
  trace[trace_position++ % TRACE_LEN] = gLocPtr;
#endif

  switch(result)  {
    case 0x00: break;                       // Vector 0: No interrupts
    case 0x02: break;                       // Vector 2: ALIFG
    case 0x04:                              // Vector 4: NACKIFG
      switch(gI2cState) {
        case READ_START_SENT:
        case READ_START_SINGLE_SENT:
            gI2cState = READ_ADD_NACK;
            break;
        case READ_READADD_SENT:
            gI2cState = READ_READADD_NACK;
            break;
        case WRITE_START_SENT:
            gI2cState = WRITE_ADD_NACK;
      }
      UCB0CTLW0 |= UCTXSTP;           // I2C stop condition
//      __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
      break;
    case 0x06: break;                       // Vector 6: STTIFG
    case 0x08: break;                       // Vector 8: STPIFG
    case 0x0a: break;                       // Vector 10: RXIFG3
    case 0x0c: break;                       // Vector 14: TXIFG3
    case 0x0e: break;                       // Vector 16: RXIFG2
    case 0x10: break;                       // Vector 18: TXIFG2
    case 0x12: break;                       // Vector 20: RXIFG1
    case 0x14: break;                       // Vector 22: TXIFG1
    case 0x16:         
      switch(gI2cState) {
        case READ_RCV_DATA:
          if (gBytesRcvd == gBytesToRcv-1) 
              UCB0CTLW0 |= UCTXSTP;         // Ensure stop condition got sent
          pI2cRcvBuffer[gBytesRcvd++] = UCB0RXBUF; 
          if (gBytesRcvd == gBytesToRcv) 
             gI2cState = READ_SUCCESS;
          break;
         default:
           rx = UCB0RXBUF;
      }
      break;
    case 0x18:                 // Vector 26: TXIFG0
        switch (gI2cState) {
          case WRITE_START_SENT:
              gI2cState = WRITE_POSPTR_SENT;
              UCB0TXBUF = gLocPtr;
              break;
          case READ_START_SINGLE_SENT:
              gI2cState = READ_READADD_SENT;
              UCB0CTLW0 &= ~UCTR;       // select rcv mode
              UCB0I2CSA = gbSlvAdd;     // slave read address
              UCB0CTLW0 |= UCTXSTT;     // generate Start condition
              gBytesRcvd = 0;
              gI2cState = READ_RCV_DATA;
              break;
          case READ_START_SENT:
              gI2cState = READ_POSPTR_SENT;
              UCB0TXBUF = gLocPtr;
              break;
          case READ_START_SENT1:
              gI2cState = READ_POSPTR_SENT1;
              UCB0TXBUF = gLocPtr;
              break;
          case READ_POSPTR_SENT1:
              gI2cState = READ_POSPTR_SENT2;
              gLocPtr = gbaLocPtr[1];
              UCB0TXBUF = gLocPtr;
              break;
          case WRITE_POSPTR_SENT:
              gI2cState = WRITE_WRITEADD_SENT;
              gBytesSent = 0;
              if (gBytesToSnd == 0) {
                  gI2cState = WRITE_SUCCESS;
                  UCB0CTLW0 |= UCTXSTP;           // I2C stop condition
                  break;
              }
              UCB0I2CSA = gbSlvAdd;     // slave address         
              UCB0TXBUF = pI2cSndBuffer[gBytesSent++];
              if (gBytesToSnd == gBytesSent) {
                gI2cState = WRITE_SUCCESS;
                UCB0CTLW0 |= UCTXSTP;           // I2C stop condition
              }
              else 
                gI2cState = WRITE_SND_DATA;
              break;
          case WRITE_SND_DATA:
              UCB0TXBUF = pI2cSndBuffer[gBytesSent++];
              if (gBytesToSnd == gBytesSent) {
                gI2cState = WRITE_SUCCESS;
                UCB0CTLW0 |= UCTXSTP;           // I2C stop condition
              }
              break;

          case READ_POSPTR_SENT:
          case READ_POSPTR_SENT2:
              gI2cState = READ_READADD_SENT;
              UCB0CTLW0 &= ~UCTR;       // select rcv mode
              UCB0I2CSA = gbSlvAdd;     // slave read address         
              UCB0CTLW0 |= UCTXSTT;     // generate Start condition
              gBytesRcvd = 0;
              gI2cState = READ_RCV_DATA;
              break;
        }    
        break;       
    case 0x1a:                              // Vector 28: BCNTIFG
      break;
    case 0x1c: break;                       // Vector 30: clock low timeout
    case 0x1e:
        break;                       // Vector 32: 9th bit
    default: break;
     
    
  }

#ifdef TRACE
  trace[trace_position++ % TRACE_LEN] = 0xBB;
  trace[trace_position++ % TRACE_LEN] = gBytesRcvd;
  trace[trace_position++ % TRACE_LEN] = gI2cState;
  if (gBytesRcvd>0)
      trace[trace_position++ % TRACE_LEN] = pI2cRcvBuffer[gBytesRcvd-1];
  else
      trace[trace_position++ % TRACE_LEN] = 0x00;
#endif
}

#else

void I2C_Init(void){
  /* Initialize SDA and SCL to input lines (-> hi ) */
  P1SEL0 &= ~(SDA+SCL);
  P1SEL1 &= ~(SDA+SCL);
  P1DIR &= ~(SDA+SCL);
  /* as output: SDA and SCL are low */
  P1OUT &= ~(SDA+SCL);
  PM5CTL0 &= ~LOCKLPM5;
}

/* delay function */
void delay(unsigned int i) {
  while (--i > 0) _NOP();
}

/* generate START condition
SDA --------\
             \-------

SCL ----------\
               \-------
*/

void I2C_Start(void) {
  SDA_HI;SCL_HI;SDA_LO; BIT_DELAY; SCL_LO; BIT_DELAY;
}

/* generate STOP condition.
SDA           /--------
    ---------/

         /---------
SCL ----/
*/

void I2C_Stop(void) {
  SDA_LO;SCL_LO;SCL_HI;BIT_DELAY;SDA_HI; BIT_DELAY;
}

/* Send a Bit
   Set data line to appropriate level (1/0)
   Set SCL to high for some time. Then set SCL low again before
   changing SDA

   transmit 0:

   SDA -------\               /------
               \-------------/
                   /-----\
   SCL -----------/       \-------------
                      0

   Note that any argument != 0 is treated as a 1-bit
*/

void I2C_SendBit(unsigned char bit ) {
  if (!bit)
      SDA_LO;
     else
      SDA_HI;
    EDGE_DELAY;
    SCL_HI; BIT_DELAY; SCL_LO;
}

void I2C_GiveAck(unsigned char ack_nack) {
    SCL_LO;
    if (ack_nack==ACK)
        SDA_LO;
    else
        SDA_HI;
    EDGE_DELAY;
    SCL_HI; BIT_DELAY; SCL_LO;
}

/* Send one byte
  Bytes put on the bus are always 8 bits long.
  The MSB (Most significant bit) is always sent first
*/
void I2C_SendByte(unsigned char byte) {
#ifdef TRACE
trace[trace_position++ % TRACE_LEN] = 0x33;
trace[trace_position++ % TRACE_LEN] = byte;
#endif

    unsigned char count;
   for (count=0;count < 8;count++) {
      I2C_SendBit(byte & 0x80);
      /* shift one position to the left */
      byte <<= 1;
   }
}

/* Receive a bit
  Release SCL Line; then check SDA; next set SCL lo again.
*/
unsigned char I2C_ReceiveBit(void) {
  unsigned char bit;
  EDGE_DELAY;
  SCL_HI;
  BIT_DELAY;
  if ((P1IN & SDA) )
      bit = '\1';
  else
      bit = '\0';
  SCL_LO;
  return bit;
}

/* Receive one byte (MSB first) */
//char check[8]={0};

unsigned char I2C_ReceiveByte(void) {
  unsigned char byte = '\0', count;
  SDA_HI;
  for (count = 0; count < 8; count++) {
    byte |= (I2C_ReceiveBit() << (7 - count));
//    check[count]=byte;
  }
  return byte;
}

/* Waiting for ACKNOWLEDGE: When a SLAVE is being addressed
or has received data it will issue an ACKNOWLEDGE pulse.
Therefore the MASTER must release the SDA line (SDA_HI)
and then release the SCL line (SCL_HI). Now it must wait
for the SLAVE to pull the DATA line low.
Now the MASTER will take the SCL line low and then the
SLAVE will release the SDA line
return value: 0 - OK, ACK received
              1 - No ACK
*/
int I2C_WaitForACK(void){
    SCL_LO;
    SDA_HI;
    EDGE_DELAY;
    SCL_HI;
    if (I2C_ReceiveBit() == '\0') {
        SCL_LO;
        return ACK_RECEIVED;
    }
    SCL_LO;
    return !ACK_RECEIVED;
}
/* Addressing: When data have to be transmitted to a SLAVE, the SLAVE
  is identified by its WRITE_ADDRESS. To read data from a SLAVE, its
  READ_ADDRESS has to be used. */

/* WriteByte: Write one Byte to a SLAVE:

   START|SLAVE_WRITE_ADDRESS|ACK_by_Slave|location_pointer|
           ACK_by_Slave|DATA_BYTE|ACK_by_Slave|STOP

    location_pointer: SLAVE location (where to write)
    return value: 0 - OK
                  1 - No Address ACK from Slave
                  2 - No Word_addres ACK from slave
                  3 - No Data ACK from slave  */
int I2C_WriteByte(unsigned char slvAdd, unsigned char location_pointer, unsigned char data_byte) {
    slvAdd <<=1;
    I2C_Start();
    I2C_SendByte(slvAdd);
    if (I2C_WaitForACK() != ACK_RECEIVED) return WRITE_WRITEADD_NACK;
    I2C_SendByte(location_pointer);
    if (I2C_WaitForACK() != ACK_RECEIVED) return WRITE_POSPTR_NACK;
      I2C_SendByte(data_byte);
    if (I2C_WaitForACK() != ACK_RECEIVED) return WRITE_SND_DATA_NACK;
     I2C_Stop();
   return I2C_SUCCESS;
}

/* ReadByte
   Read one Byte from a SLAVE:

  START|SLAVE_W_ADDRESS|ACK|location_pointer|ACK
         |START|SLAVE_R_ADDRESS|ACK|DATA_BYTE|STOP

   location_pointer: SLAVE location (from where to read)
   return value: 0 - OK
                  1 - No W_Address ACK from Slave
                  2 - No Word_addres ACK from slave
                  3 - No R_Address ACK from slave
*/
int I2C_ReadByte(unsigned char slvAdd, unsigned char location_pointer,
       unsigned char * pData) {
    slvAdd <<=1;
    I2C_Start();
    I2C_SendByte(slvAdd);
    if (I2C_WaitForACK() != ACK_RECEIVED) return READ_ADD_NACK;
     I2C_SendByte(location_pointer);
    if (I2C_WaitForACK() != ACK_RECEIVED) return READ_POSPTR_NACK;

    I2C_Start();
    I2C_SendByte(slvAdd+1);
    if (I2C_WaitForACK() != ACK_RECEIVED) return READ_DATA_NACK;
    *pData = I2C_ReceiveByte();
    I2C_Stop();
    return I2C_SUCCESS;
}


/* Write Block to SLAVE
  return value:   0 - OK
                  != 0: high byte: byte number;
                        low byte: WriteByte-Error
*/
int I2C_WriteBlock(unsigned char slvAdd,
               unsigned char* pData, unsigned char length,
               unsigned char location_pointer) {
  unsigned char count, wAdd= (slvAdd<<1);
#if 1
  I2C_Start();
  I2C_SendByte(wAdd);
  if (I2C_WaitForACK() != ACK_RECEIVED) return WRITE_ADD_NACK;
  I2C_SendByte(location_pointer);
  if (I2C_WaitForACK() != ACK_RECEIVED) return WRITE_POSPTR_SENT;
  for (count =0; count < length;count++) {
    I2C_SendByte(pData[count]);
    if (I2C_WaitForACK() != ACK_RECEIVED) return (WRITE_SND_DATA | (count <<8));
  }
  I2C_Stop();
#else
  for (count =0; count < length;count++) {
      res = I2C_WriteByte(slvAdd, location_pointer++,pData[count]);
      if (res != I2C_SUCCESS) {
          I2C_Stop();
          return ((count+1) << 8) | res;
      }
      BIT_DELAY;
    }
#endif
  return I2C_SUCCESS;
}

/* Read Block from SLAVE
  return value: see WriteBlock
*/
int I2C_ReadBlock(unsigned char slvAdd,
              unsigned char* pData, unsigned char length,
              unsigned char location_pointer) {
    unsigned char wAdd = (slvAdd << 1);
    I2C_Start();
    I2C_SendByte(wAdd);
    if (I2C_WaitForACK() != ACK_RECEIVED) return READ_ADD_NACK;
    I2C_SendByte(location_pointer);
    if (I2C_WaitForACK() != ACK_RECEIVED) return READ_POSPTR_SENT1;
    I2C_Start();
    I2C_SendByte(wAdd+1);
    if (I2C_WaitForACK() != ACK_RECEIVED) return READ_DATA_NACK;

#ifdef TRACE
  trace[trace_position++ % TRACE_LEN] = 0x55;
  trace[trace_position++ % TRACE_LEN] = P1DIR;
  trace[trace_position++ % TRACE_LEN] = P1IN;
#endif

    SCL_HI;

#ifdef TRACE
  trace[trace_position++ % TRACE_LEN] = 0x66;
  trace[trace_position++ % TRACE_LEN] = P1IN;
#endif
    unsigned int i;
    for (i=0;i<1000;i++)  {
        BIT_DELAY; BIT_DELAY; BIT_DELAY;
#ifdef TRACE
  trace[trace_position++ % TRACE_LEN] = 0x77;
  trace[trace_position++ % TRACE_LEN] = P1IN;
#endif
        if (P1IN & SCL)
            break;
    }
    unsigned char count=0;
    while (count < length) {
        pData[count++] = I2C_ReceiveByte();
        if (count == length)
            I2C_GiveAck(NACK);
        else
            I2C_GiveAck(ACK);
     }
    I2C_Stop();
    return I2C_SUCCESS;
}

int I2C_ReadBlockEx(unsigned char slvAdd,unsigned char* pData, unsigned char length,
                  unsigned char lpHi, unsigned char lpLo) {
      unsigned char wAdd = (slvAdd << 1);
      I2C_Start();
      I2C_SendByte(wAdd);
      if (I2C_WaitForACK() != ACK_RECEIVED) return READ_ADD_NACK;
      I2C_SendByte(lpHi);
      if (I2C_WaitForACK() != ACK_RECEIVED) return READ_POSPTR_SENT1;
      I2C_SendByte(lpLo);
      if (I2C_WaitForACK() != ACK_RECEIVED) return READ_POSPTR_SENT2;

      I2C_Start();
      I2C_SendByte(wAdd+1);
      if (I2C_WaitForACK() != ACK_RECEIVED) return READ_DATA_NACK;
      unsigned char count=0;
      while (count < length) {
          pData[count++] = I2C_ReceiveByte();
#ifdef TRACE
  trace[trace_position++ % TRACE_LEN] = 0xDD;
  trace[trace_position++ % TRACE_LEN] = pData[count-1];
#endif
          if (count == length)
              I2C_GiveAck(NACK);
          else
              I2C_GiveAck(ACK);
       }
      I2C_Stop();
      return I2C_SUCCESS;

 }




void I2C_Error(void) {
#ifdef TRACE
  trace[trace_position++ % TRACE_LEN] = 0xE1;
  trace[trace_position++ % TRACE_LEN] = (unsigned char)P1IN;
  trace[trace_position++ % TRACE_LEN] = gLocPtr;
#endif
  I2C_Init();
#ifdef TRACE
  trace[trace_position++ % TRACE_LEN] = 0xE2;
  trace[trace_position++ % TRACE_LEN] = (unsigned char)P1IN;
#endif

}


#endif
