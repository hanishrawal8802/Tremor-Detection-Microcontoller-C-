// SW_I2C: If defined: use software I2C implementation
// otherwise: use UCB0 I2C support

#define SW_I2C
#ifdef SW_I2C

#define SDA BIT6  /* Pin 1.6 used for SDA */
#define SCL BIT7  /* Pin 1.7 used for SCL */

#define SDA_LO  (P1DIR |= SDA)
#define SCL_LO  (P1DIR |= SCL)
#define SDA_HI  (P1DIR &= ~SDA)
#define SCL_HI  (P1DIR &= ~SCL)

#define BIT_DELAY  {_NOP();_NOP();_NOP();_NOP();_NOP();}
#define EDGE_DELAY _NOP()
#define SHORT_WAIT _NOP()
#define ACK_RECEIVED 0

#define ACK 1
#define NACK 2

#endif


#define I2C_SUCCESS     0

#define WRITE_BASE              0x60
#define WRITE_START_SENT        WRITE_BASE+1
#define WRITE_WRITEADD_NACK     WRITE_BASE+2
#define WRITE_POSPTR_SENT       WRITE_BASE+3
#define WRITE_WRITEADD_SENT     WRITE_BASE+4
#define WRITE_ADD_NACK          WRITE_BASE+5

#define WRITE_SND_DATA          WRITE_BASE+8
#define WRITE_SUCCESS           WRITE_BASE+9

#define WRITE_POSPTR_NACK       WRITE_BASE+10
#define WRITE_SND_DATA_NACK     WRITE_BASE+11



#define READ_BASE               0x40
#define READ_START_SENT        READ_BASE+1
#define READ_SEND_POSPTR       READ_BASE+2
#define READ_ADD_NACK          READ_BASE+3
#define READ_POSPTR_SENT       READ_BASE+4
#define READ_POSPTR_ACK        READ_BASE+5
#define READ_READADD_SENT      READ_BASE+6
#define READ_READADD_NACK      READ_BASE+7
#define READ_RCV_DATA          READ_BASE+8  

#define READ_START_SENT1        READ_BASE+10
#define READ_SEND_POSPTR1       READ_BASE+11
#define READ_SEND_POSPTR2       READ_BASE+12
#define READ_POSPTR_SENT1       READ_BASE+13
#define READ_POSPTR_SENT2       READ_BASE+14
#define READ_POSPTR_ACK2        READ_BASE+15

#define READ_START_SINGLE_SENT       READ_BASE+16

#define READ_POSPTR_NACK       WRITE_BASE+17
#define READ_DATA_NACK       WRITE_BASE+18



#define READ_SUCCESS           READ_BASE+102



#define MASK_BITS         0xFF00
#define TIMEOUT_BITS      0xEE00
#define NACK_BITS         0xDD00





void I2C_Init(void); 
void I2C_Error(void);

int I2C_WriteBlock(unsigned char slvAdd,unsigned char* pData, unsigned char length, 
                             unsigned char location_pointer);
int I2C_ReadBlock(unsigned char slvAdd,unsigned char* pData, unsigned char length, 
                             unsigned char location_pointer);
int I2C_ReadBlockEx(unsigned char slvAdd,unsigned char* pData, unsigned char length,
                    unsigned char location_pointerHi, unsigned char location_pointerLo);
int I2C_ReadBytes(unsigned char slvAdd,unsigned char* pData, unsigned char length);

extern volatile unsigned char gI2cState;
extern unsigned char trace[];
extern unsigned int trace_position;
#define TRACE_LEN 1000
