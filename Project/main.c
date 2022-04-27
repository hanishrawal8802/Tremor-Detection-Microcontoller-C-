#include <msp430fr6989.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "uart.h"
#include "i2c.h"
#include "lcd.h"


#define I2C_ADDRESS_MPU9150 0x68
#define MEM_LEN 6000

unsigned int gTick_0_5ms = 0;
int res = -1;
unsigned char buffer[10];

#pragma PERSISTENT(FRAM_array)
unsigned char FRAM_array[MEM_LEN] = { 0x00 };

float gyroBiasX;
float gyroBiasY;
float gyroBiasZ;

float GyroBiasX;
float GyroBiasY;
float GyroBiasZ;

unsigned int fram_index;

unsigned char gbToggle = 1;
unsigned char gbFirst = 0;

float dt = 0;

float Roll;
float Pitch;
float Yaw;

int rawGyroX;
int rawGyroY;
int rawGyroZ;

float gyroX;
float gyroY;
float gyroZ;

int i = 0;
float d[10];
int j =0;
float avg = 0;
int m;
float Count_Time = 0;
int sec=0;

/**   main.c   */
void main(void)
{
    WDTCTL = WDT_MDLY_0_5;
    unsigned char oldSec = 0xFF;
    WDTCTL = WDT_ADLY_1000;

    P1SEL0 &= ~(BIT0 + BIT1); // Bit 1 and 0 are cleared of port 1 hence default I/O
    P1SEL1 &= ~(BIT0 + BIT1);
    P9SEL0 &= ~(BIT0 + BIT1); // Bit 1 and 0 are cleared of port 9 hence default I/O
    P9SEL1 &= ~(BIT0 + BIT1);

    P1DIR |= BIT0; // we set the 0 bit of the port 1 to on hence output
    P1DIR &= ~BIT1; // we cleared the 1 bit of the port 1 hence input
    P9DIR |= BIT7; // we set the 7 bit of the port 9 to on hence output

    P1OUT |= BIT1; // this is to make a pull up resistor by setting out and ren to 1
    P1REN |= BIT1;

    P1IES |= BIT1; // falling edge at P1.1 generates interrupt
    P1IE |= BIT1; // Interrupt enable for P1.1

    PM5CTL0 &= ~LOCKLPM5;  // to activate I/O functions this needs to be cleared

    SFRIE1 |= WDTIE; // WDT-interrupt enabled

    UART_Init(SPEED_38400_SMCLK);

    LCD_Init();     // Initialize the LCD

    I2C_Init(); // Initialize the I2C

    _EINT(); // global interrupt enable

    TA0CTL = TASSEL__SMCLK | MC__CONTINUOUS;

    PM5CTL0 &= ~LOCKLPM5;

    //LCD_displayShort(123);

    buffer[0] = 0x80; // this value needed to be written for the power management to reset the device
    res = I2C_WriteBlock(I2C_ADDRESS_MPU9150, buffer, 1, 0x6B); // sets the value of 0x80 to the sensor hub reseting it
    gTick_0_5ms = 100;      // wait for 50 ms
    LPM0;
    buffer[0] = 0x00;         // this value is needed to clear the sleep bit
    res = I2C_WriteBlock(I2C_ADDRESS_MPU9150, buffer, 1, 0x6B); // clearing the sleep bit
    gTick_0_5ms = 100;      // wait for 50 ms
    LPM0;


    //gyroBiasX[0] = 0; //assign 0
    //gyroBiasY[0] = 0; //assign 0
    //gyroBiasZ[0] = 0; //assign 0


        res = I2C_ReadBlock(I2C_ADDRESS_MPU9150, buffer, 6, 0x43);

        if (fram_index < MEM_LEN - 6)
        {
            FRAM_array[fram_index++] = buffer[0];
            FRAM_array[fram_index++] = buffer[1];
            FRAM_array[fram_index++] = buffer[2];
            FRAM_array[fram_index++] = buffer[3];
            FRAM_array[fram_index++] = buffer[4];
            FRAM_array[fram_index++] = buffer[5];

            gTick_0_5ms = 40;      // wait for 20 ms
            LPM0;
        }
        else
        {
            fram_index = 0;
            while (fram_index < MEM_LEN)
            {

                unsigned char HX = FRAM_array[fram_index++];
                unsigned char LX = FRAM_array[fram_index++];
                unsigned char HY = FRAM_array[fram_index++];
                unsigned char LY = FRAM_array[fram_index++];
                unsigned char HZ = FRAM_array[fram_index++];
                unsigned char LZ = FRAM_array[fram_index++];

                gyroBiasX = (HX << 8) | LX; // combine the high byte first then low byte
                gyroBiasY = (HY << 8) | LY; // combine the high byte first then low byte
                gyroBiasZ = (HZ << 8) | LZ; // combine the high byte first then low byte

                //gyroBiasX[0] = gyroBiasX[0] + gyroBiasX[1];
                //gyroBiasY[0] = gyroBiasY[0] + gyroBiasY[1];
                //gyroBiasZ[0] = gyroBiasZ[0] + gyroBiasZ[1];

            }

            GyroBiasX = gyroBiasX / 1000;
            GyroBiasY = gyroBiasY / 1000;
            GyroBiasZ = gyroBiasZ / 1000;

        }


    Yaw = 0;

    while (1)
    {

        avg = 0;

        res = I2C_ReadBlock(I2C_ADDRESS_MPU9150, buffer, 6, 0x43);

        rawGyroX = (buffer[0] << 8) | buffer[1]; // combine the high byte first then low byte
        rawGyroY = (buffer[2] << 8) | buffer[3]; // combine the high byte first then low byte
        rawGyroZ = (buffer[4] << 8) | buffer[5]; // combine the high byte first then low byte

        // store in i th position
        d[i] = sqrt((rawGyroX - GyroBiasX)*(rawGyroX - GyroBiasX)+(rawGyroY - GyroBiasY)*(rawGyroY - GyroBiasY)+(rawGyroZ - GyroBiasZ)*(rawGyroZ - GyroBiasZ));

        //LCD_displayShort(d[i]);

        //moving average condition
        if (i == 9)
        {

            for(j=0;j<10;j++)
            {
                m = d[i-j];
                avg = avg + m;
            }

          float  avg = avg/10;   // moving average

            if (avg<400)
            {
                if(sec <=2 && sec>0)
                {
                    Count_Time = 0;
                    m = m + 1;
                    LCD_displayShort(avg); //display score
                }
                i = 0;
            }
           else
           {
               continue;
           }
        }


       i = i+1;

        gTick_0_5ms = 20;      // wait for 10 ms
        LPM0;


    }

}



#pragma vector=WDT_VECTOR
__interrupt void tick(void)
{
    sec++;
    if (gTick_0_5ms > 0)
    {
        gTick_0_5ms--;
        if (gTick_0_5ms == 0)
            LPM0_EXIT;
    }
}
