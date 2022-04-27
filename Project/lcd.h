/*
 * lcd.h
 *
 *  Created on: 23.11.2018
 *      Author: jr
 */

void LCD_Init(void);
void LCD_showChar(char c, int position);
void LCD_displayShort(signed short s);
void LCD_displayString(char* pStr);
void LCD_displayShortHex(char* pStr, int s);
void LCD_Symbol(char id, char op);


void LCD_clear(void);


#define POS1 9   /* Digit A1 */
#define POS2 5   /* Digit A2   */
#define POS3 3   /* Digit A3   */
#define POS4 18  /* Digit A4  */
#define POS5 14   /* Digit A5  */
#define POS6 7  /* Digit A6  */

// for word access to LCD memory
#define LCDMEMW ((int*)LCDMEM)

#define LCD_TMR_ID 0
    #define LCD_TMR_IDX 2
    #define LCD_TMR_COM BIT3
#define LCD_HRT_ID 1
    #define LCD_HRT_IDX 2
    #define LCD_HRT_COM BIT2
#define LCD_REC_ID 2
    #define LCD_REC_IDX 2
    #define LCD_REC_COM BIT1
#define LCD_EXCLAMATION_ID 3
    #define LCD_EXCLAMATION_IDX 2
    #define LCD_EXCLAMATION_COM  BIT0
#define LCD_BRACKETS_ID 4
    #define LCD_BRACKETS_IDX 17
    #define LCD_BRACKETS_COM BIT4
#define LCD_BATT_ID  5
    #define LCD_BATT_IDX 13
    #define LCD_BATT_COM BIT4
#define LCD_B1_ID 6
    #define LCD_B1_IDX 17
    #define LCD_B1_COM  BIT5
#define LCD_B2_ID 7
    #define LCD_B2_IDX 13
    #define LCD_B2_COM  BIT5
#define LCD_B3_ID 8
    #define LCD_B3_IDX 17
    #define LCD_B3_COM  BIT6
#define LCD_B4_ID 9
    #define LCD_B4_IDX 13
    #define LCD_B4_COM  BIT6
#define LCD_B5_ID 10
    #define LCD_B5_IDX 17
    #define LCD_B5_COM  BIT7
#define LCD_B6_ID 11
    #define LCD_B6_IDX 13
    #define LCD_B6_COM  BIT7
#define  LCD_ANT_ID 12
    #define  LCD_ANT_IDX 4
    #define  LCD_ANT_COM BIT2
#define LCD_TX_ID 13
    #define LCD_TX_IDX 8
    #define LCD_TX_COM BIT2
#define LCD_RX_ID 14
    #define LCD_RX_IDX 8
    #define LCD_RX_COM BIT0
#define LCD_NEG_ID 15
    #define LCD_NEG_IDX 10
    #define LCD_NEG_COM BIT2
#define LCD_DEG_ID 16
    #define LCD_DEG_IDX 15
    #define LCD_DEG_COM BIT2
#define LCD_A1DP_ID 17
    #define LCD_A1DP_IDX 10
    #define LCD_A1DP_COM BIT0
#define LCD_A2DP_ID 18
    #define LCD_A2DP_IDX 6
    #define LCD_A2DP_COM BIT0
#define LCD_A3DP_ID 19
    #define LCD_A3DP_IDX 4
    #define LCD_A3DP_COM BIT0
#define LCD_A4DP_ID 20
    #define LCD_A4DP_IDX 19
    #define LCD_A4DP_COM BIT0
#define LCD_A5DP_ID 21
    #define LCD_A5DP_IDX 15
    #define LCD_A5DP_COM BIT0
#define LCD_A2COL_ID 22
    #define LCD_A2COL_IDX 6
    #define LCD_A2COL_COM BIT2
#define LCD_A4COL_ID 23
    #define LCD_A4COL_IDX 19
    #define LCD_A4COL_COM BIT2


// symbol operation
#define LCD_SYMBOL_ON 0
#define LCD_SYMBOL_OFF 1
#define LCD_SYMBOL_TOGGLE 2

#define BITS_ALPHANUM_LO 0xFF
#define BITS_ALPHANUM_HI 0xFA
