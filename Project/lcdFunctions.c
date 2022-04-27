
#include <msp430fr6989.h>
#include "lcd.h"

const char lcdPos[]={POS1, POS2, POS3,POS4,POS5,POS6};

// LCD memory map for numeric digits
const char digit[10][2] = {
    {0xFC, 0x28},  /* "0" LCD segments a+b+c+d+e+f+k+q */
    {0x60, 0x20},  /* "1" */
    {0xDB, 0x00},  /* "2" */
    {0xF3, 0x00},  /* "3" */
    {0x67, 0x00},  /* "4" */
    {0xB7, 0x00},  /* "5" */
    {0xBF, 0x00},  /* "6" */
    {0xE4, 0x00},  /* "7" */
    {0xFF, 0x00},  /* "8" */
    {0xF7, 0x00}   /* "9" */
};

// LCD memory map for uppercase letters
const char alphabetBig[26][2] = {
    {0xEF, 0x00},  /* "A" LCD segments a+b+c+e+f+g+m */
    {0xF1, 0x50},  /* "B" */
    {0x9C, 0x00},  /* "C" */
    {0xF0, 0x50},  /* "D" */
    {0x9F, 0x00},  /* "E" */
    {0x8F, 0x00},  /* "F" */
    {0xBD, 0x00},  /* "G" */
    {0x6F, 0x00},  /* "H" */
    {0x90, 0x50},  /* "I" */
    {0x78, 0x00},  /* "J" */
    {0x0E, 0x22},  /* "K" */
    {0x1C, 0x00},  /* "L" */
    {0x6C, 0xA0},  /* "M" */
    {0x6C, 0x82},  /* "N" */
    {0xFC, 0x00},  /* "O" */
    {0xCF, 0x00},  /* "P" */
    {0xFC, 0x02},  /* "Q" */
    {0xCF, 0x02},  /* "R" */
    {0xB7, 0x00},  /* "S" */
    {0x80, 0x50},  /* "T" */
    {0x7C, 0x00},  /* "U" */
    {0x0C, 0x28},  /* "V" */
    {0x6C, 0x0A},  /* "W" */
    {0x00, 0xAA},  /* "X" */
    {0x00, 0xB0},  /* "Y" */
    {0x90, 0x28}   /* "Z" */
};

// Symbole: pro Zeile: id, index com
const char symbolTab[24][3] = {
    {LCD_TMR_ID, LCD_TMR_IDX, LCD_TMR_COM},
    {LCD_HRT_ID, LCD_HRT_IDX, LCD_HRT_COM},
    {LCD_REC_ID, LCD_REC_IDX, LCD_REC_COM},
    {LCD_EXCLAMATION_ID, LCD_EXCLAMATION_IDX, LCD_EXCLAMATION_COM},
    {LCD_BRACKETS_ID, LCD_BRACKETS_IDX, LCD_BRACKETS_COM},
    {LCD_BATT_ID, LCD_BATT_IDX, LCD_BATT_COM},
    {LCD_B1_ID, LCD_B1_IDX, LCD_B1_COM},
    {LCD_B2_ID, LCD_B2_IDX, LCD_B2_COM},
    {LCD_B3_ID, LCD_B3_IDX, LCD_B3_COM},
    {LCD_B4_ID, LCD_B4_IDX, LCD_B4_COM},
    {LCD_B5_ID, LCD_B5_IDX, LCD_B5_COM},
    {LCD_B6_ID, LCD_B6_IDX, LCD_B6_COM},
    {LCD_ANT_ID, LCD_ANT_IDX, LCD_ANT_COM},
    {LCD_TX_ID, LCD_TX_IDX, LCD_TX_COM},
    {LCD_RX_ID, LCD_RX_IDX, LCD_RX_COM},
    {LCD_NEG_ID, LCD_NEG_IDX, LCD_NEG_COM},
    {LCD_DEG_ID, LCD_DEG_IDX, LCD_DEG_COM},
    {LCD_A1DP_ID, LCD_A1DP_IDX, LCD_A1DP_COM},
    {LCD_A2DP_ID, LCD_A2DP_IDX, LCD_A2DP_COM},
    {LCD_A3DP_ID, LCD_A3DP_IDX, LCD_A3DP_COM},
    {LCD_A4DP_ID, LCD_A4DP_IDX, LCD_A4DP_COM},
    {LCD_A5DP_ID, LCD_A5DP_IDX, LCD_A5DP_COM},
    {LCD_A2COL_ID, LCD_A2COL_IDX, LCD_A2COL_COM},
    {LCD_A4COL_ID, LCD_A4COL_IDX, LCD_A4COL_COM},
};

void LCD_Init(void) {
  LCDCCTL0 &= ~LCDON; // switch LCD off
  LCDCPCTL0 = 0xFFFF; // LCD pins 0 to 15 enable       |
  LCDCPCTL1 = 0xFC3F; // LCD pins 16 to 21 enable, 26-31
  LCDCPCTL2 = 0x0FFF;   // LCD pins 32 to 43 enable
  
  LCDCCTL0 = LCDDIV_0|LCDPRE__16|LCD4MUX|LCDLP;   // clock divider: 1,Prescaler 16, 4-Mux, Clock: ACLK
  
  LCDCVCTL =VLCDREF_0 |VLCD3 |LCDCPEN; // charge pump enable, internal ref. voltage; V2-V4 internally generated, not switched to pins
                             // V5 is VSS

  LCDCMEMCTL = LCDCLRM; // clear LCD memory

  LCDCCPCTL = LCDCPCLKSYNC; // enable clock synchronization
  
  LCDCCTL0 |= LCDON;  // switch LCD on
}

/*
 * Displays input character at given LCD digit/position
 * Only spaces, numeric digits, and uppercase letters are accepted characters
 * position: 1-6
 */
void LCD_showChar(char c, int position) {
    LCDMEM[lcdPos[position-1]] &= ~BITS_ALPHANUM_LO;
    LCDMEM[1+lcdPos[position-1]] &= ~BITS_ALPHANUM_HI;

    if (c == ' ')   // Blank
        return;
    else if (c >= '0' && c <= '9')     {
        // Display digit
        LCDMEM[lcdPos[position-1]] |= digit[c-48][0];
        LCDMEM[1+lcdPos[position-1]] |= digit[c-48][1];
    }
    else if (c >= 'A' && c <= 'Z')    {
        // Display alphabet
        LCDMEM[lcdPos[position-1]] |= alphabetBig[c-65][0];
        LCDMEM[1+lcdPos[position-1]] |= alphabetBig[c-65][1];
    }
    else  {
        // Turn all segments on if character is not a space, digit, or uppercase letter
        LCDMEM[lcdPos[position-1]]   |= BITS_ALPHANUM_LO;
        LCDMEM[1+lcdPos[position-1]] |= BITS_ALPHANUM_HI;
    }
}


/*
 * Clears memories to all 6 digits on the LCD
 */
void LCD_clear(void) {
    LCDCMEMCTL |= LCDCLRM; // clear LCD memory
}

void LCD_displayShort(signed short s) {
  int pos = 5;
  char sign = 0;
  LCD_clear();
  if (s < 0) {
    s = -s;
    sign = 1;
  }
  
  if (s==0) {
    LCD_showChar('0', 1);
    LCD_showChar('0', 2);
    return;
  }
  
  while (s != 0) {
      char rem = s % 10;
      LCD_showChar(rem+'0', pos--);
      s= s/10;
  }

  if (sign)
      LCD_Symbol(LCD_NEG_ID,LCD_SYMBOL_ON);

}

void LCD_displayString(char* pStr) {
  int i=0;
  LCD_clear();
  while (pStr[i] != '\0') {
      LCD_showChar(pStr[i], i+1);
              i++;
  }
}

char toHexDigit(char c) {
  if (c < 10) 
    return c + '0';
  else 
    return c -10 + 'A';
}

void LCD_displayShortHex(char* pStr, int s) {
  int i=0;
  char c;
  LCD_clear();
  while (pStr[i] != '\0') {
      LCD_showChar(pStr[i], i+1);
      i++;
  }
  i++;
  c = toHexDigit((s >> 12) & 0x000F);
  LCD_showChar(c, i++);
  c = toHexDigit((s >> 8) & 0x000F);
  LCD_showChar(c, i++);
  c = toHexDigit((s >> 4) & 0x000F);
  LCD_showChar(c, i++);
  c = toHexDigit(s & 0x000F);
  LCD_showChar(c, i++);
}


void LCD_Symbol(char id, char operation) {
   int len = sizeof(symbolTab)/3;
   int i;
   for (i=0;i<len;i++) {
       if (id == symbolTab[i][0]) {
           char idx = symbolTab[i][1];
           char bit = symbolTab[i][2];

           switch(operation) {
           case LCD_SYMBOL_ON:
               LCDMEM[idx] |=bit;
               break;
           case LCD_SYMBOL_OFF:
               LCDMEM[idx] &= ~bit;
               break;
           case LCD_SYMBOL_TOGGLE:
               LCDMEM[idx] ^=bit;
               break;
           }

       }
   }
}
