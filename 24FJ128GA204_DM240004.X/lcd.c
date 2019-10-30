/*
 * File:   lcd.c
 *
 */

#include <xc.h>
#include "lcd.h"

#if (LCD_DATA_BITS == 0x0F)
  #define LCD_DATA_ON_LOW_4_BITS
#else
  #if (LCD_DATA_BITS == 0xF0)
    #define LCD_DATA_ON_HIGH_4_BITS
  #else
    #error LCD interface supports 4-bit mode only on high or low 4-bits of one port
  #endif
#endif

static unsigned char LCD_BusyBitMask;

static void LCD_E_Pulse(void)
{
    E_PIN = 1;
    delay_us(4);
    E_PIN = 0;
    delay_us(4);
}

static void LCD_DelayPOR(void)
{
    delay_ms(15);
}

static void LCD_Delay(void)
{
    delay_ms(5);
}

static unsigned char LCD_GetByte(void)
{
    unsigned char LCD_Data;
    
    LCD_PORT_DIR |= LCD_DATA_BITS; /* make LCD data bits inputs */
    RW_PIN = 1;
    
    E_PIN = 1;
    delay_us(4);
    LCD_Data = (unsigned char)(LCD_PORT_IN & LCD_DATA_BITS);
    E_PIN = 0;
    delay_us(4);
    
    LCD_Data = (unsigned char)((LCD_Data >> 4) | (LCD_Data << 4));
    
    E_PIN = 1;
    delay_us(4);
    LCD_Data |= (unsigned char)(LCD_PORT_IN & LCD_DATA_BITS);
    E_PIN = 0;
    delay_us(4);

#ifdef LCD_DATA_ON_HIGH_4_BITS
    LCD_Data = ((LCD_Data >> 4) | (LCD_Data << 4));
#endif
    return LCD_Data;
}

static void LCD_PutByte(unsigned char LCD_Data)
{
#ifdef LCD_DATA_ON_HIGH_4_BITS
    LCD_Data = ((LCD_Data >> 4) | (LCD_Data << 4));
#endif

    LCD_PORT_DIR &= ~LCD_DATA_BITS; /* make LCD data bits outputs */
    RW_PIN = 0;
    
    /* send first nibble */
    LCD_PORT_OUT &= ~LCD_DATA_BITS;
    LCD_PORT_OUT |= (unsigned char)(((LCD_Data >> 4) | (LCD_Data << 4)) & LCD_DATA_BITS);
    LCD_E_Pulse();
    
    LCD_PORT_OUT &= ~LCD_DATA_BITS;
    LCD_PORT_OUT |= (unsigned char)((LCD_Data) & LCD_DATA_BITS);
    LCD_E_Pulse();

    LCD_PORT_DIR |= LCD_DATA_BITS; /* make LCD data bits inputs */
}

static void LCD_Busy(void)
{
    if (LCD_BusyBitMask)
    {
        unsigned char LCD_Data;
    
        LCD_PORT_DIR |= LCD_DATA_BITS; /* make LCD data bits inputs */
        LCD_Data = 0;

        RS_PIN = 0;
        RW_PIN = 1;
        do
        {
            LCD_Data = LCD_GetByte();
        } while (LCD_Data & LCD_BusyBitMask);
    }
    else
    {
        LCD_Delay(); /* use 5ms delay when busy bit is unknown */
    }
}

void LCD_Init(void) 
{
    unsigned char LCD_Data;
    
    LCD_BusyBitMask = 0;
    LCD_PORT_DIR &= ~LCD_DATA_BITS; /* make LCD data bits outputs */
    E_PIN_DIR = 0;                  /* make LCD Enable strobe an output */
    RW_PIN_DIR = 0;                 /* make LCD Read/Write select an output */
    RS_PIN_DIR = 0;                 /* make LCD Register select an output */

    E_PIN = 0;                      /* set LCD Enable strobe to not active */
    RW_PIN = 0;                     /* set LCD Read/Write select to Write */
    RS_PIN = 0;                     /* set LCD Register select to command group */
    LCD_PORT_OUT &= ~LCD_DATA_BITS; /* set LCD data bits to zero */
    LCD_DelayPOR();                 /* wait for LCD power on to complete */

    /* Force LCD to 8-bit mode */
    LCD_PORT_OUT &= ~LCD_DATA_BITS; /* set LCD data bits to zero */
    LCD_PORT_OUT |= (0b00110011 & LCD_DATA_BITS);
    LCD_E_Pulse();
    LCD_Delay();
    LCD_E_Pulse();
    LCD_Delay();
    LCD_E_Pulse();
    LCD_Delay();
    
    /* Set LCD to 4-bit mode */
    LCD_PORT_OUT &= ~LCD_DATA_BITS; /* set LCD data bits to zero */
    LCD_PORT_OUT |= (0b00100010 & LCD_DATA_BITS);
    LCD_E_Pulse();
    LCD_Delay();

    /* Initialize LCD mode */
    LCD_WriteCmd(LCD_FORMAT);

    /* 
     * Find position of busy bit.
     * Required when using 4-bit mode.
     */
    LCD_SetDDRamAddr(LINE_ONE+1);
    LCD_Busy();
    RS_PIN = 0;
    LCD_Data = LCD_GetByte();

    if (LCD_Data == 0x01)
    {
        LCD_BusyBitMask = 0x80;
    }
    else
    {
        if (LCD_Data == 0x10)
        {
            LCD_BusyBitMask = 0x08;
        }
    }

    /* Turn on display, Setup cursor and blinking */
    LCD_WriteCmd(DOFF & CURSOR_OFF & BLINK_OFF);
    LCD_WriteCmd(DON & CURSOR_OFF & BLINK_OFF);
    LCD_WriteCmd(CLEAR_DISPLAY);
    LCD_WriteCmd(SHIFT_CUR_LEFT);

    /* Set first position on line one, left most character */
    LCD_SetDDRamAddr(LINE_ONE);
}

void LCD_SetCGRamAddr(unsigned char data)
{
    LCD_Busy();
    RS_PIN = 0;            
    LCD_PutByte((unsigned char)(data | 0x40));
}

void LCD_SetDDRamAddr(unsigned char data)
{
    LCD_Busy();
    RS_PIN = 0;
    LCD_PutByte((unsigned char)(data | 0x80));
}

void LCD_WriteCmd(unsigned char data)
{
    LCD_Busy();
    RS_PIN = 0;
    LCD_PutByte(data);
}

void LCD_WriteData(unsigned char data)
{
    LCD_Busy();
    RS_PIN = 1;
    LCD_PutByte(data);
    RS_PIN = 0;
}

void LCD_WriteString(char * pString)
{
    while(*pString)
    {
        LCD_WriteData(*pString);
        pString++;
    }
}
unsigned char LCD_GetBusyBitMask(void)
{
    return LCD_BusyBitMask;
}
