/*
 * https://github.com/dsoze1138/24FJ128GA204_DM240004
 *
 * File:   main.c
 * Author: dan1138
 * Target: PIC24FJ128GA204
 * Compiler: XC16 v1.36
 * IDE: MPLABX v5.25
 *
 * Created on October 29, 2019, 12:27 PM
 *
 *                                                           PIC24FJ128GA204
 *               +----------------+               +--------------+                +-----------+                +--------------+
 *         <>  1 : RB9/RP9        :    LED2 <> 12 : RA10         :          <> 23 : RB2/RP2   : X2-32KHZ <> 34 : RA4/SOSCO    :
 *  RGB_G  <>  2 : RC6/RP22       :  LCD_RS <> 13 : RA7          :          <> 24 : RB3/RP3   :     LED1 <> 35 : RA9          :
 *  RGB_B  <>  3 : RC7/RP23       :  LCD_RW <> 14 : RB14/RP14    :      POT <> 25 : RC0/RP16  :          <> 36 : RC3/RP19     :
 *     S2  <>  4 : RC8/RP24       :         <> 15 : RB15/RP15    :          <> 26 : RC1/RP17  :          <> 37 : RC4/RP20     :
 *     S1  <>  5 : RC9/RP25       :     GND -> 16 : AVSS         :          <> 27 : RC2/RP18  :    RGB_R <> 38 : RC5/RP21     :
 *    3v3  <>  6 : VBAT           :     3v3 -> 17 : AVDD         :      3v3 -> 28 : VDD       :      GND -> 39 : VSS          :
 *   10uF  ->  7 : VCAP           : ICD_VPP -> 18 : MCLR         :      GND -> 29 : VSS       :      3v3 -> 40 : VDD          :
 *         <>  8 : RB10/RP11/PGD2 :  LCD_D4 <> 19 : RA0/AN0      :   LCD_D6 <> 30 : RA2/OSCI  :          <> 41 : RB5/RP5/PGD3 :
 *         <>  9 : RB11/RP11/PGC2 :  LCD_D5 <> 20 : RA1/AN1      :   LCD_D7 <> 31 : RA3/OSCO  :          <> 42 : RB6/RP6/PGC3 :
 *         <> 10 : RB12/RP12      : ICD_PGD <> 21 : RB0/RP0/PGD1 :   LCD_E  <> 32 : RA8       :          <> 43 : RB7/RP7      :
 *         <> 11 : RB13/RP13      : ICD_PGC <> 22 : RB1/RP1/PGC1 : X2-32KHZ <> 33 : RB4/SOSCI :          <> 44 : RB8/RP8      :
 *               +----------------+               +--------------+                +-----------+                +--------------+
 *                                                              TQFP-44
 * 
 * Description:
 * 
 * Bare metal initialization of the DM240004 Curiosity Board
 * 
 * Setup the system oscillator for 32MHz using the internal FRC and the 4x PLL.
 * Turn on the 32.768KHz secondary oscillator.
 * Flash LED1 on for 500 milliseconds then off for 500 milliseconds.
 * Add 4-bit parallel HD44780 interface for LCD character module.
 */
// CONFIG4
#pragma config DSWDTPS = DSWDTPS1F      // Deep Sleep Watchdog Timer Postscale Select bits (1:68719476736 (25.7 Days))
#pragma config DSWDTOSC = LPRC          // DSWDT Reference Clock Select (DSWDT uses LPRC as reference clock)
#pragma config DSBOREN = OFF            // Deep Sleep BOR Enable bit (DSBOR Disabled)
#pragma config DSWDTEN = OFF            // Deep Sleep Watchdog Timer Enable (DSWDT Disabled)
#pragma config DSSWEN = OFF             // DSEN Bit Enable (Deep Sleep operation is always disabled)
#pragma config PLLDIV = PLL4X           // USB 96 MHz PLL Prescaler Select bits (4x PLL selected)
#pragma config I2C1SEL = DISABLE        // Alternate I2C1 enable bit (I2C1 uses SCL1 and SDA1 pins)
#pragma config IOL1WAY = OFF            // PPS IOLOCK Set Only Once Enable bit (The IOLOCK bit can be set and cleared using the unlock sequence)

// CONFIG3
#pragma config WPFP = WPFP127           // Write Protection Flash Page Segment Boundary (Page 127 (0x1FC00))
#pragma config SOSCSEL = ON             // SOSC Selection bits (SOSC circuit selected)
#pragma config WDTWIN = PS25_0          // Window Mode Watchdog Timer Window Width Select (Watch Dog Timer Window Width is 25 percent)
#pragma config PLLSS = PLL_FRC          // PLL Secondary Selection Configuration bit (PLL is fed by the on-chip Fast RC (FRC) oscillator)
#pragma config BOREN = OFF              // Brown-out Reset Enable (Brown-out Reset Disabled)
#pragma config WPDIS = WPDIS            // Segment Write Protection Disable (Disabled)
#pragma config WPCFG = WPCFGDIS         // Write Protect Configuration Page Select (Disabled)
#pragma config WPEND = WPENDMEM         // Segment Write Protection End Page Select (Write Protect from WPFP to the last page of memory)

// CONFIG2
#pragma config POSCMD = NONE            // Primary Oscillator Select (Primary Oscillator Disabled)
#pragma config WDTCLK = LPRC            // WDT Clock Source Select bits (WDT uses LPRC)
#pragma config OSCIOFCN = ON            // OSCO Pin Configuration (OSCO/CLKO/RA3 functions as port I/O (RA3))
#pragma config FCKSM = CSECMD           // Clock Switching and Fail-Safe Clock Monitor Configuration bits (Clock switching is enabled, Fail-Safe Clock Monitor is disabled)
#pragma config FNOSC = FRC              // Initial Oscillator Select (Fast RC Oscillator (FRC))
#pragma config ALTCMPI = CxINC_RB       // Alternate Comparator Input bit (C1INC is on RB13, C2INC is on RB9 and C3INC is on RA0)
#pragma config WDTCMX = WDTCLK          // WDT Clock Source Select bits (WDT clock source is determined by the WDTCLK Configuration bits)
#pragma config IESO = OFF               // Internal External Switchover (Disabled)

// CONFIG1
#pragma config WDTPS = PS32768          // Watchdog Timer Postscaler Select (1:32,768)
#pragma config FWPSA = PR128            // WDT Prescaler Ratio Select (1:128)
#pragma config WINDIS = OFF             // Windowed WDT Disable (Standard Watchdog Timer)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT disabled in hardware; SWDTEN bit disabled)
#pragma config ICS = PGx1               // Emulator Pin Placement Select bits (Emulator functions are shared with PGEC1/PGED1)
#pragma config LPCFG = OFF              // Low power regulator control (Disabled - regardless of RETEN)
#pragma config GWRP = OFF               // General Segment Write Protect (Write to program memory allowed)
#pragma config GCP = OFF                // General Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF             // JTAG Port Enable (Disabled)
/*
 * Defines for system oscillator frequency.
 * Make sure that the PIC initialization selects this frequency.
 */
#define FSYS (32000000ul)
#define FCY  (FSYS/2ul)
/*
 * Target specific Special Function Register definotions
 */
#include <xc.h>
/*
 * Standard header files
 */
#include <stdint.h>
/*
 * Local header files
 */
#include "lcd.h"
/*
 * Initialize this PIC
 */
void PIC_Init(void)
{
    uint16_t ClockSwitchTimeout;

    /* 
     * Disable all interrupt sources 
     */ 
    __builtin_disi(0x3FFF); /* disable interrupts for 16383 cycles */ 
    IEC0 = 0; 
    IEC1 = 0; 
    IEC2 = 0; 
    IEC3 = 0; 
    IEC4 = 0; 
    IEC5 = 0; 
    IEC6 = 0; 
    IEC7 = 0; 
    __builtin_disi(0x0000); /* enable interrupts */ 
    
    INTCON1bits.NSTDIS = 1; /* Disable interrupt nesting */
    
    /*
     * At Power On Reset the configuration words set the system clock
     * to use the FRC oscillator. At this point we need to enable the
     * PLL to get the system clock running at 32MHz.
     * 
     * Clock switching on the 24FJ family with the PLL can be a bit tricky.
     * 
     * First we need to check if the configuration words enabled clock
     * switching at all, then turn off the PLL, then setup the PLL and
     * finally enable it. Sounds simple, I know. Make sure you verify this 
     * clock setup on the real hardware.
     */

    if(!OSCCONbits.CLKLOCK) /* if primary oscillator switching is unlocked */
    {
        /* Select primary oscillator as FRC */
        __builtin_write_OSCCONH(0b000);

        /* Request switch primary to new selection */
        __builtin_write_OSCCONL(OSCCON | (1 << _OSCCON_OSWEN_POSITION));

        /* wait, with timeout, for clock switch to complete */
        for(ClockSwitchTimeout=10000; --ClockSwitchTimeout && OSCCONbits.OSWEN;);

        CLKDIV   = 0x0000; /* set for FRC clock 8MHZ operations */

        /* Select primary oscillator as FRCPLL */
        __builtin_write_OSCCONH(0b001);
        /* Request switch primary to new selection */
        __builtin_write_OSCCONL(OSCCON | (1 << _OSCCON_OSWEN_POSITION));
        
        /* ALERT: This may be required only when the 96MHz PLL is used */
        CLKDIVbits.PLLEN = 1;

        /* wait, with timeout, for clock switch to complete */
        for(ClockSwitchTimeout=10000; --ClockSwitchTimeout && OSCCONbits.OSWEN;);

        /* wait, with timeout, for the PLL to lock */
        for(ClockSwitchTimeout=10000; --ClockSwitchTimeout && !OSCCONbits.LOCK;);
        
        /* at this point the system oscillator should be 32MHz */
    }
    
    /* Turn on Secondary Oscillation Amplifier */
    __builtin_write_OSCCONL(OSCCON | (1 << _OSCCON_SOSCEN_POSITION));
    
    /* Turn off all analog inputs */
    ANSA = 0;
    ANSB = 0;
    ANSC = 0;
}
/*
 * WARNING: Not a portable function.
 *          Maximum delay 16384 instruction cycles.
 *          At 16 MIPS this is 1024 microseconds.
 *
 *          Minimum  1MHz instruction cycle clock.
 */
void delay_us(unsigned short delay)
{
    if ((delay * (FCY/1000000ul) > 16383ul))
    {
        asm("   repeat  #16383\n"
            "   clrwdt        \n"
            ::);

    }
    else
    {
        asm("   repeat  %0    \n"
            "   clrwdt        \n"
        :: "r" (delay*(FCY/1000000ul)));
    }
}
/*
 * WARNING: Not a portable function.
 *          Maximum 16MHz instruction cycle clock.
 *          Minimum  8Khz instruction cycle clock.
 */
void delay_ms(unsigned long delay)
{
    if(delay--)
    {
        asm("1: repeat  %0    \n"
            "   clrwdt        \n"
            "   sub  %1,#1    \n"
            "   subb %d1,#0   \n"
            "   bra  c,1b     \n"
        :: "r" (FCY/1000ul-6ul), "C" (delay));
    }
}
/*
 * Main Application
 */
int main(void) 
{
    /*
     * Application initialization
     */
    PIC_Init();
    LCD_Init();
    
    LCD_SetDDRamAddr(LINE_ONE);
    LCD_WriteString("LCD Test v1.0");
    
    /* Set RA9 for output to drive LED1 */
    LATAbits.LATA9 = 0;
    TRISAbits.TRISA9 = 0;
    
    /*
     * Application process loop
     */
    for(;;)
    {
        LATAbits.LATA9 ^= 1;
        delay_ms(500);
    }
    return 0;
}
