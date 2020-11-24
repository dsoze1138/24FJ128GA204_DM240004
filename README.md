# PIC24FJ128GA204 in a DM240004 Curiosity Board

Bare metal initialization of the DM240004 Curiosity Board

Setup the system oscillator for 32MHz using the internal FRC and the 4x PLL.

Turn on the 32.768KHz secondary oscillator.

Flash LED1 on for 500 milliseconds then off for 500 milliseconds.

LCD character module interface using 4-bit parallel HD44780.

Add project for UART demo, see StackOverflow topic: 

https://stackoverflow.com/questions/64988847/cannot-send-or-receive-data-through-uart-on-pic24f-development-board