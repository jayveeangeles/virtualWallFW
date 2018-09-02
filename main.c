/* 
 * File:   main.c
 * Author: vincentj
 *
 * Created on August 7, 2018, 6:49 PM
 */

#define _XTAL_FREQ 2000000  

#include <stdlib.h>
#include <xc.h>
#include <pic12f683.h>

// CONFIG
#pragma config FOSC = INTOSCIO  // Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA4/OSC2/CLKOUT pin, I/O function on RA5/OSC1/CLKIN)
#pragma config WDTE = OFF        // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select bit (MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Detect (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)

#define TRUE 1
#define FALSE 0
#define PWM_OFF CCP1CON = 0x10
#define PWM_ON CCP1CON = 0x1c
#define VIRTUAL_WALL_CMD 162
#define GET_BIT(x, y) x & (1 << y);
#define T_ACQ 5
#define DURATION_SECONDS 240
#define DURATION_MINUTES 120
#define LV_THRESH 553 // 2.7/5 * 1024

void interrupt isr(void) {
    if (PIR1bits.TMR1IF == 1) // timer 1 interrupt flag
    {
        PIR1bits.TMR1IF = 0;           // interrupt must be cleared by software
        TMR1H = 0xE0;             // preset for timer1 MSB register
        TMR1L = 0;             // preset for timer1 LSB register
    }
};

/*
 * 
 */
int main(int argc, char** argv) {
    unsigned int i;
    unsigned int bitValue;
    unsigned int adval;
    unsigned int secondsCounter;
    unsigned int minutesCounter;

    // General init
    OSCCON = 0x51; // Internal 2MHz osc.
    ADCON0 = 0; // all pins digital
    ANSEL = 0; // all pins digital
    ANSELbits.ADCS = 0b001; // FOSC/8
    ANSELbits.ANS0 = 1; // analog AN0
    CMCON0 = 7; // Comparators off.
    TRISIO = 0x0; // all output except GP3
    TRISIObits.TRISIO0 = 1; // GP0 as input
    GPIO = 0; // all pins low
    
    // PWM init
    PR2 = 0b00001100; // Set up PWM for roughly 38kHz
    T2CON = 0b00000100;
    CCPR1L = 0b00000110;
    WDTCON = 0b00010000; // WDT Prescalar = 1000 = 8192 = 272ms
                         // 17ms x (8192 / 512[base])
    PSA = 0; // Assign prescalar to Timer0
    PWM_OFF;
    
    //Timer1 Registers 0.25s
    // 1/32768 * prescale * (2^16 - preload timer)
    T1CONbits.T1CKPS1 = 0;   // bits 5-4  Prescaler Rate Select bits
    T1CONbits.T1CKPS0 = 0;   // bit 4
    T1CONbits.T1OSCEN = 1;   // bit 3 Timer1 Oscillator Enable Control bit 1 = on
    T1CONbits.nT1SYNC = 1;    // bit 2 Timer1 External Clock Input Synchronization Control bit...1 = Do not synchronize external clock input
    T1CONbits.TMR1CS = 1;    // bit 1 Timer1 Clock Source Select bit...1 = External clock
    TMR1H = 0xE0;             // preset for timer1 MSB register
    TMR1L = 0;             // preset for timer1 LSB register
    
    // Interrupt Registers
    INTCON = 0;           // clear the interrpt control register
    INTCONbits.TMR0IE = 0;        // bit5 TMR0 Overflow Interrupt Enable bit...0 = Disables the TMR0 interrupt
    INTCONbits.TMR0IF = 0;        // bit2 clear timer 0 interrupt flag
    PIR1bits.TMR1IF = 0;            // clear timer1 interupt flag TMR1IF
    PIE1bits.TMR1IE = 1;                // enable timer 1 interrupt
    INTCONbits.GIE = 1;           // bit7 global interrupt enable
    INTCONbits.PEIE = 1;          // bit6 Peripheral Interrupt Enable bit...1 = Enables all unmasked peripheral interrupts
    
    ADCON0bits.ADFM = 1;
    ADCON0bits.VCFG = 0;
    ADCON0bits.CHS = 0b00;
    ADCON0bits.ADON = 1;

    __delay_us(T_ACQ);
    for (minutesCounter = 0; minutesCounter < DURATION_MINUTES; minutesCounter++) {
        for (secondsCounter = 0; secondsCounter < DURATION_SECONDS; secondsCounter++)
        {
            if ((secondsCounter % 5) == 0)
            {
                ADCON0bits.GO_nDONE = 1;
                while(ADCON0bits.GO_nDONE){}
                adval = ((unsigned int) ((ADRESH << 8) + ADRESL));
                if (adval >= LV_THRESH)
                {
                    GPIObits.GP1 = 0;
                } 
                else
                {
                    GPIObits.GP1 = 1;
                }
            }

            for (i = 0; i < 8; i++) // Send a few IR bursts
            {
                bitValue = GET_BIT(VIRTUAL_WALL_CMD, i);
                if (bitValue == 0) {
                    PWM_ON;
                    __delay_us(1000);
                    PWM_OFF;
                    __delay_us(3000);
                } else {
                    PWM_ON;
                    __delay_us(3000);
                    PWM_OFF;
                    __delay_us(1000);
                }
            }

            if (GPIObits.GP1)
                GPIObits.GP1 = 0;

            // Then sleep for a moment
            T1CONbits.TMR1ON = 1;    // bit 0 enables timer
            SLEEP();
            T1CONbits.TMR1ON = 0;
        }
    }
    
    ADCON0bits.ADON = 0;
    SLEEP();
    return (EXIT_SUCCESS);
}