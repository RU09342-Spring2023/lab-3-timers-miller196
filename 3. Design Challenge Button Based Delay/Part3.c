/*
 * Part3.c
 *
 *  Created on: March 2, 2023
 *      Author: David Miller
 */
#include <msp430.h>
#include <msp430fr2355.h>

unsigned int bp = 50000; //button press
unsigned int bpt = 0; // button press time
unsigned const int bpi = 50000;//blink period initial
void timerInit();
void gpioInit();
void main(){

    WDTCTL = WDTPW + WDTHOLD;   // Stop watchdog timer

    PM5CTL0 &= ~LOCKLPM5;    // Disable the GPIO power-on default high-impedance mode to activate previously configured port settings

    gpioInit();
    timerInit();
    __enable_interrupt();       // Enable global interrupts

    while(1) {
        __bis_SR_register(LPM3_bits | GIE); // Enter low power mode with interrupts enabled
    }
}

void gpioInit(){
    //Initialize the Red and Green LED
    //Configure RED LED on P1.0 as Output
    P1OUT &= ~BIT0;                         // Clear P1.0 output latch for a defined power-on state
    P1DIR |= BIT0;                          // Set P1.0 to output direction

    //Configure Green LED on P6.6 as Output
    P6OUT &= ~BIT6;                         // Clear P6.6 output latch for a defined power-on state
    P6DIR |= BIT6;                          // Set P6.6 to output direction

    //Initialize Button 2.3
    P2OUT |= BIT3;                          // Configure P2.3 as pulled-up
    P2REN |= BIT3;                          // P2.3 pull-up register enable
    //P2IES &= ~BIT3;                         // P2.3 Falling edge - button rises up
    P2IES |= BIT3;                          // P2.3 Rising edge - button press down
    P2IE |= BIT3;                           // P2.3 interrupt enabled

    //Initialize Button 4.1
    P4OUT |= BIT1;                          // Configure P2.3 as pulled-up
    P4REN |= BIT1;                          // P4.1 pull-up register enable
    //P4IES &= ~BIT1;                         // P4.1 Falling edge - button rises up
    P4IES |= BIT1;                          // P4.1 Rising edge - button press down
    P4IE |= BIT1;                           // P4.1 interrupt enabled

}

void timerInit(){
    //Initialize Timer B0
    TB0CCR0 = 300;  //arbitrary initial value
    TB0CTL |= TBSSEL_1 + MC_0 + ID_3 + TBCLR; // Set the clock source to ACLK, stop the timer, and clear it
    TB0CCTL0 |= CAP + CM_3; // Set Timer B0 to capture mode, set for both edges

    //Initialize Timer B1
    TB1CCR0 = bp;      // Set the max count for Timer B1
    TB1CTL = TBSSEL_1 | MC_1 | ID_3;  // ACLK, up mode
    TB1CCTL0 = CCIE;       // TBCCR0 interrupt enabled
}

// button count interrupt
#pragma vector = PORT2_VECTOR
__interrupt void Port2_ISR(void)
{
    if (P2IES & BIT3)
    {
        TB0CTL |= MC_2; //start timer B0 to measure how long the button is pressed

        P2IES &= ~BIT3; // Change edge to falling edge
    }
    else if (P2IES != BIT3) //check if the interrupt was triggered off a falling edge
    {
        TB0CTL &= ~(MC0 + MC1); //stop timer B0 to record how long the button was pressed


        //Put timer B0 value into the max timer B1 CCR0
        bpt = TB0R; // Record the button press time
        TB1CCR0 = bpt; // Set the max count for Timer B1 to the button press time

        TB0CTL |= TBCLR; //clear timer B0 for next button press

        P2IES |= BIT3; // Change edge sensitivity to look for next rising edge
    }

    P6OUT ^= BIT6;           // P6.6 = toggle - to test to see if it is reading the button press
    P2IFG &= ~BIT3;         // Clear interrupt flag for P2.3
}

// Interrupt port 4 reset
#pragma vector = PORT4_VECTOR
__interrupt void Port4_ISR(void)
{
    bp = bpi;
    TB1CCR0 = bp;


    TB0CTL &= ~(MC0 + MC1); // Stop Timer B0 - just in case
    TB0CTL |= TBCLR; // Clear Timer B0

    P4IFG &= ~BIT1;         // Clear interrupt flag for P4.1
}

// Timer B interrupts
#pragma vector = TIMER1_B0_VECTOR
__interrupt void Timer1_B0_ISR(void)
{
    P1OUT ^= BIT0;          // Toggle P1.0 LED

}





