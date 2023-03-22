/*
 * Part2.c
 *
 *  Created on: March 1, 2023
 *      Author: David Miller
 *
 *      This code will need to change the speed of an LED between 3 different speeds by pressing a button.
 */

#include <msp430.h>

void gpioInit();
void timerInit();

int blink_speed = 0;    // Speed of LED blinking

void main(){

    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer

    gpioInit();
    timerInit();

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    __bis_SR_register(LPM3_bits | GIE);

}


void gpioInit(){
    // @TODO Initialize the Red or Green LED
    P1OUT &= ~BIT0;     // Clear P1.0 output latch for a defined power-on state
    P1DIR |= BIT0;      // Set Pin P1.0 as output for the LED

    // @TODO Initialize Button 2.3
    P2OUT |= BIT3;                          // Configure P2.3 as pulled-up
    P2REN |= BIT3;                           // P2.3 pull-up register enable
    P2IES &= ~BIT3;                         // P2.3 Low --> High edge
    P2IE |= BIT3;                           // P2.3 interrupt enabled
}

void timerInit(){
    // @TODO Initialize Timer B1 in Continuous Mode using ACLK as the source CLK with Interrupts turned on
    TB1CCR0 = 50000;      // Set the maximum count for Timer B1
    TB1CTL = TBSSEL_1 | MC_2;  // ACLK, continuous mode
    TB1CCTL0 = CCIE;       // TBCCR0 interrupt enabled
}


/*
 * INTERRUPT ROUTINES
 */

// Port 2 interrupt service routine
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
    // @TODO Remember that when you service the GPIO Interrupt, you need to set the interrupt flag to 0.
    P2IFG &= ~BIT3;     // Clear the interrupt flag for Pin P2.3
    // @TODO When the button is pressed, you can change what the CCR0 Register is for the Timer. You will need to track what speed you should be flashing at.
    blink_speed = blink_speed + 1;      // Increase the blink speed counter
}


// Timer B1 interrupt service routine
#pragma vector = TIMER1_B0_VECTOR
__interrupt void Timer1_B0_ISR(void)
{
    // @TODO You can toggle the LED Pin in this routine and if adjust your count in CCR0.
    P1OUT ^= BIT0;      // Toggle the LED Pin P1.0

    if (blink_speed == 1) {
        TB1CCR0 += 40000;   // Set CCR0 to blink slowly
    }
    else if (blink_speed == 2) {
        TB1CCR0 += 2000;   // Set CCR0 to blink at medium speed
    }
    else if (blink_speed == 3) {
        TB1CCR0 += 100;   // Set CCR0 to blink quickly
        blink_speed = 0;    // Reset blink speed counter
    }
}

