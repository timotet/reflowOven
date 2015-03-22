/*
 * spi.c
 *
 *  Created on: April 5, 2014
 *      Author: Tim Toliver
 *
 *  this sets up the USCI module for SPI
 *
 *  this particular setup is for a nokia 5110 lcd @ 4Mhz
 */

#include "spi.h"

void initSPI8(void){

	UCB0CTL1 |= UCSWRST; // reset USCI module

	P1SEL |= SCLK + MOSI + MISO;         // enable P1.5,6,7 for peripheral function
	P1SEL2 |= SCLK + MOSI + MISO;        // set up for SPI

	UCB0CTL0 |= UCCKPH + UCMSB + UCMST + UCSYNC; //data captured on 1st edge, MSB, master mode, 3wire SPI, synchronous mode
	UCB0CTL1 |= UCSSEL_2;                        // SMCLK
	UCB0BR0 |= 0x01;                             // SMCLK = 4Mhz/1 = 4Mhz
	UCB0BR1 = 0;
	UCB0CTL1 &= ~UCSWRST;                        // release USCI for use

	//IE2 |= UCB0RXIE + UCB0TXIE;                // Enable USCI0 RX and TX interrupts
	IE2 |= UCB0RXIE ;
	//IFG2 = 0;                                  // clear any pending flags

}



