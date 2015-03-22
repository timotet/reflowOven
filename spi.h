/*
 * spi.h
 *
 *  Created on: April 5, 2014
 *      Author: Tim Toliver
 *
 *
 *  This sets up USCI module for SPI
 *  This particular setup is for a nokia 5110 lcd
 */

#ifndef SPI_H_
#define SPI_H_

#include <msp430G2553.h>

#define SCLK      BIT5    // Port 1.5 hardware SPI
#define MISO      BIT6    // Port 1.6 hardware SPI SOMI
#define MOSI      BIT7    // Port 1.7 hardware SPI SIMO


void initSPI8(void);


#endif /* SPI_H_ */
