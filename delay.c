/*
 * delay.c
 *
 *  Created on: Mar 2, 2014
 *      Author: Tim
 */


void delay(int ms){ // Delays by the specified Milliseconds

	while (ms--) {
		__delay_cycles(1000); //set to 1000 for 1 Mhz
	}
}
