/*
 * This compiles with CCS
 * 4/5/14
 * program for a DIY solder re-flow oven
 *
 * 1 talk to nokia 5011 LCD with  8 bit spi and clean up library
 *
 * 2 talk to MAX6675 thermocouple to digital converter via spi 12 bit resolution
 *
 * 3 rough out a PID algorithm (used PID from Tim Wescott's PID without a PhD)
 *
 * 4 set up button inputs and outputs for mosfet to fire SSD to control oven
 *
 * 5 set up an output for a buzzer
 *
 * 6 write a solder profile
 *
 * 7 melt solder
 *
 * Note: I am using floating point math in the PID algorithm. It seems the
 * Igain is best set as a fraction, I tried to use integer math but could not
 * get the desired results.
 * The float gets truncated to an int when the drive value is passed to the
 * relayDrive function.
 *
 */

#include <stdbool.h>
#include <math.h>
#include <msp430G2553.h>

#include "nokia5110.h"
#include "delay.h"
#include "spi.h"
#include "pid.h"
#include "leadPlot.h"
#include "rhosPlot.h"

#define pwm_1        BIT5     // port2 pin5    pwm out
#define relayOn      P2OUT |= pwm_1
#define relayOff     P2OUT &= ~pwm_1
#define toggleRelay  P2OUT ^= pwm_1
//#define pwm_2  BIT4   // port2 pin4    pwm out
#define pwmOn  1
#define pwmOff 0

#define maxOn       1
#define maxOff      0
#define max_CS      BIT3    // max chip select
#define max_CS_Lo   P1OUT &= ~max_CS
#define max_CS_Hi   P1OUT |= max_CS
#define buzzer      BIT2    // port1 pin2
#define button1     BIT0    // port1 pin0
#define button2     BIT1    // port1 pin1
// debug leds
#define dLed1  BIT3    // port3 pin3
#define dLed1_Hi      P3OUT |= dLed1
#define dLed1_Lo      P3OUT &= ~dLed1
#define dLed1_toggle  P3OUT ^= dLed1

#define dLed2  BIT4    // port3 pin4
#define dLed2_Hi      P3OUT |= dLed2
#define dLed2_Lo      P3OUT &= ~dLed2
#define dLed2_toggle  P3OUT ^= dLed2


enum display_mode {
	scroll, start, reflow, pid, RoHS, LEAD, plot, exit
};
enum display_mode mode = start;

enum screen_number {
	sStart = 1, sProfile = 3, sPID = 5
};
enum screen_number screenNum = sStart;

// for keeping track of menu items
static const unsigned char modeLookUp[13] = { exit, 0, reflow, 0, pid, 0, RoHS,
		0, 0, 0, 0, 0, LEAD };

//// address's in memory ///////
unsigned int * Flash_pGain = (unsigned int *) 0x1040; // info space C
float * Flash_iGain =        (float *) 0x1044;
unsigned int * Flash_dGain = (unsigned int *) 0x1050;

unsigned char sampCnt = 0;        // counter for sampling the max 6675 @ 2.75hz

unsigned int lowByte = 0;         // for max data tweaking
unsigned int highByte = 0;
unsigned int maxValue = 0;        // for max spi read
unsigned int result = 0;          // max temp result
unsigned char cnt = 0;            // for spi interrupt
static unsigned char tempInC[5];  // for temp after itoa
//unsigned int fahrenheit = 0;
//unsigned int lFahrenheit = 0;
//static unsigned char aFahrenheit[5];
//char tempString[3];
unsigned int count = 0;                              // for counting seconds during reflow cycle
static unsigned char aCount[5] = { 0, 0, 0, 0, 0 };  // for converting digits in itoa function
unsigned int max_read = 0;
unsigned char maxStat = 0;
unsigned char sPos = 0;            // y position to scroll from
unsigned char nLines = 0;          // # of lines to scroll through
unsigned char click = 0;           // keep track of the button presses
unsigned char select = 0;          // for selecting an option
unsigned char maxClick = 0;        // for max number of clicks
bool psFlag = false;
bool pwmFlag = false;
bool reflowFlag = false;
extern bool fontFlag = false;  // This switches between 3x5 font and 5x7 font

// PID stuff
SPid PlantPID;
unsigned int setPoint = 0;
//signed int lastDrive = 0;
//signed int drive = 0;
float lastDrive = 0;
float drive = 0;
static const unsigned int leadLook[5] = {125,185,185,0,0};   // goal temps for the 2 modes
static const unsigned int leadTime[5] = {120,250,275,480};   // to compare to count
static const unsigned int RohsLook[5] = {175,225,225,0,0};
static const unsigned int RohsTime[5] = {180,310,335,480};
unsigned int profileLook[5] = {0};                           // for loading in the reflow parameters
unsigned int time[4] = {0};
bool pidFlag = false;
signed int duty = 0;

// the reflow modes
enum PIDmode {
	preHeat, rampUp, soak, cool, done
};
enum PIDmode pidMode = done;


////////////////////////////function prototypes/////////////////////////

void loadFlash();
void initGPIO(void);                      // set up GPIO
unsigned int maxRead(void);               // read max6675 thermocouple amplifier
int itoa(signed int val, unsigned char *str); // int to ascii
//void ftoa(float f, unsigned char *buf, unsigned int decPlaces); // float to ascii
//unsigned int itof(unsigned int i);      // convert 12 bit integer to farenheight
void buzz(void);                          // for end of cycle alert
void stopBuzz(void);                      // stop the buzzer
void startPwm(void);                      // start the PWM for the oven
void stopPwm(void);                       // stop the PWM for the oven
void startSecTimer(void);                 // for second timer
void stopSecTimer(void);
void startSampler(void);
void stopSampler(void);
void startCapture(void);                // for checking button press
void startScreen(void);                 // start screen
void reflowScreen(void);                // reflow screen
void pidScreen(void);                   // PID screen
void profileScreen(void);               // profile screen
void Scroll(char curPos);               // scroll arrow on lcd for menu select
void LEADScreen(void);                  // for lead
void RoHSScreen(void);                  // for RoHS
void exitScreen(void);                  // for bail out
void relayDrive(float drive);           // for driving SSR
void loadProfile(const unsigned int *aTime, const unsigned int *aTemp);     // for loading profile variables
void loadFlash(void);
void writeFlash(void);

//////////////////////ISR's//////////////////////////////////

// SPI interrupt
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void) {

	while (!(IFG2 & UCB0TXIFG));      // USCI_A0 TX buffer ready?  clean up later??????

	maxValue = UCB0RXBUF;             // read the RX buffer this clears the RX interrupt

	if (cnt == 0 && maxStat == maxOn) {   // for keeping track of the byte order
		highByte = maxValue;
		cnt++;
	}
	else if (cnt == 1 && maxStat == maxOn) {
		lowByte = maxValue;
		cnt = 0;
	}
}

// Port1 button interrupt
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void) {

	if (P1IFG & button1) {     // is it button1?

			P1IFG &= ~button1;     // clear interrupt for button1

			buzz();
			delay(1500);
			stopBuzz();

		if (mode == scroll && pidFlag) { // pid menu
			if (click == 2) {
				PlantPID.pGain++;
			}
			else if (click == 3) {
				//PlantPID.iGain++;
				PlantPID.iGain += .001;
			}
			else if (click == 4) {
				PlantPID.dGain++;
			}
		}

        else if (mode == scroll) {

			//dLed1_toggle;
			LcdClearSome(0, 8, 7, 48);
			if ((nLines % 2) == 0) {
				click += 2;
			} else {
				click++;
			}

			if (click > maxClick) {
				click = sPos;
			}
		}
	}

	else if (P1IFG & button2) { // or is it button2?

		P1IFG &= ~button2;       // clear interrupt for button2

		buzz();
		delay(1600);
		stopBuzz();

		if (mode == scroll && pidFlag) {            // update pid menu
			if (click == 2) {
				if (PlantPID.pGain <= 0) {
					PlantPID.pGain = 0;
				} else {
					PlantPID.pGain--;
					startCapture();                        // is button being held down?
				}
			} else if (click == 3) {
				if (PlantPID.iGain <= 0) {
					PlantPID.iGain = 0;
				} else {
					//PlantPID.iGain--;
					PlantPID.iGain -= .001;
					startCapture();                        // is button being held down?
				}
			} else if (click == 4) {
				if (PlantPID.dGain <= 0) {
					PlantPID.dGain = 0;
				} else {
					PlantPID.dGain--;
					startCapture();                        // is button being held down?
				}
	        }
	    }

		else if (pwmFlag | reflowFlag){
		//else if (reflowFlag){

			mode = exit;                                 // to abort reflow
		}

	    else if (mode == scroll) {
	    	select = 1;
	    }
    }
}

// TIMER0 A0 interrupt
#pragma vector = TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void) {

	// set up for 2.75Hz interrupt
	sampCnt++;
	//dLed1_toggle;
}

/*
// TIMER0 A1 interrupt
#pragma vector = TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR(void) {

	switch (TA0IV) {

	case 2:    // CCR1 10hz interrupt
		//max_read = maxRead();
        sampCnt++;
		break;
	case 4:    // CCR2 not used
		break;
	case 10:   // overflow
		break;
	}
}
*/

// TIMER1 A1 interrupt
#pragma vector = TIMER1_A1_VECTOR
__interrupt void TIMER1_A1_ISR(void) {

	switch (TA1IV) {
	case 2:
		// CCR1 one second interrupt
		if (P1IN & button2) {   //check to see if button2
			TA1CTL = MC_0;      // stop timer
			TA1CCR0 = 0;        // stop timer
			dLed1_Hi;
			dLed2_Hi;
			mode = scroll;
			pidFlag = false;
			delay(5000);
			dLed2_Lo;           // make sure leds are off
			dLed1_Lo;
		}
		break;
	case 4:    // CCR2 PWM indicator
		break;
	case 10:   // overflow
	    break;
	}
}

#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void) { // use watchdog for 32ms button debounce

	//IFG1 &= ~WDTIFG; // clear interrupt flag.   this is automatically cleared by servicing interrupt
	//WDTCTL = WDTPW + WDTHOLD; // put WDT back in hold state
	count++;
	dLed2_toggle;
}

//////////////////////////////functions//////////////////////////
void initGPIO(void) {

	//// Port 1
	P1DIR |= max_CS;                // set max_CS to output
	P1DIR &= ~(button1 + button2);  // set p1.1 as an input
	P1OUT |= max_CS;                // max_CS is high
	P1REN |= (button1 + button2);   // Resistor Enable for button
	P1OUT &= ~(button1 + button2);  // set pull down on button
	P1IES &= ~(button1 + button2);  // Interrupt Edge Select - 0: trigger on rising edge, 1: trigger on falling edge
	P1IFG &= ~(button1 + button2);  // interrupt flag for button is off
	P1IE |= (button1 + button2);    // enable interrupt

	//// Port 2
	P2SEL &= ~(PIN_SCE + pwm_1); // turns on port 2 as gpio
	P2DIR |= PIN_SCE + pwm_1;    // set as outputs
	P2OUT |= PIN_SCE;            // set SCE high
	P2OUT &= ~pwm_1;             // pwm_1 low to make sure fet is off

	//// Port 3
	P3SEL &= ~(PIN_VCC + PIN_DC + PIN_RESET + PIN_BLIGHT + dLed1 + dLed2); // turns on port 3 as gpio
	P3DIR |= PIN_VCC + PIN_DC + PIN_RESET + PIN_BLIGHT + dLed1 + dLed2;    // set as outputs
	P3OUT &= ~(PIN_VCC + PIN_DC + PIN_RESET + PIN_BLIGHT + dLed1 + dLed2); // set these low
}

void startSecTimer(void) {

	WDTCTL = WDT_ADLY_1000;     // start watch dog timer for 1000ms/1 sec interrupt
}

void stopSecTimer(void) {

	WDTCTL = WDTPW + WDTHOLD;     // Stop WDT
	count = 0;
}

void startCapture(void) {

    TA1CCTL1 |= CCIE;                   // compare interrupt enabled
    TA1CCR0 = 32768 - 1;                // start timerA for 1 second interrupt
	TA1CTL |= TASSEL_1 + MC_1 + ID_0 ;  // ACLK, upmode, divide by 1
}

void buzz(void) {

	// Configure Port Pins
	P1DIR |= buzzer;          // p1.2 Output TA0.1
	P1SEL |= buzzer;          // TA0.1 Option select
	TA0CCR0 = 1000 - 1;       // Period Register start timer
	TA0CCR1 = 499;            // TA0.1 50% duty cycle
	TA0CTL = TASSEL_2 + MC_1; // SMCLK, up mode
	TA0CCTL1 |= OUTMOD_6;     // TA0.1 CCR1, Reset/Set
}

void stopBuzz(void) {

	TA0CTL = MC_0;    // stop timer
	TA0CCR0 = 0;      // load 0 into CCR0
}

void startSampler(void){

	P1DIR &= ~buzzer; // p1.2 Output TA0.1  turn this off so we dont hear a click
	P1SEL &= ~buzzer;                 // GPIO select
	TA0CTL |= TASSEL_1 + MC_1 + ID_0; // ACLK, upmode, divide by 1
	TA0CCR0 = 5957 - 1;               // start timerA for 2.75Hz interrupt
	//TA0CCR0 = 6553 - 1;               // start timerA for 2.5Hz interrupt
	//TA0CCR0 = 5462 - 1;               // start timerA for 3Hz interrupt
	//TA0CCR0 = 4097 - 1;               // start timerA for 4Hz interrupt
	//TA0CCR0 = 3251 - 1;               // start timerA for 5Hz interrupt
	//TA0CCR0 = 8001 - 1;               // start timerA for 2Hz interrupt
	TA0CCTL0 |= CCIE;                   // compare interrupt enabled
}

void stopSampler(void){

	TA0CTL = MC_0;    // stop timer
	TA0CCR0 = 0;      // load 0 into CCR0
	sampCnt = 0;
}

void startPwm(void) {

	// Configure Port Pins
	P2SEL |= pwm_1;                                 // TA1.2 Option select
	P2DIR |= pwm_1;                                 // p2.4 Output
	P3SEL |= dLed1;                                 // for visual feed back
	P3DIR |= dLed1;
	// load compare register for ~5hz square wave
	//TA1CCR0 = 3251 - 1;                             // Period Register  ~5hz
	//TA1CCR2 = 0;                                    // start with 0 duty cycle == SSR full on

	TA1CCR0 = 8001 - 1;                           // Period Register
	TA1CCR2 = 1;                                  // TA1.2 99% duty cycle  ~2hz

	//TA1CCR0 = 271 - 1;                            // Period Register  ~60hz
	//TA1CCR2 = 135;

	//TA1CTL = TASSEL_2 + MC_1 + ID_1 + TAIE;     // SMCLK, up mode, divide by 2 enable interrupt
	TA1CTL = TASSEL_1 + MC_3 + ID_0;              // ACLK, up-down mode, divide by 1
	TA1CCTL2 |= OUTMOD_6;                         // TA1.2 CCR1, Reset/Set compare
	pwmFlag = true;
}

void stopPwm(void) {

	TA1CTL = MC_0;     // stop timer
	TA1CCR0 = 0;       // load 0 into CCR0
	P2SEL &= ~pwm_1;   // put pin in GPIO mode to shut off relay
	P2DIR |= pwm_1;    // set to output
	P2OUT &= ~pwm_1;   // set low
	P3SEL &= ~dLed1;   // change back to GPIO for led indicator
	P3DIR |= dLed1;
	dLed1_Lo;

	pwmFlag = false;
}

unsigned int maxRead(void) {

	// read the max 6675
	maxStat = maxOn;
	cnt = 0;

	max_CS_Lo;                       // max_CS pin low

	UCB0TXBUF = 0x00;                // load shift register with dummy bite to send 2 times
	while (!(IFG2 & UCB0TXIFG));     // so we can read the RX buffer in the RX interrupt
	delay(1); //WTF??

	UCB0TXBUF = 0x00;
	while (!(IFG2 & UCB0TXIFG));
	delay(1); //Why?

	max_CS_Hi;                           // max_CS pin high

	result = (highByte << 8) + lowByte;  // shift bits into position
	result = (result >> 3) & 0xFFF;      // for 12 bit resolution

	maxStat = maxOff;

	return result * .23;                // multiply by a small fudge factor
}

// This itoa handles negative numbers
int itoa(signed int val, unsigned char *str) {

	int i = 0;

	if (val < 0) {
		str[0] = '-';
		return 1 + itoa(-val, str + 1);
	}

	if (val / 10) {
		i = itoa(val / 10, str);
	}
	str[i] = val % 10 + '0';
	str[++i] = '\0';

	return i; /* strlen(s), i.e. the next free slot in array */
}

void ftoa(float f, unsigned char *buf, unsigned int decPlaces) {

	unsigned int pos = 0, i, dp, num;

	if (f < 0) {
		buf[pos++] = '-';
		f = -f;
	}

	dp = 0;
	while (f >= 10.0) {
		f = f / 10.0;
		dp++;
	}

	unsigned int total = dp + decPlaces;

	for (i=0; i < total; i++) {
				num = f;
				f=f-num;
				if (num>9) {
					buf[pos++]='#';
				} else {
					buf[pos++]='0'+num;
				}
				if (dp==0) {
					buf[pos++]='.';
				}
				f=f*10.0;
				dp--;
			} // loop

	buf[pos++] = 0;  // null

	//return pos;
}

/*
// fahrenheit conversion
unsigned int itof(unsigned int i) { // convert celsius integer to fahrenheit

	return i * 9 / 5 + 32; // fahrenheit conversion
}
*/

void startScreen(void) {

	LcdClear();
	fontFlag = false;      // we want 5x7 font for this
	screenNum = sStart;
	mode = scroll;
	LcdGotoXY(4, 0);
	LcdString("!Reflow Time!");
	LcdGotoXY(8, 2);
	LcdString("Reflow");
	LcdGotoXY(8, 4);
	LcdString("Adjust PID");

	sPos = 2;
	click = sPos;
	nLines = 2;
	maxClick = 4;
}

void pidScreen(void) {

	LcdClear();
	screenNum = sPID;
	mode = scroll;
	LcdGotoXY(5, 0);
	LcdString("Adjust PID");
	LcdGotoXY(9, 2);
	LcdString("P = ");
	LcdGotoXY(9, 3);
	LcdString("I = ");
	LcdGotoXY(9, 4);
	LcdString("D = ");
	LcdGotoXY(9, 5);
	LcdString("exit");

	LcdGotoXY(38, 2);
	itoa(PlantPID.pGain, aCount);
	LcdString(aCount);
	LcdGotoXY(38, 3);
	ftoa(PlantPID.iGain, aCount, 4);
	//itoa(PlantPID.iGain, aCount);
	LcdString(aCount);
	LcdGotoXY(38, 4);
	itoa(PlantPID.dGain, aCount);
	LcdString(aCount);

	sPos = 2;
	click = sPos;
	nLines = 5;
	maxClick = 5;
}

void profileScreen(void) {

	LcdClear();
	screenNum = sProfile;
	mode = scroll;
	LcdGotoXY(12, 0);
	LcdString("Profile?");
	LcdGotoXY(9, 2);
	LcdString("RHOS");
	LcdGotoXY(9, 4);
	LcdString("LEAD");

	sPos = 2;
	click = sPos;
	nLines = 2;
	maxClick = 4;
}

void RoHSScreen(void) {

	mode = plot;

	LcdClear();
	LcdGotoXY(15, 1);
	LcdString("RoHS");
	LcdGotoXY(11, 2);
	LcdString("Profile");
	LcdGotoXY(15, 3);
	LcdString("Loaded");
	loadProfile(RohsTime, RohsLook);   // load the RoHS profile
	delay(20000);
	LcdClear();                        // clear now not in reflow screen
	LcdBmp(rhosPlot);                  // load the plot
	reflowFlag = true;                 // for exit
	fontFlag = true;                   // we want 3x5 font for this
	startSecTimer();                   // start the second timer
	startPwm();
	//relayOn;
	startSampler();
}

void LEADScreen(void) {

	mode = plot;

	LcdClear();
	LcdGotoXY(15, 1);
	LcdString("LEAD");
	LcdGotoXY(11, 2);
	LcdString("Profile");
	LcdGotoXY(15, 3);
	LcdString("Loaded!");
	loadProfile(leadTime, leadLook);    // load the lead profile
    delay(20000);
    LcdClear();                         // clear now not in reflow screen
    LcdBmp(leadPlot);                   // load the plot
    reflowFlag = true;                  // for exit
    fontFlag = true;                    // we want 3x5 font for this
    startSecTimer();                    // start the second timer
    startPwm();
    //relayOn;
    startSampler();
}

void reflowScreen() {

	if (count <= time[0]) {

		setPoint = profileLook[preHeat];

	} else if (count > time[0] && count <= time[1]) {

		setPoint = profileLook[rampUp];

	} else if (count > time[1] && count <= time[2]) {

		setPoint = profileLook[soak];

	} else if (count > time[2] && count < time[3]) {

		setPoint = profileLook[cool];
		stopPwm();                            // turn off PWM SSR is off
		//relayOff;
		dLed1_Lo;

	} else if (count >= time[3]) {            // cycle is finished

        mode = start;
		fontFlag = false;
		stopPwm();
		stopSampler();
		//relayOff;
		stopSecTimer();
		reflowFlag = false;
		dLed1_Lo;
		dLed2_Lo;
		setPoint = profileLook[done];
		LcdClear();
		LcdGotoXY(20, 1);
		LcdString("Cycle");
		LcdGotoXY(5, 3);
		LcdString("Completed!");
		buzz();
		delay(30000);
		stopBuzz();
		count = 0;
		sampCnt = 0;
		return;
	}

	LcdGotoXY(0, 5);
	LcdString("setTemp:");    // display temp we are shooting for
	itoa(setPoint, aCount);
	LcdString(aCount);
	LcdString("  ");
	LcdString("temp:");       // display current temp
	itoa(max_read, tempInC);
	LcdString(tempInC);
	LcdString("  ");
	LcdGotoXY(0, 0);
	LcdString("sec:");        // display second count
	itoa(count, aCount);
	LcdString(aCount);
	LcdString("  ");
	//LcdGotoXY(65, 0);
	//ftoa(PlantPID.iState, aCount, 3);      // display Integrator output
	//LcdString(aCount);
	//LcdString("  ");
	LcdGotoXY(48, 0);
	//itoa(drive, aCount);     // display PID output
	ftoa(drive, aCount, 3);
	LcdString(aCount);
	LcdString("   ");
	LcdGotoXY(58, 1);
	itoa(duty, aCount);      // display duty cycle of PWM
	LcdString(aCount);
	LcdString("  ");

	plotScreen(count, max_read);

}

void exitScreen(void) {

	LcdClear();
	mode = start;
	reflowFlag = false;
	fontFlag = false;
	LcdClear();
	LcdGotoXY(15, 1);
	LcdString("REFLOW");
	LcdGotoXY(12, 3);
	LcdString("ABORTED!");

	buzz();
	stopPwm();
	stopSampler();
	//relayOff;
	dLed1_Lo;
	dLed2_Lo;
	stopSecTimer();
	delay(10000);
	stopBuzz();
	count = 0;
	sampCnt = 0;
}

void Scroll(char curPos) {

	if (select == 0) {

		LcdGotoXY(0, curPos);
		LcdString(">");
	}

	else if (select == 1 && screenNum == sPID) {

		select = 0;
		pidFlag = true;     // set to true so we stay on PID menu

		if (click == 5) {  // we are on the exit line so go back to start screen
			mode = start;
			pidFlag = false;
			writeFlash();    // write the values to the info mem for next time
		}
	}

	else if (select == 1) {

		mode = modeLookUp[(click * screenNum)];
		select = 0;
	}
}

void relayDrive(float Drive) {

	if (Drive != lastDrive) {

		if (Drive <= 0) {                // PWM needs to be constrained between
			Drive = 0;                   // these values or else it goes full on
		} else if (Drive >= 4000) {      // when the Drive goes negative
			Drive = 4000;                // in other words TimerA pukes!
		}                                // 8000 for TA1CCR2 == TA1CCR0

		// when Drive is at zero we have reached the set point
		// we need to map to the inverse of the Drive output
		duty = map(Drive, 0, 4000, 4000, 0);     // 8000 = 0% duty cycle
		//duty = Drive;
		if (Drive <= 0) {                        // when Drive goes negative we get a big number here
			duty = 8000;                         // set the duty cycle to 0
		}else if (Drive >= 4000) {               // test for both situations
			duty = 8000;
		}

		TA1CCR2 = duty;
	}

	lastDrive = Drive;
}

void loadProfile(const unsigned int *aTime, const unsigned int *aTemp) {

	unsigned int i;

	for (i = 0; i <= 4; i++) {     // load the time look up table

		time[i] = aTime[i];

	}
	for (i = 0; i <= 5; i++) {     // load the temp look up table

		profileLook[i] = aTemp[i];

	}
}

void loadFlash(void) {

	PlantPID.pGain = *Flash_pGain;     // tuned values loaded from flash
	PlantPID.iGain = *Flash_iGain;
	PlantPID.dGain = *Flash_dGain;
}

void writeFlash(void) {

	_disable_interrupts();
	FCTL1 = FWKEY + ERASE;            // Set Erase bit
	FCTL3 = FWKEY;                    // Clear Lock bit

	*Flash_pGain = 0;                 // Dummy write to erase Flash segment
	*Flash_iGain = 0;
	*Flash_dGain = 0;

	FCTL1 = FWKEY + WRT;              // Set WRT bit for write operation

	*Flash_pGain = PlantPID.pGain;    // Write value to flash
	*Flash_iGain = PlantPID.iGain;
	*Flash_dGain = PlantPID.dGain;

	FCTL1 = FWKEY;                    // Clear WRT bit
	FCTL3 = FWKEY + LOCK;             // Set LOCK bit
	_enable_interrupts();
}

///////////////////////////////// Main /////////////////////////////////
int main(void) {


	WDTCTL = WDTPW + WDTHOLD;     // Stop WDT

	BCSCTL1 = CALBC1_16MHZ;       // calibrate basic clock system control 1 to 16mhz
	DCOCTL = CALDCO_16MHZ;        // calibrate DCO to 16mhz

	BCSCTL2 |= DIVS_2;            // SMCLK = 16/4 = 4Mhz for SPI
	BCSCTL3 |= LFXT1S_0 | XCAP_3; // Use 32 khz crystal as ACLK + 12.5 pF caps

	IE1 |= WDTIE;                 //enable watchdog timer interrupt

	//// Flash timing////
	FCTL2 = FWKEY + FSSEL_1 + FN5 + FN3;    // MCLK = 16Mhz/32+8 = 400khz // has to be between 257-476 khz these both work
	//FCTL2 = FWKEY + FSSEL_1 + FN3 + FN2;    // SMCLK= 4Mhz/8+4 = 333khz

	initGPIO();
	initSPI8();     // set up 8 bit SPI for nokia LCD and max6675

	//_bis_SR_register(LPM0_bits + GIE);      // Enter LPM0 w/ interrupt
	_enable_interrupts();

	LcdInit();                  // set up nokia 5110 lcd

    loadFlash();                // load PID values from before power down

	//PlantPID.pGain = 10;      // start values
	//PlantPID.iGain = .001;
	//PlantPID.dGain = 50;

    max_read = maxRead();         // take a temp reading
	//WDTCTL = WDT_ADLY_1000;     // start watch dog timer for 1000ms/1 sec interrupt

	dLed1_Hi;
	dLed2_Hi;
	delay(10000);
	dLed2_Lo;
	dLed1_Lo;

	for (;;) {     // infinite loop


		if (sampCnt != 0) {            // read the max 6675 @ 2.75hz

			max_read = maxRead();

			// update the PID
			drive = UpdatePID(&PlantPID, setPoint - max_read, max_read);
			relayDrive(drive);
			sampCnt = 0;
		}

		// for updating PID screen values
		if (mode == scroll && screenNum == sPID) {

			if (click == 2) {
				LcdGotoXY(38, 2);
				itoa(PlantPID.pGain, aCount);
				LcdString(aCount);
				LcdString("   ");            // clear out trailing digits if any
			}

			else if (click == 3) {
				LcdGotoXY(38, 3);
				//itoa(PlantPID.iGain, aCount);
				ftoa(PlantPID.iGain, aCount, 4);
				LcdString(aCount);
				LcdString("   ");
			}

			else if (click == 4) {
				LcdGotoXY(38, 4);
				itoa(PlantPID.dGain, aCount);
				LcdString(aCount);
				LcdString("   ");
			}
		}

		switch (mode) {

		case scroll:
			Scroll(click);
			break;
		case start:
			startScreen();
			break;
		case reflow:
			profileScreen();
			break;
		case pid:
			pidScreen();
			break;
		case RoHS:
			RoHSScreen();
			break;
		case LEAD:
			LEADScreen();
			break;
		case plot:
			reflowScreen();
			break;
		case exit:
			exitScreen();
			break;
		default:
			Scroll(click);
			break;

		}

	}

}
