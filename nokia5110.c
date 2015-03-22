/*
 *      nokia5110.c
 *
 *      Functions for Nokia 5110 LCD
 */

#include <stdbool.h>
#include "nokia5110.h"
#include "font5x7.h"
#include "font3x5.h"
#include "delay.h"

// This switches between 3x5 font and 5x7 font
extern bool fontFlag;
// for lcdClearSome function and plotScreen function
enum pixMode { off, on, xor};
enum pixMode pMode = off;

unsigned char pixMode = 0;

//static unsigned char lcdBuf[LCDROWMAX][LCDCOLMAX] = { { 0 }, { 0 } };  // need more ram!!!!

void LcdGotoXY(unsigned char x, unsigned char y)
{
  LcdWrite(CMD ,(0x40 | (y & 0x07)));    // Y row == 0100 0yyy
  LcdWrite(CMD ,(0x80 | (x & 0x7F)));    // X column == 1xxx xxxx
}

void LcdBmp(const unsigned char my_array[])
{
  unsigned short index = 0;
  //LcdWrite(CMD , 0x22);     // vertical addressing
  for (index = 0; index < 504; index++)
  {
   LcdWrite(DATA,my_array[index]);
  }
  //LcdWrite(CMD , 0x20);     // horizontal addressing
}

void LcdCharacter(unsigned char character) {

	if (fontFlag) {

		unsigned int temp = 0;
		temp = font3x5[character - 0x20];

		LcdWrite(DATA, (temp & 0x1F));
		LcdWrite(DATA, (temp >> 5) & 0x1F);
		LcdWrite(DATA, (temp >> 10));

	} else {

		char index = 0;
		for (index = 0; index < 5; index++) {
			LcdWrite(DATA, font5x7[character - 0x20][index]);
		}
	}
	LcdWrite(DATA, 0x00);
}

void LcdString(unsigned char *characters)
{
  while (*characters){
   LcdCharacter(*characters++);
 }
}

void LcdClear(void) {
  int i,j;

  LcdWrite(CMD, 0x80);
  LcdWrite(CMD, 0x40);

  for (i=0;i<6;i++)      // number of rows
  for (j=0;j<LCD_X;j++)  // number of columns
  LcdWrite(DATA, 0x00);
}

/*
 * Name         : clearSome
 * Description  : Clear part of the screen
 * Argument(s)  : xs, ys are x any y start points for top left corner of rectangle,
 * xe, ye are end points for lower left corner of rectangle
 * Return value : none
 */
void LcdClearSome(unsigned char xs, unsigned char ys, unsigned char xe, unsigned char ye) {

	pMode = off;
	drawFilledRectangle(xs,ys,xe,ye);
	pMode = on;
}

void LcdInit(void)
{
  P3OUT |= PIN_BLIGHT;
  P3OUT |= PIN_VCC;         // power to LCD
  P3OUT |= PIN_RESET;       // set RESET high
  P3OUT &= ~PIN_RESET;      // set RESET low
  delay(35);
  P3OUT |= PIN_RESET;       // set RESET high
  P2OUT |= PIN_SCE;         // SCE pin high
  LcdWrite(CMD , 0x21);     // LCD Extended instruction set
  LcdWrite(CMD , 0x9F);     // Set LCD Vop (Contrast). //0xE0 - BF  may have to play with
  LcdWrite(CMD , 0x07);     // Set Temp coefficent. //0x04 =t0 //0x05=t1 // 0x06=t2 // 0x07=t3
  LcdWrite(CMD , 0x13);     // LCD bias mode 1:100 0x10 //1:48 0x13
  LcdWrite(CMD , 0x20);     // LCD basic instruction set
  LcdWrite(CMD , 0x08);     // lcd blank
  LcdWrite(CMD , 0x0C);     // LCD  0x0C for black on white //0x0d for inverse
  LcdClear();
}

void LcdWrite(char cmd, char data) {

   P2OUT &= ~PIN_SCE;        // SCE pin low
   (cmd == CMD) ? (P3OUT &= ~PIN_DC) : (P3OUT |= PIN_DC); // check to see if we are writing a CMD or DATA
   UCB0TXBUF = data;         // load shift register with data to send
   while (!(IFG2 & UCB0TXIFG));
   delay(1);                 // Having this here is lame!!!!
   P2OUT |= PIN_SCE;         // SCE pin high
}

/*
 * Name         : printV
 * Description  : prints a word vertically
 * Argument(s)  : x , y , length of word (can't be longer than 6 letters)
 * Return value : None
 */

/*
void printV(unsigned char x, unsigned char y, unsigned char length,
		char *characters) {

	for (length = 0; length <= 5; length++) {
		LcdGotoXY(x, y);
		y++;
		LcdWrite(DATA, *characters++);
	}
}
*/

/*
 * Name         : setPixel
 * Description  : Set a single pixel either on or off
 * Argument(s)  : x,y - position, x = 0-83, y = 0-6
 */
void setPixel(unsigned char x, unsigned char y) {
	unsigned char value = 0;
	unsigned char row = 0;

	//if (x < 0 || x >= LCDCOLMAX || y < 0 || y >= LCDPIXELROWMAX)
		//return;

	row = y / 8;

	if (pMode == off) {
		value &= ~(1 << (y % 8));
	} else if (pMode == xor) {      // need a buffer for this
		value ^= (1 << (y % 8));
	} else {
		value |= (1 << (y % 8));
	}

	LcdGotoXY(x, row);
	LcdWrite(DATA, value);
}

/*
 * Name         : drawLine
 * Description  : Draws a line between two points on the display.
 * Argument(s)  : x1, y1 - Absolute pixel coordinates for line origin.
 *                x2, y2 - Absolute pixel coordinates for line end.
 *
 * Return value : none
 */
void drawLine(unsigned char x1, unsigned char y1, unsigned char x2,
		unsigned char y2) {
	int dx, dy, stepx, stepy, fraction;

	/* Calculate differential form */
	/* dy   y2 - y1 */
	/* -- = ------- */
	/* dx   x2 - x1 */

	/* Take differences */
	dy = y2 - y1;
	dx = x2 - x1;

	/* dy is negative */
	if (dy < 0) {
		dy = -dy;
		stepy = -1;
	} else {
		stepy = 1;
	}

	/* dx is negative */
	if (dx < 0) {
		dx = -dx;
		stepx = -1;
	} else {
		stepx = 1;
	}

	dx <<= 1;
	dy <<= 1;

	/* Draw initial position */
	setPixel(x1, y1);

	/* Draw next positions until end */
	if (dx > dy) {
		/* Take fraction */
		fraction = dy - (dx >> 1);
		while (x1 != x2) {
			if (fraction >= 0) {
				y1 += stepy;
				fraction -= dx;
			}
			x1 += stepx;
			fraction += dy;

			/* Draw calculated point */
			setPixel(x1, y1);
		}
	} else {
		/* Take fraction */
		fraction = dx - (dy >> 1);
		while (y1 != y2) {
			if (fraction >= 0) {
				x1 += stepx;
				fraction -= dy;
			}
			y1 += stepy;
			fraction += dx;

			/* Draw calculated point */
			setPixel(x1, y1);
		}
	}
}

/*
 * Name         : drawFilledRectangle
 * Description  : Draw a filled rectangle given the top left and bottom right points
 * 		  just simply draws horizontal lines where the rectangle would be
 * Argument(s)  : x1, y1 - Absolute pixel coordinates for top left corner
 *                x2, y2 - Absolute pixel coordinates for bottom right corner
 *
 * Return value : none
 */
void drawFilledRectangle(unsigned char x1, unsigned char y1, unsigned char x2,
		unsigned char y2) {
	unsigned char i;
	for (i = y1; i <= y2; i++) {
		drawLine(x1, i, x2, i);
	}
}


// mapping function taken from arduino
unsigned int map(unsigned int x, unsigned int in_min, unsigned int in_max, unsigned int out_min, unsigned int out_max){

  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/*
// mapping function taken from arduino
float map(float x, float in_min, float in_max, float out_min, float out_max){

  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
*/
/*
// for ploting the screen !!!!!Not enough RAM!!!!!
void loadBuf(unsigned char x, unsigned char y) {

	unsigned char value = 0;
	unsigned char row = 0;

	row = y / 8;

	value = lcdBuf[row][x];

	value |= (1 << (y % 8));

	lcdBuf[row][x] = value;
}

// for writing the screen buffer to the screen
void writeBuf(unsigned char **buf){

	unsigned int i, j;

	for (i = 0; i <= LCDROWMAX-1; i++){
		for (j = 0; j <= LCDCOLMAX; j++){

			LcdWrite(DATA, buf[i][j]);
		}
	}

}
*/

//for real time plot x = time, y = degrees
void plotScreen(unsigned int x, unsigned int y) {

	unsigned int secCount = 0;
	unsigned int degrees = 0;

	secCount = map(x,0,500,0,LCD_X);        // hard coded time range 0-504, map to nokia x
	degrees = map(y,0,300,0,LCD_Y - 8);     // temp range 0 - 280'C , map to nokia y - 1 row

	degrees = (LCD_Y - 8) - degrees;        // flip the screen

	//pMode = xor;                           // doesnt work without a buffer
	setPixel(secCount,degrees);
	//pMode = on;
	//loadBuf(secCount,degrees);
	//writeBuf(lcdBuf);
}

