/*
 * nokia5110.h
 *
 */

#ifndef NOKIA5110_H_
#define NOKIA5110_H_

#include <msp430G2553.h>

// These are for the DIY booster pack I made
#define PIN_DC        BIT5    // Port 3.5
#define PIN_RESET     BIT6    // Port 3.6
#define PIN_SCE       BIT0    // Port 2.0
#define PIN_VCC       BIT7    // Port 3.7
#define PIN_BLIGHT    BIT0    // Port 3.0

// lcd
#define LCD_X     84    //columns
#define LCD_Y     48   //rows
#define LCDROWMAX	6
#define LCDCOLMAX	84
#define LCDPIXELROWMAX	48

//
#define CMD  0     //for command data
#define DATA  1   //for character data

//// LCD Function Prototypes ///////
void LcdWrite(char cmd,char data);
void LcdInit();
void LcdClear();
void LcdClearSome(unsigned char xs, unsigned char ys, unsigned char xe, unsigned char ye);
void LcdString(unsigned char *characters);
void LcdCharacter(unsigned  char character);
void LcdGotoXY(unsigned char x, unsigned char y);
void LcdBmp(const unsigned char my_array[]);
//void printV(unsigned char x, unsigned char y, unsigned char length, char *characters);
void setPixel(unsigned char x, unsigned char y);
void drawFilledRectangle(unsigned char x1, unsigned char y1,
			unsigned char x2, unsigned char y2);
void drawLine(unsigned char x1, unsigned char y1, unsigned char x2,
		unsigned char y2);
//unsigned int map(unsigned int x, unsigned int in_min, unsigned int in_max, unsigned int out_min, unsigned int out_max);
float map(float x, float in_min, float in_max, float out_min, float out_max);
void plotScreen(unsigned int x, unsigned int y);
//void loadBuf(unsigned char x, unsigned char y);
//void writeBuf(char *buf);

#endif /* NOKIA5110_H_ */
