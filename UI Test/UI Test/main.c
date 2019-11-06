/*
 * Lab7.c
 *
 * Created: 4/8/2019 10:29:57 AM
 * Author : dtocci1
 */ 
#define F_CPU 16000000
#include <avr/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "lcd_functions.h"
#include "floatConvert.h"

#define HEIGHT_SELECT "[HEIGHT]  ANGLE "
#define ANGLE_SELECT " HEIGHT  [ANGLE]"
#define NO_HS_SELECT " HEIGHT   ANGLE "
#define DEFAULT_ANGLE " 00.0     00.0"
#define MAX_HEIGHT 100
#define MAX_ANGLE 90
#define _BV(n) (1 << n)

typedef enum __attribute__ ((__packed__)) {HEIGHT, ANGLE, CHEIGHT, CANGLE} State;
volatile int tmpCLK=0;
volatile int tmpDT=0;
volatile int tmpSW=0;
volatile int state = HEIGHT; // starts in height selection by default
volatile float height = 10.3;
volatile float angle = 00.0;
volatile int valueConfirm = 0;

int main(void)
{
	// arbitrary ports right now
	DDRC &= ~(_BV(2) | _BV(1) | _BV(0));
	PCICR |= (1<<PCIE1); // pin change interrupt 1
	PCMSK1 |= (1<<PCINT8) | (1<<PCINT9) | (1<<PCINT10);
	
	
	char heightConv[16] = "";
	char angleConv[16] = "";
	char test[4] = "";

	lcd_init();
	angleConv[0]='0';
	angleConv[1]='0';
	angleConv[2]='.';
	angleConv[3]='0';
	heightConv[0]='0';
	heightConv[1]='0';
	heightConv[2]='.';
	heightConv[3]='0';
	
	// PRINT DEFAULT STATE
	lcd_gotoxy(1,1);
	lcd_print(HEIGHT_SELECT);
	lcd_gotoxy(1,2);
	lcd_print(DEFAULT_ANGLE);
	
	sei();
	// ***** MAIN LOOP *****//
	while(1)
	{
		//This is where the PID loop will be located
		//After the interrupt the PID will account for change
		//Must check to make sure value was confirmed before accounting for change in PID loop
		//This is why I have the valueConfirm variable
	}
	
	
}

// Rotary encoder interrupt based on pin change:
// -Adjusts LCD Display (what is selected)
// -Changes height / angle
//		- 0 <= angle <= 90
//		- 0 <= height <= ???
//		- maybe add feature to indicate height / angle reached max value on LCD
//	-Button press either:
//		- Confirms height / angle adjustment
//		- Selects angle / height and moves user to adjust height / angle

ISR(PCINT1_vect) 
{
	tmpCLK=PINC & (1<<2);
	tmpDT=PINC&(1<<1);
	tmpSW=PINC&(1<<0);
	if(tmpCLK == 0)
	{
		if (tmpDT == 0) // right turn	
		{
			switch(state)
			{
				case HEIGHT: // change to angle state
					lcd_print(ANGLE_SELECT);
					state = ANGLE;
					break;
				case ANGLE: // do nothing OR change to height state?
					// do nothing as of rn fuck it
					break;
				case CHEIGHT: // increment height value (as long as < MAX (?))
					if (height < MAX_HEIGHT) // total guess right now
						height=height + 0.1; //increment height by tenth
					break;	
				case CANGLE: // increment angle value (as long as <= MAX (90))
					if (angle < MAX_ANGLE)
						angle=angle + 0.1;
					break;
			}
		}
		else // left turn
		{
			switch(state)
			{
				case HEIGHT: // do nothing OR change to angle state?
					// again we aren't doing anything (rn) for this so fuck it
					break;
				case ANGLE: // change to height state
					lcd_print(HEIGHT_SELECT);
					state = HEIGHT;
					break;
				case CHEIGHT: // decrement height value (as long as >= MIN (0) )
					if (height > 0)
						height = height - 0.1;
					break;
				case CANGLE: // increment angle value (as long as >= MIN (0))
					if(angle > 0)
						angle = angle - 1;
					break;
			}
		}
	}
	else if(tmpSW == 0) // button press
	{
		switch(state)
		{
			case HEIGHT: // change to height change state
				state = CHEIGHT;
				break;
			case ANGLE: // change to angle change state
				state = CANGLE;
				break;
			case CHEIGHT: // confirm height change value
				valueConfirm = 1; // set flag
				state = HEIGHT; // exit from change function
				break;
			case CANGLE: // confirm angle change value
				valueConfirm = 1; // set flag - might need to make unique flag
				state = ANGLE;
				break;
		}
	}

}






/************************************************* NUMBER CHANING FUNCTIONS **********************************************************************************/

void print_height_angle(char *angleConv, char *heightConv, int heightSelect) // possible way to fix based on state 0=height 1=angle
{
	lcd_gotoxy(1,1);
	if (heightSelect == 1)
		lcd_print(HEIGHT_SELECT);
	else 
		lcd_print(ANGLE_SELECT);
	// un-select bottom row
	lcd_gotoxy(1,2);
	lcd_print(" ");
	lcd_print(heightConv);
	lcd_print(" ");
	lcd_gotoxy(10,2);
	lcd_print(" ");
	lcd_print(angleConv);
	lcd_print(" ");
}



void print_height_change(char *conversion)
{
	//ensure height becomes unselected
	lcd_gotoxy(1,1);
	lcd_print(NO_HS_SELECT);
	lcd_gotoxy(1,2);
	lcd_print("[");
	lcd_gotoxy(2,2);
	lcd_print(conversion);
	lcd_gotoxy(6,2);
	lcd_print("]");
}

void print_angle_change(char *conversion)
{
	lcd_gotoxy(1,1);
	lcd_print(NO_HS_SELECT);
	lcd_gotoxy(10,2);
	lcd_print("[");
	lcd_gotoxy(11,2);
	lcd_print(conversion);
	lcd_gotoxy(15,2);
	lcd_print("]");
	
}

