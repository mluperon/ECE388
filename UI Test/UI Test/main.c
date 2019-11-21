/*
 * Lab7.c
 *
 * Created: 4/8/2019 10:29:57 AM
 * Author : dtocci1
 *
 *	TO DO: Finish pin change routine
 *		   Implement all peripherals and ports
 */ 

#define F_CPU 16000000
#include <avr/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "lcd_functions.h" // Dr. Viall's code for using the LCD
#include "floatConvert.h" // Code to convert a float into a character array

#define HEIGHT_SELECT "[HEIGHT]  ANGLE "
#define ANGLE_SELECT " HEIGHT  [ANGLE]"
#define NO_HS_SELECT " HEIGHT   ANGLE "
#define DEFAULT_ANGLE " 00.0     00.0"
#define MAX_HEIGHT 15 // max height value 15ft
#define MAX_ANGLE 90 // max angle value 90 degrees
#define RIGHT 123 // PINC value for turning right
#define LEFT 125 // PINC value for turning left
#define BUTTON 111 // PINC value for pushing button
#define _BV(n) (1 << n)

typedef enum __attribute__ ((__packed__)) {HEIGHT, ANGLE, CHEIGHT, CANGLE} State; // used to navigate display
	//HEIGHT =  [HEIGHT] ANGLE
	//		   	 0.00   0.00			// Height is selected, turning knob will change it to angle selection (ANGLE), clicking button will change it to change height (CHEIGHT)
	//
	//ANGLE =  HEIGHT [ANGLE]			// Angle is selected, turning knob will change it to height selection (HEIGHT), clicking button will change it to change angle (CANGLE)
	//		    0.00   0.00
	//
	//CHEIGHT = HEIGHT ANGLE			// Height value is selected, turning knob will increase or decrease height by 0.1 as long as within bounds of [0,15]
	//		   [0.00]  0.00				// Clicking button will confirm value of height setting valueConfirm flag = 1 and returning user to height selection (HEIGHT)
	//
	//CHEIGHT = HEIGHT ANGLE			// Angle value is selected, turning knob will increase or decrease angle by 0.1 as long as within bounds of [0,90]
	//		    0.00  [0.00]			// Clicking button will confirm value of angle setting valueConfirm flag = 1 and returning user to angle selection (ANGLE)
	//
	
volatile int state = HEIGHT; // starts in height selection by default
volatile double height = 0.00; // global variable for height
volatile double angle = 00.0; // global variable for angle
volatile int valueConfirm = 0; // flag signifying a new value for EITHER height or angle has been made. Will be used in PID loop to signal when to adjust fan speeds
volatile char heightConv[16] = ""; // global character array for storing height to be output to LCD
volatile char angleConv[16] = ""; // global character array for storing angle to be output to LCD

int main(void)
{
	int dummy=0;
	// arbitrary ports right now
	DDRC &= ~(_BV(2) | _BV(1) | _BV(4)); // sets PORTC 1, 2, and 4 to input (input from rotary encoder)
										 // 1 = DT signal
										 // 2 = CLK signal
										 // 4 = SW signal (button press)
										 
	PCICR |= (1<<PCIE1); // Enable pin change interrupt on PORTC
	PCMSK1 |= (1<<PCINT12) | (1<<PCINT9) | (1<<PCINT10); // Enable Pins 1,2, and 4 to trigger this interrupt
	
	// These variables are Char arrays as the LCD cannot output int/float values, it must be sent as a string
	
	lcd_init(); // initialize the LCD according to Dr. Viall's 263 code
	
	angleConv[0]='0'; // v Set up height and angle to start at 00.0 value
	angleConv[1]='0';
	angleConv[2]='.';
	angleConv[3]='0';
	heightConv[0]='0';
	heightConv[1]='0';
	heightConv[2]='.';
	heightConv[3]='0';// ^
	
	// PRINT DEFAULT STATE
	lcd_gotoxy(1,1); // go to row 1 column 1 of LCD
	lcd_print(HEIGHT_SELECT); // Print ->   [HEIGHT] ANGLE
	lcd_gotoxy(1,2); // go to row 2 column 2 of LCD
	lcd_print(DEFAULT_ANGLE); // Print -> 00.0 00.0
	
	sei(); // enable global interrupts
	// ***** MAIN LOOP *****//
	while(1)
	{
		dummy=0;
		dummy=0;	
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

int tmp=0;
ISR(PCINT1_vect) 
{
	_delay_ms(5);
	tmp=PINC;
	if(PINC == RIGHT) //if right turn triggered interrupt
	{
		switch(state)
		{
			case HEIGHT: // change to angle state
				print_height_angle(angleConv,heightConv,0);
				state = ANGLE;
				break;
			case ANGLE: // do nothing OR change to height state?
				// do nothing as of rn fuck it
				break;
			case CHEIGHT: // increment height value (as long as < MAX (?))
				if (height < MAX_HEIGHT) // total guess right now
				{
					height=height + 0.1; //increment height by tenth
					ftoa(height,heightConv); // convert height to char array (heightConv) with 1 decimal place
					print_height_change(heightConv); // print conversion to LCD
				}
				break;	
			case CANGLE: // increment angle value (as long as <= MAX (90))
				if (angle < MAX_ANGLE)
				{
					angle=angle + 0.1;
					ftoa(angle,angleConv); // convert angle to char array (angleConv) with 1 decimal place
					print_angle_change(angleConv);	// print conversion to LCD
				}
				break;
		}
	}
	
	
	if (PINC == LEFT) // If left turn triggered interrupt
	{
		switch(state)
		{
			case HEIGHT: // do nothing OR change to angle state?
				// again we aren't doing anything (rn) for this so fuck it
				break;
			case ANGLE: // change to height state
				print_height_angle(angleConv,heightConv, 1);
				state = HEIGHT;
				break;
			case CHEIGHT: // decrement height value (as long as >= MIN (0) )
				if (height > 0)
				{
					height = height - 0.1;
					ftoa(height,heightConv); // convert height to char array (heightConv) with 1 decimal place
					print_height_change(heightConv); // print conversion to LCD
				}
				break;
			case CANGLE: // increment angle value (as long as >= MIN (0))
				if(angle > 0)
				{
					angle = angle - 0.1;
					ftoa(angle,angleConv); // convert angle to char array (angleConv) with 1 decimal place
					print_angle_change(angleConv);	// print conversion to LCD
				}
				break;
		}
	}


	if(PINC == BUTTON) // If button press triggered interrupt
	{
		switch(state)
		{
			case HEIGHT: // change to height change state
				ftoa(height,heightConv);
				print_height_change(heightConv);
				state = CHEIGHT;
				break;
			case ANGLE: // change to angle change state
				ftoa(angle,angleConv);
				print_angle_change(angleConv);
				state = CANGLE;
				break;
			case CHEIGHT: // confirm height change value
				valueConfirm = 1; // set flag
				print_height_angle(angleConv,heightConv, 1); // refresh screen with height selected 
				state = HEIGHT; // exit from change function
				break;
			case CANGLE: // confirm angle change value
				valueConfirm = 1; // set flag - might need to make unique flag
				print_height_angle(angleConv,heightConv, 0); // refresh screen with angle selected
				state = ANGLE;
				break;
		}
	}
	
	while(PINC != 127)
	{
		
	}

}






/************************************************* NUMBER CHANING FUNCTIONS **********************************************************************************/

// Function used to print either the height or angle selection ** THIS FUNCTION IS ONLY FOR CHANGING THE SELECTION **
void print_height_angle(char *angleConv, char *heightConv, int heightSelect)
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
	lcd_gotoxy(6,2);
	lcd_print(" ");
	lcd_gotoxy(10,2);
	lcd_print(" ");
	lcd_print(angleConv);
	lcd_gotoxy(15,2);
	lcd_print(" ");
}

// Function used to print a change in the height value
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

// Function used to print a change in the angle value
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

