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
#include <avr/delay.h>
#include "lcd_functions.h" // Dr. Viall's code for using the LCD
#include "floatConvert.h" // Code to convert a float into a character array

#define HEIGHT_SELECT "[HEIGHT]  ANGLE "
#define ANGLE_SELECT " HEIGHT  [ANGLE]"
#define NO_HS_SELECT " HEIGHT   ANGLE "
#define DEFAULT_ANGLE " 00.0     00.0"
#define MAX_HEIGHT 15 // max height value 15ft
#define MAX_ANGLE 90 // max angle value 90 degrees
#define RIGHT 122 // PINC value for turning right
#define LEFT 124 // PINC value for turning left
#define BUTTON 118 // PINC value for pushing button
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
volatile float height = 0.00; // global variable for height
volatile float angle = 00.0; // global variable for angle
volatile int valueConfirm = 0; // flag signifying a new value for EITHER height or angle has been made. Will be used in PID loop to signal when to adjust fan speeds
volatile char heightConv[16] = ""; // global character array for storing height to be output to LCD
volatile char angleConv[16] = ""; // global character array for storing angle to be output to LCD
volatile int overflowCount = 0; // global variable used for tracking how many times the timer overflows (each overflow is equivalent to one second)

int main(void)
{
	//********************************************** PIN CHANGE INTERRUPT SETUP **********************************************//
	DDRC &= ~(_BV(2) | _BV(1) | _BV(4)); // sets PORTC 1, 2, and 4 to input (input from rotary encoder)
										 // 1 = DT signal
										 // 2 = CLK signal
										 // 4 = SW signal (button press)
										 
	PCICR |= (1<<PCIE1); // Enable pin change interrupt on PORTC
	PCMSK1 |= (1<<PCINT11) | (1<<PCINT9) | (1<<PCINT10); // Enable Pins 1,2, and 4 to trigger this interrupt
	
	
	//********************************************** PWM SETUP **********************************************//
	// Set up PWM on PortB(1)
	DDRB |= (1<<1); // set PINB 1 to output
	PORTB &= ~(1<<1); // Ensure PINB 1 is not outputting voltage
	TCCR1A= (0b11 << COM1A0) | ( 0b00 << COM1B0) | (0b10 << WGM10); // set up PWM with prescalar
	TCCR1B= (0b11 << WGM12) | (0b010<< CS10);
	ICR1= 40000-1; // (20MS /8 PRESCALAR)
	OCR1A=36000-1; // 1000->4000 0.5ms to 2ms *** adjust ***
	
	// Set up 5 second timer for startup
	TCCR3A = 0; //(0b00 << COM3A0) | (0b00 << COM3B0)
	TCCR3B = (0 << ICNC3) | (0 << ICES3) | (0b00 << WGM32) | (0b101 << CS30);
	TIMSK3 = (1 << TOIE3);
	TCNT3 = -15625; // One second timer value
	
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
	
	// ***** SPEED CONTROLLER STARTUP ***** //
	
	
	
	// ***** MAIN LOOP ***** //
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

	if(PINC == RIGHT) //if right turn triggered interrupt
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
				{
					height=height + 0.1; //increment height by tenth
					ftoa(height,heightConv,1); // convert height to char array (heightConv) with 1 decimal place
					print_height_change(heightConv); // print conversion to LCD
				}
				break;	
			case CANGLE: // increment angle value (as long as <= MAX (90))
				if (angle < MAX_ANGLE)
				{
					angle=angle + 0.1;
					ftoa(angle,angleConv,1); // convert angle to char array (angleConv) with 1 decimal place
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
				print_height_angle(angleConv,heightConv, 0);
				state = HEIGHT;
				break;
			case CHEIGHT: // decrement height value (as long as >= MIN (0) )
				if (height > 0)
				{
					height = height - 0.1;
					ftoa(height,heightConv,1); // convert height to char array (heightConv) with 1 decimal place
					print_height_change(heightConv); // print conversion to LCD
				}
				break;
			case CANGLE: // increment angle value (as long as >= MIN (0))
				if(angle > 0)
				{
					angle = angle - 0.1;
					ftoa(angle,angleConv,1); // convert angle to char array (angleConv) with 1 decimal place
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
				ftoa(height,heightConv,1);
				print_height_change(heightConv);
				state = CHEIGHT;
				break;
			case ANGLE: // change to angle change state
				ftoa(angle,angleConv,1);
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

}

// Timer used to track seconds for startup routine of speed controller
ISR (TIMER3_OVF_vect)
{
	TCNT3 = -15625; // reset the 1sec timer value 
	overflowCount++;
	if (overflowCount == 5) // wait 5 seconds
	{
		OCR1A=38000-1;	// adjust pulse width of waveform being generated from 2ms to 1ms
		TCNT3 = 0; // set counter to 0
		TIMSK3 = (0 << TOIE3); // disable timer
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
	lcd_print(" ");
	lcd_gotoxy(10,2);
	lcd_print(" ");
	lcd_print(angleConv);
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

