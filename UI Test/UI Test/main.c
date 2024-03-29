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
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include "lcd_functions.h" // Dr. Viall's code for using the LCD
#include "floatConvert.h" // Code to convert a float into a character array

#define HEIGHT_SELECT "[HEIGHT]  ANGLE "
#define ANGLE_SELECT " HEIGHT  [ANGLE]"
#define NO_HS_SELECT " HEIGHT   ANGLE "
#define DEFAULT_ANGLE " 00.0     00.0"
#define MAX_HEIGHT 150 // max height value 15in
#define MAX_ANGLE 900 // max angle value 90 degrees
#define RIGHT 123 // PINC value for turning right
#define LEFT 125 // PINC value for turning left
#define BUTTON 111 // PINC value for pushing button

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
volatile int height = 0; // global variable for height
volatile int angle = 0; // global variable for angle
volatile double dAngle = 0.0;
volatile double dHeight = 0.0;
volatile int valueChange = 1; // increment / decrement height and angle by this value
volatile int valueConfirm = 0; // flag signifying a new value for EITHER height or angle has been made. Will be used in PID loop to signal when to adjust fan speeds
char heightConv[16] = ""; // global character array for storing height to be output to LCD
char angleConv[16] = ""; // global character array for storing angle to be output to LCD

void print_angle_change(char *);
void print_height_change(char *);
void print_height_angle(char *,  char *, int);
void test_angles();
void test_heights();

int main(void)
{

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

float tmp=0;
ISR(PCINT1_vect) 
{
	_delay_ms(5);
	tmp=PINC;
// 	ftoa(tmp,heightConv);
// 	print_height_change(heightConv);
	
	if(PINC == RIGHT) //if right turn triggered interrupt
	{
		switch(state)
		{
			case HEIGHT: // change to angle state
				print_height_angle(angleConv,heightConv,0);
				state = ANGLE;
				break;
			case ANGLE: // Change increment/decrement value to 10
				valueChange = 10;
				cli();
				lcd_gotoxy(1,1);
				lcd_print("Order +-1       ");
				_delay_ms(1000);
				lcd_gotoxy(1,1);
				lcd_print(ANGLE_SELECT);
				sei();
				break;
			case CHEIGHT: // increment height value (as long as < MAX (?))
				if ((height+valueChange) <= MAX_HEIGHT) // total guess right now
				{
					height=height + valueChange; //increment height by tenth
					ftoa(height,heightConv); // convert height to char array (heightConv) with 1 decimal place
					print_height_change(heightConv); // print conversion to LCD
				}
				break;	
			case CANGLE: // increment angle value (as long as <= MAX (90))
				if ((angle+valueChange) <= MAX_ANGLE)
				{
					angle=angle + valueChange;
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
			case HEIGHT: // Change increment/decrement value to 1
				valueChange = 1;
				cli();
				lcd_gotoxy(1,1);
				lcd_print("Order +-.1     ");
				_delay_ms(1000);
				lcd_gotoxy(1,1);
				lcd_print(HEIGHT_SELECT);
				sei();
				break;
			case ANGLE: // change to height state
				print_height_angle(angleConv,heightConv, 1);
				state = HEIGHT;
				break;
			case CHEIGHT: // decrement height value (as long as >= MIN (0) )
				if ((height-valueChange) >= 0)
				{
					height = height - valueChange;
					ftoa(height,heightConv); // convert height to char array (heightConv) with 1 decimal place
					print_height_change(heightConv); // print conversion to LCD
				}
				break;
			case CANGLE: // increment angle value (as long as >= MIN (0))
				if((angle-valueChange) >= 0)
				{
					angle = angle - valueChange;
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
				// Convert height to angle
				dAngle = height; // save height variable as double 
				dAngle = dAngle / 10; // remove extra power from when stored as int
				dAngle = dAngle * dAngle; // square dAngle
				dAngle = sqrt(225 - dAngle) / 15;
				dAngle = acos(dAngle) * (180 / M_PI);  // See Alex's lab notebook for formula used (combo of pythag. and trig.)
				angle = floor((dAngle * 10));
				if (height == 150)
				{
					angle = 900;
				}
				ftoa(angle, angleConv);
				print_height_angle(angleConv,heightConv, 1); // refresh screen with height selected 
				state = HEIGHT; // exit from change function
				break;
			case CANGLE: // confirm angle change value
				valueConfirm = 1; // set flag - might need to make unique flag
				// Convert angle to height
				dHeight = angle; // save angle as double
				dHeight = (dHeight/10) * (M_PI / 180); // convert angle to radians
				dHeight = 15*(cos(dHeight));
				dHeight = dHeight * dHeight;
				dHeight = sqrt((225-dHeight));
				height = floor((dHeight*10));
				ftoa(height,heightConv);
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


/************************************************* REQUIREMENT TEST FUNCTIONS **********************************************************************************/

// This function will loop through each value possible for the angle and display it on the LCD display. 5 numbers will be displayed each second
void test_angles()
{
	while (angle <= MAX_ANGLE) 
	{
		ftoa(angle,angleConv);
		print_angle_change(angleConv);
		angle++;
		_delay_ms(200);
	}
}


// This function will loop through each value possible for the height and display it on the LCD display. 5 numbers will be displayed each second
void test_heights()
{
	while (angle <= MAX_HEIGHT)
	{
		ftoa(height,heightConv);
		print_angle_change(heightConv);
		height++;
		_delay_ms(200);
	}
}