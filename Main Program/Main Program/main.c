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
#define MAX_HEIGHT 150 // max height value 1.5ft
#define MAX_ANGLE 900 // max angle value 90 degrees
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
volatile int height = 0; // global variable for height
volatile int angle = 0; // global variable for angle
volatile int valueChange = 1;
volatile int valueConfirm = 0; // flag signifying a new value for EITHER height or angle has been made. Will be used in PID loop to signal when to adjust fan speeds
volatile int setupFlag = 0;
volatile char heightConv[16] = ""; // global character array for storing height to be output to LCD
volatile char angleConv[16] = ""; // global character array for storing angle to be output to LCD
volatile int overflowCount = 0; // global variable used for tracking how many times the timer overflows (each overflow is equivalent to one second)

int main(void)
{
	PORTE &= ~(1<<3); // Ensure 12V is ON
	DDRE &= ~(1<<3);  // Ensure 12V is OFF
	
	//********************************************** PIN CHANGE INTERRUPT SETUP **********************************************//
	DDRC &= ~(_BV(2) | _BV(1) | _BV(4)); // sets PORTC 1, 2, and 4 to input (input from rotary encoder)
										 // 1 = DT signal
										 // 2 = CLK signal
										 // 4 = SW signal (button press)
	
	
	//********************************************** PWM SETUP **********************************************//
	// Set up PWM on PortB(1)
	DDRB |= (1<<1); // set PINB 1 to output
	PORTB &= ~(1<<1); // Ensure PINB 1 is not outputting voltage
	TCCR1A= (0b11 << COM1A0) | ( 0b00 << COM1B0) | (0b10 << WGM10); // set up PWM with pre-scalar
	TCCR1B= (0b11 << WGM12) | (0b010<< CS10);
	ICR1= 40000-1; // (20MS /8 PRESCALAR)
	OCR1A=36000-1; // 1000->4000 0.5ms to 2ms *** adjust ***
	
	// Set up 5 second timer for startup
	TCCR3A = 0; //(0b00 << COM3A0) | (0b00 << COM3B0)
	TCCR3B = (0 << ICNC3) | (0 << ICES3) | (0b00 << WGM32) | (0b101 << CS30);
	TIMSK3 = (0 << TOIE3); // Ensure timer 3 is disabled
	TCNT3 = -15625; // One second timer value
	
	// These variables are Char arrays as the LCD cannot output int/float values, it must be sent as a string
	angleConv[0]='0'; // v Set up height and angle to start at 00.0 value
	angleConv[1]='0';
	angleConv[2]='.';
	angleConv[3]='0';
	heightConv[0]='0';
	heightConv[1]='0';
	heightConv[2]='.';
	heightConv[3]='0';// ^
	
	// Set up LCD and print starting message state 
	lcd_init(); // initialize the LCD according to Dr. Viall's 263 code
	lcd_gotoxy(1,1); // go to row 1 column 1 of LCD
	lcd_print("Starting system"); // Print ->   [HEIGHT] ANGLE
	lcd_gotoxy(1,2); // go to row 2 column 2 of LCD
	lcd_print("Please wait..."); // Print -> 00.0 00.0
	
	
	sei(); // enable global interrupts
	
	// ***** SPEED CONTROLLER STARTUP ***** //
	TIMSK3 = (1 << TOIE3); // Enable PWM timer for startup
	PORTE &= ~(1<<3); // TURN ON 12V SUPPLY
	DDRE |= (1<<3);   // TURN ON 12V SUPPLY
	
	PORTB |= (1<<1); // Begin outputting 2ms pulse
	// Timer 3 will auto adjust pulse length after this routine
	
	while(setupFlag != 1)
	{
		// Wait for setup to finish
	}
	
	//SETUP COMPLETE
	lcd_gotoxy(1,1); // go to row 1 column 1 of LCD
	lcd_print("System start    "); // Print ->   [HEIGHT] ANGLE
	lcd_gotoxy(1,2); // go to row 2 column 2 of LCD
	lcd_print("completed!      "); // Print -> 00.0 00.0
	_delay_ms(1000);
	lcd_print("Enabling UI...  ");
	lcd_print("Enjoy! :)       ");
	
	// ***** Enable User Control ***** //		
	PCICR |= (1<<PCIE1); // Enable pin change interrupt on PORTC
	PCMSK1 |= (1<<PCINT11) | (1<<PCINT9) | (1<<PCINT10); // Enable Pins 1,2, and 4 to trigger this interrupt
	
	
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
ISR(PCINT1_vect)
{
	_delay_ms(5);
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
		// Wait for PINC to return to default state 
		// i.e. no pulse being sent from rotary encoder	
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
		setupFlag = 1;
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