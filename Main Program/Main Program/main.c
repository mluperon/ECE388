/*
	Code for: The Balancing Fan Project
	Group Name: The Midnight Snack Club
	Group Members: Alex Amorim, Liam Cross, Melanie Luperon, and Dylan Tocci
	
	Features:
		- User interface controlled by LCD display and Rotary encoder
		- 0-90 degrees of selection, user can increment by 1 degree at a time
		- 0-25 inches of selection, user can select 1 inch at a time
		- Live reading of fan arm position at all times
		- Adjustable contrast on LCD display
		- Debugging mode for developer (commented out)
		- Functioning PID loop
 */ 

#define F_CPU 16000000
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd_functions.h" // Dr. Viall's code for using the LCD
#include "floatConvert.h" // Code to convert a float into a character array
#include "Putty.h"	// See Putty.h for code source - used to attempt to interface with PuTTy

#define HEIGHT_SELECT "[HEIGHT]  ANGLE "
#define ANGLE_SELECT " HEIGHT  [ANGLE]"
#define NO_HS_SELECT " HEIGHT   ANGLE "
#define DEFAULT_ANGLE "   00       00 "
#define MAX_HEIGHT 15 // max height value 1.5ft
#define MAX_ANGLE 90 // max angle value 90 degrees
#define RIGHT 0b01110011 // PINC value for turning right
#define LEFT 0b01110101 // PINC value for turning left
#define BUTTON 0b01100111 // PINC value for pushing button
#define MIN_POT 14
#define MAX_POT 371
#define BASE PE3
//#define _BV(n) (1 << n)

// Function prototypes
void print_angle_change(char *);
void print_height_change(char *);
void print_height_angle(char *,  char *, int);
void test_angles();
void test_heights();
void peripheralSetup();

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
	
//PID variables
char tmpOutput[16] = ""; // Debugging char array for PID to print to LCD
volatile int pwmChange = 0;
volatile int previousPosition = 14;
volatile int previousPidError = 0;
volatile int integral = 0;
volatile int derivative = 0;
volatile int output = 0;
volatile int bias = 0;
volatile int tmpPotVal = 0;
volatile int overflow = 0;

//Adjustment variables
//These values below worked best up to this point in time
//They are seemingly random
volatile float kp = 50;
volatile float ki = 0;
volatile float kd = 600;

volatile int state = HEIGHT; // starts in height selection by default
volatile int height = 0; // global variable for height
volatile int angle = 0; // global variable for angle
volatile double dAngle = 0.0; // temporary double for angle
volatile double dHeight = 0.0; // temporary double for height
volatile int valueChange = 1; // Can be changed to modify how quickly height and angle may be adjusted
volatile int valueConfirm = 0; // flag signifying a new value for EITHER height or angle has been made. Will be used in PID loop to signal when to adjust fan speeds
volatile int setupFlag = 0; // flag signifying the setup is completed
volatile int potVal=0; // Holds current value for potentiometer (position of fan)
char heightConv[16] = ""; // global character array for storing height to be output to LCD
char angleConv[16] = ""; // global character array for storing angle to be output to LCD
char potConv[16] = ""; // Debugging char array used for printing value of potentiometer to LCD screen
volatile int overflowCount = 0; // global variable used for tracking how many times the timer overflows (each overflow is equivalent to one second)

int main(void)
{
	//USART_init();
	// These variables are Char arrays as the LCD cannot output int/float values, it must be sent as a string
	angleConv[0]='0'; // v Set up height and angle to start at 00.0 value
	angleConv[1]='0';
	angleConv[2]='\0';
	heightConv[0]='0';
	heightConv[1]='0';
	heightConv[2]='\0';
	// ^
	
	// Set up all outside ports (LCD, Rotary encoder, PWM, ADC, timers, etc.)
	peripheralSetup();
	
	// Enable input from ADC conversion
	ADCSRA |= (1<<ADSC);
	
	//SETUP COMPLETE
	lcd_gotoxy(1,1); // go to row 1 column 1 of LCD
	lcd_print("System start    "); // Print ->   [HEIGHT] ANGLE
	lcd_gotoxy(1,2); // go to row 2 column 2 of LCD
	lcd_print("completed!      "); // Print -> 00.0 00.0
	_delay_ms(1000);
	lcd_gotoxy(1,1);
	lcd_print("Enabling UI...  ");
	lcd_gotoxy(1,2);
	lcd_print("Enjoy! :)       ");
	_delay_ms(2000);
	
	// ***** Enable User Control ***** //		
	PCICR |= (1<<PCIE1); // Enable pin change interrupt on PORTC
	PCMSK1 |= (1<<PCINT12) | (1<<PCINT9) | (1<<PCINT10); // Enable Pins 1,2, and 4 to trigger this interrupt
	
	lcd_gotoxy(1,1); // go to row 1 column 1 of LCD
	lcd_print(HEIGHT_SELECT); // Print ->   [HEIGHT] ANGLE
	lcd_gotoxy(1,2); // go to row 2 column 2 of LCD
	lcd_print(DEFAULT_ANGLE); // Print -> 00.0 00.0

	// ***** MAIN LOOP ***** //
	//USART_init();
	// ^ Usart messes up LCD very badly, and could not be used for debug
	// Code can be found in Putty.h header file
	
	// PID variables were placed here to debug a bug found in earlier code - * does not seem necessary
	int desiredPosition = 0; // 0 degree desired upon startup
	int currentPosition = potVal; // set current position to whatever potentiometer reads while fan is flat
	int pidError = 0; 

	while(1)
	{
		
		//itos(potVal,potConv);
		//lcd_gotoxy(1,1);
		//lcd_print(potConv);
		
		//This is where the PID loop will be located
		//After the interrupt the PID will account for change
		//Must check to make sure value was confirmed before accounting for change in PID loop
		//This is why I have the valueConfirm variable
		
		// BASIC IDEA OF LOOP
		// +=================+
		// 1)Check position
		// 2)Compare to new position
		// 3)Adjust speed based on distance
		// 4)loop above
		// rough eq: potVal = 14 + (angle * 3.96);
		
		// need timer = ?
		currentPosition = potVal; // read current potentiometer value from ADC port
		if(valueConfirm == 1)
		{
			desiredPosition = (angle *3.96) + 14;
			// desiredPosition = 370 - ((angle *3.96) + 14);
			valueConfirm = 0; // reset flag
		}
		
// 		if ( angle == 0)
// 			currentPosition = 0; // calculate desired position
// 		else
// 			currentPosition = 14 + (angle * 3.96); // calculate desired position
		pidError = desiredPosition - currentPosition;
		integral = integral + pidError;
		derivative = pidError - previousPidError;
		output = kp*pidError + ki*integral + kd*derivative;
		
//	**** LCD PRINT STATEMENTS FOR DEBUGGING ****
// 		itos(abs(pidError), tmpOutput);
// 		lcd_gotoxy(1,1);
// 		lcd_print(tmpOutput);
// 		lcd_print("  ");
// 		itos(potVal,potConv);
// 		lcd_gotoxy (1,7);
// 		lcd_print(potConv);
//----------------------------------------------------------------------------------------------------------------



		// Set fan speed
		pwmChange =  -1 * (output); // Liam changed -------------------------------------------------------------------------
		if((OCR1A + pwmChange) <= 38000 && (OCR1A + pwmChange) >= 36000) // Ensure new PWM value within bounds of speed controller
			OCR1A = OCR1A + pwmChange - 1;
		else
		{
			if (OCR1A + pwmChange > 38000 ) // Speed is too large
			{
				overflow = OCR1A + pwmChange - 38000; // Calculate how much it overshoots and find max value that can be added within bounds
				OCR1A = OCR1A + (pwmChange - overflow - 1);	// Add said value ( -1 to be safe )
			}
			else
			{
				overflow = 36000 - (OCR1A + pwmChange); // Calculate how much it undershoots and find max value that can be added (pwmChange will be negative in this case) within bounds
				OCR1A = OCR1A + (pwmChange + overflow) + 1; // Add said value ( +1 to be safe )
			}
		}
		//------------------------------------------------------------------------------------------------------------------------------
		previousPidError = pidError; // Save current error as previous error for next iteration of PID loop
		
		// Possible delay for PID loop
		_delay_ms(1);
		
	}
	
	
}

// Rotary encoder interrupt based on pin change:
// -Adjusts LCD Display (what is selected)
// -Changes height / angle
//		- 0 <= angle <= 90 degrees
//		- 0 <= height <= 15 in
//		- maybe add feature to indicate height / angle reached max value on LCD
//	-Button press either:
//		- Confirms height / angle adjustment
//		- Selects angle / height and moves user to adjust height / angle
ISR(PCINT1_vect)
{
	_delay_ms(5);
//  ** Debug Code for showing PINC values when entering interrupt on LCD **
// 	ftoa(PINC, potConv);
// 	lcd_gotoxy(1,2);
// 	lcd_print(potConv);
	if(PINC == RIGHT) //if right turn triggered interrupt - 0b01110011 --- (PINC & 0b00000100) == 0b00000100
	{
		switch(state)
		{
			case HEIGHT: // change to angle state
				print_height_angle(angleConv,heightConv,0);
				state = ANGLE;
				break;
			case ANGLE:
				// do nothing
				break;
			case CHEIGHT: // increment height value (as long as < MAX (?))
				if ((height+valueChange) <= MAX_HEIGHT) // total guess right now
				{
					height = height + valueChange; //increment height by tenth
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
				// do nothing
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
				dAngle = dAngle * dAngle; // square dAngle
				dAngle = sqrt(225 - dAngle) / 15;
				dAngle = acos(dAngle) * (180 / M_PI);  // See Alex's lab notebook for formula used (combo of pythag. and trig.)
				angle = floor(dAngle);
				if (height == 15)
				{
					angle = 90;
				}
				ftoa(angle, angleConv);
				print_height_angle(angleConv,heightConv, 1); // refresh screen with height selected
				state = HEIGHT; // exit from change function
				break;
			case CANGLE: // confirm angle change value
				valueConfirm = 1; // set flag - might need to make unique flag
				// Convert angle to height
				dHeight = angle; // save angle as double
				dHeight = (dHeight) * (M_PI / 180); // convert angle to radians
				dHeight = 15*(cos(dHeight));
				dHeight = dHeight * dHeight;
				dHeight = sqrt((225-dHeight));
				height = floor(dHeight);
				ftoa(height,heightConv);
				print_height_angle(angleConv,heightConv, 0); // refresh screen with angle selected
				state = ANGLE;
				break;
		}
	}
	

	while(PINC != 0b01110111)
	{
		
	}

}



// Timer used to track seconds for startup routine of speed controller
ISR (TIMER3_OVF_vect)
{
	TCNT3 = -15625; // reset the 1sec timer value 
	overflowCount++;
	if (overflowCount == 5) // wait  seconds
	{
		OCR1A=38000-1;	// adjust pulse width of waveform being generated from 2ms to 1ms
		TCNT3 = 0; // set counter to 0
		TIMSK3 = (0 << TOIE3); // disable timer
		setupFlag = 1;
	}
}

ISR(ADC_vect)
{
	potVal = ADC;
	ADCSRA |= (1<<ADSC);
}

/************************************************* STARTUP ROUTINE **********************************************************************************/

void peripheralSetup()
{
	// Set up ADC on PortC(3)
	DDRC &= ~(1<<3); // Set PC3 as input
	PORTC |= (1 << 3);
	
	// Set registers for ADC on PortC3
	ADMUX = (0b01<<REFS0|(0<<ADLAR)|(0b0011<<MUX0));
	ADCSRA = (1<<ADEN)|(0<<ADSC)|(0<<ADATE)|(0<<ADIF)|(1<<ADIE)|(0b111<<ADPS0);
	ADCSRB = 0b000<<ADTS0;
	
	
	PORTE &= ~(1<<3); // Ensure 12V is OFF for speed controller 
	DDRE &= ~(1<<3);  // Ensure 12V is OFF for speed controller
	
	//************************************** PIN CHANGE INTERRUPT SETUP *************************************//
	DDRC &= ~(_BV(2) | _BV(1) | _BV(4)); // sets PORTC 1, 2, and 4 to input (input from rotary encoder)
	// 1 = DT signal
	// 2 = CLK signal
	// 4 = SW signal (button press)
	
	
	//********************************************** PWM SETUP **********************************************//
	// Set up PWM on PortB(1)
	DDRB |= (1<<1); // set PINB 1 to output
	PORTB = ~(1<<1); // Ensure PINB 1 is not outputting voltage
	TCCR1A= (0b11 << COM1A0) | ( 0b00 << COM1B0) | (0b10 << WGM10); // set up PWM with pre-scalar
	TCCR1B= (0b11 << WGM12) | (0b010<< CS10);
	ICR1= 40000-1; // (20MS /8 PRESCALAR)
	OCR1A=36000-1; // 1000->4000 0.5ms to 2ms *** adjust ***
	
	// Set up 4 second timer for startup
	TCCR3A = 0; //(0b00 << COM3A0) | (0b00 << COM3B0)
	TCCR3B = (0 << ICNC3) | (0 << ICES3) | (0b00 << WGM32) | (0b101 << CS30);
	TIMSK3 = (0 << TOIE3); // Ensure timer 3 is disabled
	TCNT3 = -15625; // One second timer value
	
	// Enable global interrupts
	sei();
	
	// Set up LCD and print starting message state
	lcd_init(); // initialize the LCD according to Dr. Viall's 263 code
	lcd_gotoxy(1,1); // go to row 1 column 1 of LCD
	lcd_print("Starting system"); // Print ->   [HEIGHT] ANGLE
	lcd_gotoxy(1,2); // go to row 2 column 2 of LCD
	lcd_print("Please wait..."); // Print -> 00.0 00.0
	
	
	// ***** SPEED CONTROLLER STARTUP ***** //
	TIMSK3 = (1 << TOIE3); // Enable PWM timer for startup
	PORTE |= (1<<BASE); // TURN ON 12V SUPPLY
	DDRE |= (1<<BASE);   // TURN ON 12V SUPPLY

	while (setupFlag != 1)
	{
		// Wait until setup is completed
		// This flag will be set by Timer3 Overflow, after it has counted 5 seconds
		// This gives the speed controller time to set up according to its specs on the datasheet
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
	lcd_print("   "); // remove bracket
	lcd_print(heightConv);
	lcd_print(" "); //remove bracket
	lcd_gotoxy(12,2);
	lcd_print(" "); //remove bracket
	lcd_print(angleConv);
	lcd_print(" "); //remove bracket
}

// Function used to print a change in the height value
void print_height_change(char *conversion)
{
	//ensure height becomes unselected
	lcd_gotoxy(1,1);
	lcd_print(NO_HS_SELECT);
	lcd_gotoxy(3,2);
	lcd_print("[");
	lcd_print(conversion);
	lcd_print("]");
}

// Function used to print a change in the angle value
void print_angle_change(char *conversion)
{
	lcd_gotoxy(1,1);
	lcd_print(NO_HS_SELECT);
	lcd_gotoxy(12,2);
	lcd_print("[");
	lcd_print(conversion);
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

