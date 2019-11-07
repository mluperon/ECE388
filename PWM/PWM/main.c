/*
 * PWM.c
 *
 * Created: 11/7/2019 10:58:30 AM
 * Author : dylma
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>

volatile int overflowCount = 0;

//PWM uses pb1
int main(void)
{
	// Set up PWM on PortB(1)
	DDRB |= (1<<1);
	PORTB = 0;
 	TCCR1A= (0b11 << COM1A0) | ( 0b00 << COM1B0) | (0b10 << WGM10);
 	TCCR1B= (0b11 << WGM12) | (0b010<< CS10);
 	ICR1= 40000-1; // (20MS /8 PRESCALAR)
 	OCR1A=36000-1; // 1000->4000 0.5ms to 2ms *** adjust ***
	
	// Set up 5 second timer for startup	
	TCCR3A = 0; //(0b00 << COM3A0) | (0b00 << COM3B0) | ()
	TCCR3B = (0 << ICNC3) | (0 << ICES3) | (0b00 << WGM32) | (0b101 << CS30);
	TIMSK3 = (1 << TOIE3);
	TCNT3 = -15625;
	

	sei();
    /* Replace with your application code */
    while (1) 
    {
    }
}

ISR (TIMER3_OVF_vect)
{
	TCNT3 = -15625;
	overflowCount++;
	if (overflowCount == 5)
		OCR1A=38000-1;	// adjust timer1
	
}

