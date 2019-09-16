/*
 * Prelab 01, ECE388.c
 *
 * Created: 9/12/2019 11:05:50 AM
 * Author : Alex Amorim, Liam Cross, Dylan Tocci, Melanie Luperon 
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>


int timerValues[3] = {-1563,-781,-7812}; //start at 5 Hz
int timerCount=0;

int main(void)
{
	DDRB=(1<<5);//0b00100000;
	TCCR1A = (0b00<<COM1A0)|(0b00<<COM1B0)|(0b00<<WGM10);
	TCCR1B = (0b0<<ICNC1)|(0b0<<ICES1)|(0b00<<WGM12)|(0b101<<CS10); // 1024 pre-scalar
	
	TIMSK1=(0b1<<TOIE1);
	TCNT1 = timerValues[timerCount];
	
	sei();
    /* Replace with your application code */
    while (1) 
    {
		if((PINB & (1 << 7)) == 0)
		{
			timerCount = (timerCount + 1) % 3;
			TCNT1 = timerValues[timerCount];	
			while((PINB & (1 << 7)) == 0)
			{
				
			}
		}
    }

}

ISR (TIMER1_OVF_vect)
{
	PORTB ^= (1<<5);
	TCNT1=timerValues[timerCount];
}
