/*
 * ECE388_PID.c
 *
 * Created: 12/2/2019 4:24:01 PM
 * Author : The Midnight Snack Club 
 * Model code retrieved from http://brettbeauregard.com/blog/tag/beginners-pid/
 */ 

#include <stdio.h>
 
// Variables 
unsigned long lastTime; 
double Input, Output, Setpoint;
double errorSum, lastError;
double kp, ki, kd;

void Compute()
{
	//Time since last calculated 
	unsigned long now = millis();     // Arduino function,; returns the current time in milliseconds 
	double timeChange = (double) (now - lastTime);
	
	//Compute all the working error variables
	double errorSum = Setpoint - Input;
	errorSum += (errorSum * timeChange);
	double dErr = ( errorSum - lastError) / timeChange;
	lastError = errorSum
	//PID output
	Output = kp * errorSum + ki * errorSum + kd * dErr;
	
	lastError = errorSum; 
	lastTime = now; 
}

void SetTunings(double Kp, double Ki, double Kd)
{
	kp = Kp;
	ki = Ki;
	kd = Kd;
}  

#include <avr/io.h> 


int main(void)
{    while (1) 
    {
		Compute();
    }
}

