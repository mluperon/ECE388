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

#define DEFAULT_MEASUREMENT " 00.0     00.0"
#define HEIGHT_SELECT "[HEIGHT]  ANGLE "
#define ANGLE_SELECT " HEIGHT  [ANGLE]"
#define NO_HS_SELECT " HEIGHT   ANGLE "
#define DEFAULT_ANGLE " 00.0     00.0"


int main(void)
{
	char heightConv[16] = "";
	char angleConv[16] = "";
	char test[4] = "";
	float height = 10.3;
	float angle = 00.0;
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
	
		
	_delay_ms(2000);
	print_height_change(heightConv);
	_delay_ms(2000);
	height=33.8;
	ftoa(height,heightConv,1);
	print_height_change(heightConv);
	
	//BEGIN CHANGING VALUES ETC.
	_delay_ms(2000);
	print_height_angle(angleConv,heightConv, 1); // select height
	_delay_ms(2000);
	print_height_angle(angleConv,heightConv, 0); //angle select
	_delay_ms(2000);
	print_angle_change(angleConv);
	_delay_ms(2000);
	angle=37.9;
	ftoa(angle,angleConv,1);
	print_angle_change(angleConv);
	_delay_ms(2000);
	print_height_angle(angleConv,heightConv,1); 
	
}

void print_height_angle(char *angleConv, char *heightConv, int heightSelect) // possible way to fix based on state 0=height 1=angle
{
	lcd_gotoxy(1,1);
	if (heightSelect == 1)
		lcd_print(HEIGHT_SELECT);
	else 
		lcd_print(ANGLE_SELECT);
	//unselect bottom row
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

