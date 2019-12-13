/*
	Code for printing to the LCD
	Taken from old code of ECE 263 Course
	Code by: Dr. Phillip Viall

 */ 


#ifndef LCD_FUNCTIONS_H_
#define LCD_FUNCTIONS_H_

#define	LCD_DPRT  PORTD
#define	LCD_DDDR  DDRD
#define	LCD_DPIN  PIND
#define	LCD_CPRT  PORTE
#define	LCD_CDDR  DDRE
#define	LCD_CPIN  PINE
#define	LCD_RS  0
#define	LCD_RW  1
#define	LCD_EN  2



//*******************************************************
void lcdCommand( unsigned char cmnd )
{
  LCD_DPRT = cmnd;			
  LCD_CPRT &= ~ (1<<LCD_RS);
  LCD_CPRT &= ~ (1<<LCD_RW);
  LCD_CPRT |= (1<<LCD_EN);	
  _delay_us(1);				
  LCD_CPRT &= ~ (1<<LCD_EN);
  _delay_us(100);			
}

//*******************************************************
void lcdData( unsigned char data )
{
  LCD_DPRT = data;			
  LCD_CPRT |= (1<<LCD_RS);	
  LCD_CPRT &= ~ (1<<LCD_RW);
  LCD_CPRT |= (1<<LCD_EN);	
  _delay_us(1);				
  LCD_CPRT &= ~ (1<<LCD_EN);
  _delay_us(100);			
}

//*******************************************************
void lcd_init()
{
  LCD_DDDR = 0xFF;
  LCD_CDDR = 0xFF;
 
  LCD_CPRT &=~(1<<LCD_EN);	
  _delay_us(2000);			
  lcdCommand(0x38);					
  lcdCommand(0x0C);
  lcdCommand(0x01); //clear
  _delay_us(2000);			
  lcdCommand(0x06);			
}

//*******************************************************
void lcd_gotoxy(unsigned char x, unsigned char y)
{  
 unsigned char firstCharAdr[]={0x80,0xC0,0x94,0xD4};//table 12-5  
 lcdCommand(firstCharAdr[y-1] + x - 1);
 _delay_us(100);	
}

//*******************************************************
void lcd_print( char * str )
{
  unsigned char i = 0 ;
  while(str[i]!=0)
  {
    lcdData(str[i]);
    i++ ;
  }
}





#endif /* LCD_FUNCTIONS_H_ */