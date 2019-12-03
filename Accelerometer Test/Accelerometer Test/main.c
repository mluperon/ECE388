/*
 * Accelerometer Test.c
 *
 * Created: 12/2/2019 4:07:55 PM
 * Author : dylma
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

// This code will be used to test the accelerometer.
// It should be able to poll and read data from the accelerometer using either I2C or SPI

// PIN WIRING
// VDD = 3.3V (or 5V)
// CS -> Bitwise level shit -> PB2 (SS0; Slave select line)
// SDA -> Bitwise level shit -> PB3 (MOSI; Master out slave in)
// SD0 -> Bitwise level shit -> PB4 (MISO; Master in slave out)
// SCL -> Bitwise level shit -> PB5 (SCK0; Clock signal)
// GND -> GND

int main(void)
{
	// Set up SPI0 Registers
	SPCR0 = (1 << SPIE) | (1 << SPE) | (0 << DORD) | (1 << MSTR) | (1 << CPOL) | (1 << CPHA) | (0b00< SPR0); // Enable SPI Flag; Enable SPI; Set Atmega as Master; SCK high when idle and sampled on leading; Clock rate = 4MHz 
	SPSR0 = (1 << SPIF) | (1 << WCOL) | (0 << SPI2X); // Set interrupt flag; Set Write collision flag; 4MHz clock
    // SPDR0 = Data register
	// When writing to data register, transmission between it and the Shift register will begin
	// When reading from data register, the Shift register receive buffer will be read. 
	// When sending data to accelerometer, CS must be low at start, and end on high
	
	// *************************** SET UP ACCELEROMETER *************************** //
	// For 4-wire SPI, SPI bit in DATA_FORMAT register must be cleared
	// This register is located at address 0x31 (0b110001)
	// DATA_FORMAT clear D6
	
	SPDR0 = 0b110001; // set multiple byte mode to 1 and address 0x31
	SPI_wait();
	// Set range bits to D0 = 1, D1 = 0 4g
	// Justify right, with sign extension D2 = 0
	// FULL_RES D3 = 1 full resolution
	// D4 = 0 default
	// INT_INVERT D5 = 1 active low interrupt 
	// SPI D6 = 0 4-wire SPI
	// SELF_TEST D7 = 0, no self-test
	SPDR0 = 0b00101001;
	SPI_wait();
	_delay_ms(1); // Wait to signify new signal
	SPDR0 = 0b1111000; // set multiple byte mode to 1 and address to 0x38
	SPI_wait();
	
	// FIFO Mode D7 AND D6 = 0, bypass
	// Trigger mode D5=0, LINKED TO INT1
	// Samples = 0, bypass everything
	SPDR0 = 0b00000000;
	SPI_wait();
	_delay_ms(1); // Wait to signify new signal

	
	/* Replace with your application code */
    while (1) 
    {
		
    }
}


void SPI_wait()
{
	while (SPIF != 1)
	{
		// Wait for transfer to finish
	}
}