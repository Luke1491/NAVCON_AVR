/*
 * MAX7219.c
 *
 * Created: 2017-05-26 18:49:22
 *  Author: LUKE
 */ 
// 16MHz clock
//#define F_CPU 16000000UL

// Outputs, pin definitions
#define PIN_SCK                   PB5
#define PIN_MOSI                  PB3
#define PIN_SS                    PB1

#define ON                        1
#define OFF                       0

#define MAX7219_LOAD1             PORTB |= (1<<PIN_SS)
#define MAX7219_LOAD0             PORTB &= ~(1<<PIN_SS)

#define MAX7219_MODE_DECODE       0x09
#define MAX7219_MODE_INTENSITY    0x0A
#define MAX7219_MODE_SCAN_LIMIT   0x0B
#define MAX7219_MODE_POWER        0x0C
#define MAX7219_MODE_TEST         0x0F
#define MAX7219_MODE_NOOP         0x00

// I only have 3 digits, no point having the
// rest. You could use more though.
#define MAX7219_DIGIT0            0x01
#define MAX7219_DIGIT1            0x02
#define MAX7219_DIGIT2            0x03
#define MAX7219_DIGIT3            0x04
#define MAX7219_DIGIT4            0x05
#define MAX7219_DIGIT5            0x06
#define MAX7219_DIGIT6            0x07
#define MAX7219_DIGIT7            0x08

#define MAX7219_CHAR_BLANK        0xF
#define MAX7219_CHAR_NEGATIVE     0xA

#include <avr/io.h>
#include "MAX7219.h"
#include <util/atomic.h>




char digitsInUse = 8;

void MAX7219_Init(void) {
	//SPI inicjalization made in enc28j60.c in enc28j60Init() function. Parameters set as below:
	//If you not using enc28j60, uncomment below lines
	
	// SCK MOSI CS/LOAD/SS
	DDRB |= (1 << PIN_SCK) | (1 << PIN_MOSI) | (1 << PIN_SS);

	// SPI Enable, Master mode
	SPCR |= (1 << SPE) | (1 << MSTR)| (1<<SPI2X); //TRYB MASTER, PRESKALER F_CPU/2
	

	// Decode mode to "Font Code-B"
	MAX7219_writeData(MAX7219_MODE_DECODE, 0xFF);

	// Scan limit runs from 0.
	MAX7219_writeData(MAX7219_MODE_SCAN_LIMIT, digitsInUse - 1);
	MAX7219_writeData(MAX7219_MODE_INTENSITY, 10);
	MAX7219_writeData(MAX7219_MODE_POWER, ON);
}

void spiSendByte (char databyte)
{
	// Copy data into the SPI data register
	SPDR = databyte;
	// Wait until transfer is complete
	while (!(SPSR & (1 << SPIF)));
}

void MAX7219_writeData(char data_register, char data)
{
	MAX7219_LOAD0;
	// Send the register where the data will be stored
	spiSendByte(data_register);
	// Send the data to be stored
	spiSendByte(data);
	MAX7219_LOAD1;
}

void MAX7219_clearDisplay(void)
{
	char i = digitsInUse;
	// Loop until 0, but don't run for zero
	do {
		// Set each display in use to blank
		MAX7219_writeData(i, MAX7219_CHAR_BLANK);
	} while (--i);
}

void MAX7219_displayNumber(volatile long number)
{
	char negative = 0;

	// Convert negative to positive.
	// Keep a record that it was negative so we can
	// sign it again on the display.
	if (number < 0) {
		negative = 1;
		number *= -1;
	}


	MAX7219_clearDisplay();

	// If number = 0, only show one zero then exit
	if (number == 0) {
		MAX7219_writeData(MAX7219_DIGIT0, 0);
		return;
	}
	
	// Initialization to 0 required in this case,
	// does not work without it. Not sure why.
	char i = 0;
	
	// Loop until number is 0.
	do {
		MAX7219_writeData(++i, number % 10);
		// Actually divide by 10 now.
		number /= 10;
	} while (number);

	// Bear in mind that if you only have three digits, and
	// try to display something like "-256" all that will display
	// will be "256" because it needs an extra fourth digit to
	// display the sign.
	if (negative) {
		MAX7219_writeData(i, MAX7219_CHAR_NEGATIVE);
	}
}

void MAX7219_SendCourseAndSpeed(uint16_t course, int16_t speed) {
	uint8_t znak = 1;   //zmienna przechowuj¹ca informacje o znaku 1-->wiêksza od zera, 0-->niejsza od zera
	if(speed < 0) znak = 0;
	if(speed >999 || speed < -999) {
		MAX7219_writeData(MAX7219_DIGIT0, MAX7219_CHAR_NEGATIVE); //wyœwitl kreskê 
		MAX7219_writeData(MAX7219_DIGIT1, MAX7219_CHAR_NEGATIVE); //wyœwitl kreskê 
		MAX7219_writeData(MAX7219_DIGIT2, MAX7219_CHAR_NEGATIVE); //wyœwitl kreskê 
		MAX7219_writeData(MAX7219_DIGIT3, MAX7219_CHAR_NEGATIVE); //wyœwitl kreskê 
		} //je¿eli sped bêdzie wiêksza od 999 lub mniejsza od -999 to poka¿ same kreski
	else {
		if(!znak){ speed = -speed; MAX7219_writeData(MAX7219_DIGIT3, MAX7219_CHAR_NEGATIVE); } //je¿eli speed jest ujemna (zmienna znak ma wartoœæ false) to zmieñ znak i wyœwietl na miejscu 4 cyfry znak negative.
		MAX7219_writeData(MAX7219_DIGIT0, speed % 10);       // ostatnia cyfra wyœwietlana na wyœwietlaczu
		MAX7219_writeData(MAX7219_DIGIT1, ((speed / 10) % 10) | 0x80);  // cyfra 2 + wyœwietl znak dziesiêtny (kropki)
		MAX7219_writeData(MAX7219_DIGIT2, (speed / 100) % 10);  // cyfra 3
		if(znak) MAX7219_writeData(MAX7219_DIGIT3,MAX7219_CHAR_BLANK);  // wygaœ cyfrê
		}
	//####################################################################################
	if (course > 3600) {
		MAX7219_writeData(MAX7219_DIGIT4, MAX7219_CHAR_NEGATIVE); //wyœwitl kreskê 
		MAX7219_writeData(MAX7219_DIGIT5, MAX7219_CHAR_NEGATIVE); //wyœwitl kreskê 
		MAX7219_writeData(MAX7219_DIGIT6, MAX7219_CHAR_NEGATIVE); //wyœwitl kreskê 
		MAX7219_writeData(MAX7219_DIGIT7, MAX7219_CHAR_NEGATIVE); //wyœwitl kreskê 
	}
	MAX7219_writeData(MAX7219_DIGIT4, course % 10 ); //cyfra dziesiêtna kursu
	MAX7219_writeData(MAX7219_DIGIT5, ((course / 10) % 10) | 0x80 ); //cyfra jednosci kursu + kropka
	MAX7219_writeData(MAX7219_DIGIT6, (course / 100) % 10 ); //cyfra dziêsiêtnych kursu
	MAX7219_writeData(MAX7219_DIGIT7, (course / 1000) % 10 ); //cyfra setnych kursu
}

void MAX7219_autopilotOFF(void)
{
	MAX7219_writeData(MAX7219_DIGIT0, MAX7219_CHAR_NEGATIVE); //wyœwitl kreskê
	MAX7219_writeData(MAX7219_DIGIT1, MAX7219_CHAR_NEGATIVE); //wyœwitl kreskê
	MAX7219_writeData(MAX7219_DIGIT2, MAX7219_CHAR_NEGATIVE); //wyœwitl kreskê
	MAX7219_writeData(MAX7219_DIGIT3, MAX7219_CHAR_NEGATIVE); //wyœwitl kreskê
	MAX7219_writeData(MAX7219_DIGIT4, MAX7219_CHAR_NEGATIVE); //wyœwitl kreskê
	MAX7219_writeData(MAX7219_DIGIT5, MAX7219_CHAR_NEGATIVE); //wyœwitl kreskê
	MAX7219_writeData(MAX7219_DIGIT6, MAX7219_CHAR_NEGATIVE); //wyœwitl kreskê
	MAX7219_writeData(MAX7219_DIGIT7, MAX7219_CHAR_NEGATIVE); //wyœwitl kreskê
}


	

