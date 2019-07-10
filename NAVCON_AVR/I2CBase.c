/*
 * I2CBase.c
 *
 * Created: 2017-05-26 19:15:23
 *  Author: LUKE
 
 ************Biblioteka do obs³ugi g³ównego wyœwietlacza (kurs  i prêdkoœæ)****************
 */ 
#include "I2CBase.h"
#include <avr/io.h>



uint8_t I2C_Error;
uint8_t cy1;
uint8_t cy2;
uint8_t cy3;
uint8_t cy4;
uint8_t cy5;
uint8_t cy6;
uint8_t cy7;
uint8_t cy8;

void I2C_Init()
{
	TWCR =_BV(TWEA) | _BV(TWEN);		//W³¹cz interfejs I2C
	I2C_SetBusSpeed(I2CBUSCLOCK/100);
}

void I2C_Start()
{
	TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
	I2C_WaitForComplete();
	if((TW_STATUS!=TW_START) && (TW_STATUS!=TW_REP_START)) I2C_SetError(I2C_STARTError);
}

void I2C_SendAddr(uint8_t address)
{
	uint8_t Status;
	
	if((address & 0x01)==0) Status=TW_MT_SLA_ACK;
	else Status=TW_MR_SLA_ACK;
	
	TWDR=address;
	TWCR=_BV(TWINT) | _BV(TWEN);
	I2C_WaitForComplete();
	if(TW_STATUS!=Status) I2C_SetError(I2C_NoACK); else I2C_SetError(I2C_OK);
}

void I2C_SendByte(uint8_t byte)
{
	TWDR=byte;
	TWCR=_BV(TWINT) | _BV(TWEN);
	I2C_WaitForComplete();
	if(TW_STATUS!=TW_MT_DATA_ACK) I2C_SetError(I2C_NoACK);
}

uint8_t I2C_ReceiveData_ACK()
{
	TWCR=_BV(TWEA) | _BV(TWINT) | _BV(TWEN);
	I2C_WaitForComplete();
	if(TW_STATUS!=TW_MR_DATA_ACK) I2C_SetError(I2C_NoACK);
	return TWDR;
}

uint8_t I2C_ReceiveData_NACK()
{
	TWCR=_BV(TWINT) | _BV(TWEN);
	I2C_WaitForComplete();
	if (TW_STATUS!=TW_MR_DATA_NACK) I2C_SetError(I2C_NoNACK);
	return TWDR;
}

void I2C_SetBusSpeed(uint16_t speed)
{
	speed=(F_CPU/speed/100-16)/2;		//speed=TWBR*4^TWPS
	uint8_t prescaler=0;
	while(speed>255)				//Oblicz wartoœæ preskalera
	{
		prescaler++;
		speed=speed/4;
	};
	TWSR=(TWSR & (_BV(TWPS1)|_BV(TWPS0))) | prescaler;
	TWBR=speed;
}

void I2C_SendStartAndSelect(uint8_t addr)
{
	I2C_Start();
	I2C_SendAddr(addr);
}

void I2C_SendTxt(char *txt)
{
	while(*txt) I2C_SendByte(*txt++);
	I2C_SendByte(0);	//Wyœlij znak koñca ³añcucha
}


void I2C_StartSelectWait(uint8_t addr)
{
	do
	{
		I2C_SendStartAndSelect(addr);
	}while(I2C_Error==I2C_NoACK);
}

void wyswietl_led(int speed, int course) {  //int w AVR ma wielkoœæ 2 bajtów czyli int == int16_t
	uint8_t znak = 1;   //zmienna przechowuj¹ca informacje o znaku 1-->wiêksza od zera, 0-->niejsza od zera
	if(speed < 0) znak = 0;
	if(speed >999 || speed < -999) {cy5 = 11; cy6 = 11; cy7 = 11; cy8 = 11;} //je¿eli sped bêdzie wiêksza od 999 lub mniejsza od -999 to poka¿ same kreski
	else {
		if(!znak){ speed = -speed; cy4 = 11; } //je¿eli speed jest ujemna (zmienna znak ma wartoœæ false) to zmieñ znak i wyœwietl na miejscu 4 cyfry znak negative (11 indeks w tablicy cyfry)
		cy1 = speed % 10;       // ostatnia cyfra wyœwietlana na wyœwietlaczu
		cy2 = (speed / 10) % 10;  // cyfra 2
		if(speed <100 && speed > -100) cy3 = 10; //kod 10 w tablicy cyfry wygasza wszystkie segmenty
		cy3 = (speed / 100) % 10;  // cyfra 3
		if(znak) cy4 = 10;           //indeks w tablicy który przypisany jest wy³¹czonemy wyœwietlaczowi
	}
	//########################################
	if (course > 3600) {cy5 = 11; cy6 = 11; cy7 = 11; cy8 = 11;}
	cy5 = course % 10;
	cy6 = (course / 10) % 10;
	cy7 = (course / 100) % 10;
	cy8 = (course / 1000) % 10;
	
	
}

void I2C_SendCourseAndSpeed(int course, int speed, uint8_t adrr) {
	wyswietl_led( speed, course);
	I2C_StartSelectWait(adrr);
	I2C_SendByte('s'); I2C_SendByte(cy1); I2C_SendByte(cy2); I2C_SendByte(cy3); I2C_SendByte(cy4);
	I2C_Stop();
	I2C_WaitTillStopWasSent();
	I2C_StartSelectWait(adrr);
	I2C_SendByte('c'); I2C_SendByte(cy5); I2C_SendByte(cy6); I2C_SendByte(cy7); I2C_SendByte(cy8);
	I2C_Stop();
	I2C_WaitTillStopWasSent();
	
}