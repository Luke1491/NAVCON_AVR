/*
 * I2CBase.h
 *
 * Created: 2017-05-26 19:15:52
 *  Author: LUKE
 ************Biblioteka do obs³ugi g³ównego wyœwietlacza (kurs  i prêdkoœæ)****************
 */ 

//*************************************************************************************
//******************************ADRES WYŒWIETLACZA*************************************
#define I2C_MAIN_DISPL_ADDR 190
//*************************************************************************************
//*************************************************************************************

#ifndef I2CBASE_H_
#define I2CBASE_H_
#include <stdint.h>
#include <util/twi.h>

#define I2CBUSCLOCK		50000UL

#define I2C_OK          0
#define I2C_STARTError	1
#define I2C_NoNACK		3
#define I2C_NoACK		4

extern uint8_t I2C_Error;
extern uint8_t cy1;
extern uint8_t cy2;
extern uint8_t cy3;
extern uint8_t cy4;
extern uint8_t cy5;
extern uint8_t cy6;
extern uint8_t cy7;
extern uint8_t cy8;

inline void I2C_SetError(uint8_t err) { I2C_Error=err;};

void I2C_Init();
void I2C_Start();
static inline void I2C_Stop() {TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);};	//Nadaj sygna³ STOP
void I2C_SetBusSpeed(uint16_t speed);	//Ustaw prêdkoœæ magistrali I2C (Prêdkoœæ wyniesie speed*100[Hz])
void I2C_SendAddr(uint8_t address);		//Wyœlij adres urz¹dzenia slave
void I2C_SendByte(uint8_t byte);		//Wyœlij bajt danych do slave
uint8_t I2C_ReceiveData_ACK();			//Odbierz bajt danych od slave, wysy³aj¹c ACK
uint8_t I2C_ReceiveData_NACK();			//Odbierz bajt danych od slave, nie wysy³aj ACK
static inline void I2C_WaitForComplete() {while (!(TWCR & _BV(TWINT)));};		//Zaczekaj na end bie¿¹cej operacji I2C
static inline void I2C_WaitTillStopWasSent() {while (TWCR & _BV(TWSTO));};	//Zaczekaj na end nadawania sygna³u STOP
void I2C_SendStartAndSelect(uint8_t addr);				//Wyœlij START i adres urz¹dzenia slave
void I2C_SendTxt(char *txt);
void I2C_StartSelectWait(uint8_t addr);
void wyswietl_led(int speed, int course);
void I2C_SendCourseAndSpeed(int course, int speed, uint8_t adrr);




#endif /* I2CBASE_H_ */