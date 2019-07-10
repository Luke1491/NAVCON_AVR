/*
 * MAX7219.h
 *
 * Created: 2017-05-26 18:52:22
 *  Author: LUKE
 */ 


#ifndef MAX7219_H_
#define MAX7219_H_





void MAX7219_Init(void);
void spiSendByte (char databyte);
void MAX7219_writeData(char data_register, char data);
void MAX7219_clearDisplay(void);
void MAX7219_displayNumber(volatile long number);
void MAX7219_SendCourseAndSpeed(uint16_t course, int16_t speed);//podawaæ argumenty pomno¿one przez 10 np 270.6 stopnii -->2706
void MAX7219_autopilotOFF(void); //wyœwietla kreski na wyœwietlaczu

#endif /* MAX7219_H_ */