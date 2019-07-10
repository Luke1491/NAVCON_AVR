/*
 * common.c
 *
 * Created: 2017-05-24 20:52:07
 * Modified: 11-04-2019
 *  Author: LUKE
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include "common.h"
#include "ws2812b.h"
#include "I2CBase.h"
#include "mpr121.h"
#include "ip_arp_udp_tcp.h"
#include "enc28j60.h"
#include "net.h"
#include "MAX7219.h"
#include <math.h>

#include <util/delay.h>


#define BUZZER_ENABLE PORTC|=(1<<PC1)
//########################################################
//###########DEKLARACJE ZMIENNYCH GLOBALNYCH##############
//########################################################
SHIPMODEL shipmodel={100,1,150,200};
OWNSHIP ownship;
SHIPPARAM shipparam = {0, 0, 0, 0, 0, 0, 0};
	
//##########SHIP MODEL 1############
	const OWNSHIP flashOwnship1 PROGMEM = 
	{
		70,100,14366666,53933333 , "257901000", "0","33", "5" , "8812532", "NAVA@@@", "NORWAY OFFSHORE@@@@@","1","95", "24", "10", "10", 7,17,13,50,"7.2", "GDANSK@@@@@@@@@@@@@@" 
	};
	const SHIPMODEL shipmodel1 PROGMEM = 
	{
		300,6,185,50
	};
	//############################################
	const OWNSHIP flashOwnship2 PROGMEM =
	{
		70,100,14366666,53933333 , "257721012", "0","33", "5" , "9127006", "NAVB@@@", "FERRY MAGICALLY@@@@@","1","224", "35", "20", "15", 7,17,13,50,"6.5", "GDANSK@@@@@@@@@@@@@@"
	};
	const SHIPMODEL shipmodel2 PROGMEM =
	{
		100,3,220,100
	};
	//##########################################################
	const OWNSHIP flashOwnship3 PROGMEM =
	{
		70,100,14366666,53933333 , "219383307", "0","33", "5" , "7490750", "NAVC@@@", "ELLIE MOONE@@@@@@@@@","1","397", "56", "40", "38", 7,17,13,50,"15", "GDANSK@@@@@@@@@@@@@@"
	};
	const SHIPMODEL shipmodel3 PROGMEM =
	{
		100,1,260,400
	};
//#############################################################

uint8_t touched;		//capacitive status
uint16_t prevTouched;	//user choice
uint8_t switchedToAuto;  
uint8_t switchedToManual;
//#############################################################
int32_t roseCourse;

uint32_t newRoseDiskCourse;
//##################################
uint8_t encoderNumberOfStepsPerSecond, encoderBiggerStepAllowed;

volatile uint16_t computingParametersTimer, 
				  steerReactionTimer, 
				  engineReactionTimer, 
				  speedSetInManualTimer, 
				  buzzerTimer, 
				  TouchTimer, 
				  roseDiskStepTimer;
//#########################################################################
//###################hardware timer every 10 ms############################
void timerInit(void) 
{
	TCCR0A = (1<<WGM01); //CTC timer mode (compare with OCR0A register)
	TCCR0B = (1<<CS00) | (1<<CS02); //prescaler 1024 --> timer increase own register by one in freq = 15625 Hz (for F_CPU=16 MHZ)
	TIMSK0 = (1<<OCIE0A); //enable interrupt after timer reach OCR0A
	OCR0A = 156; //interrupt every 10 ms
}

ISR(TIMER0_COMPA_vect)
{
	computingParametersTimer= computingParametersTimer + 1;
	steerReactionTimer= steerReactionTimer + 1;
	engineReactionTimer = engineReactionTimer + 1;
	speedSetInManualTimer = speedSetInManualTimer + 1;
	buzzerTimer = buzzerTimer + 1;
	TouchTimer = TouchTimer + 1;
	roseDiskStepTimer = roseDiskStepTimer + 1;
}
//#########################################################################
void NAVCON_CALIBRATION(void)
{
	for(uint8_t i = 0; i<(STEER_MAXPIX/2); i++)
	{
		steerLed[i] = rgbColors[1];
	}
	for(uint8_t i = (STEER_MAXPIX/2); i<STEER_MAXPIX; i++)
	{
		steerLed[i] = rgbColors[4];
	}
	for(uint8_t i = 0; i<(SPEED_MAXPIX/2); i++)
	{
		speedLed[i] = rgbColors[4];
	}
	for(uint8_t i = (SPEED_MAXPIX/2); i<SPEED_MAXPIX; i++)
	{
		speedLed[i] = rgbColors[1];
	}
	for(uint8_t i = 0; i<ROSE_DISK_MAXPIX; i++)
	{
		roseLed[i].r = 50;
		roseLed[i].b = 50;
		roseLed[i].g = 50;
	}
	ws2812SendDataPORTD((uint8_t *)speedLed, SPEED_MAXPIX * 3, PREDKOSC_PIN);
	ws2812SendDataPORTD((uint8_t *)steerLed, STEER_MAXPIX * 3, STER_PIN);
	ws2812SendDataPORTD((uint8_t *)roseLed, ROSE_DISK_MAXPIX * 3, TARCZA_PIN);
	uint8_t counter0 = 0, counter1=0;
	uint8_t Electrode0Register=0;
	uint8_t baselineEle0Register=0;
	do{
		Electrode0Register = MPR_read_reg(MPR_DEVICE_ADDRESS, MPR_R_TOUCHSTATUS_F);
		baselineEle0Register = MPR_read_reg(MPR_DEVICE_ADDRESS, MPR_R_BASELINE);
		baselineEle0Register= baselineEle0Register<<2;
		MAX7219_SendCourseAndSpeed(baselineEle0Register, Electrode0Register);
	    counter0++;
		if(counter0>60) counter0=1;
		if(counter0>0 && counter0<15) counter1 = 1;
		if(counter0>=15 && counter0<30) counter1 = 2;
		if(counter0>=30 && counter0<45) counter1 = 3;
		if(counter0>=45 && counter0<60) counter1 = 4;
		switch(counter1)
		{	
		case 1:
	{
		I2C_StartSelectWait(I2C_MAIN_DISPL_ADDR);
		I2C_SendByte('c'); I2C_SendByte(11); I2C_SendByte(12); I2C_SendByte(12); I2C_SendByte(11);
		I2C_Stop();
	} break;
	case 2:
	{
		I2C_StartSelectWait(I2C_MAIN_DISPL_ADDR);
		I2C_SendByte('c'); I2C_SendByte(11); I2C_SendByte(1); I2C_SendByte(10); I2C_SendByte(11);
		I2C_Stop();
	} break;
	case 3:
	{
		I2C_StartSelectWait(I2C_MAIN_DISPL_ADDR);
		I2C_SendByte('c'); I2C_SendByte(11); I2C_SendByte(13); I2C_SendByte(13); I2C_SendByte(11);
		I2C_Stop();
	} break;
	case 4:
	{
		I2C_StartSelectWait(I2C_MAIN_DISPL_ADDR);
		I2C_SendByte('c'); I2C_SendByte(11); I2C_SendByte(10); I2C_SendByte(14); I2C_SendByte(11);
		I2C_Stop();
	} break;
	default: {
		I2C_StartSelectWait(I2C_MAIN_DISPL_ADDR);
		I2C_SendByte('s'); I2C_SendByte(11); I2C_SendByte(11); I2C_SendByte(11); I2C_SendByte(11);
		I2C_Stop();
	} break;
	}//end switch
	
	moveRoseDiskByOneStep(1);
    _delay_ms(3);
	
  }while(((Electrode0Register-baselineEle0Register)>10) || (!(PINC & (1<<PC2))) );
  roseCourse = 3567;
   for(uint16_t i = 0; i<7; i++) //45 for console for Szczecin; 7 for console for supForNav
   {
	   moveRoseDiskByOneStep(1);
	   _delay_ms(8);
	   roseCourse++;
   }
   roseCourse = 0; //PO KOREKSCIE roseCourse = 0
   if(ownship.course)
   {
   for(uint16_t i = 0; i<ownship.course; i++)
  {
	 moveRoseDiskByOneStep(1);
	 _delay_ms(5); 
	 roseCourse++;
  }
   }//end if(ownship.course)
  motorCoilOff();
}

//###############COMPUTING SPPED AND COURSE FUNCTIONS###################
int8_t computeROT(int8_t currentSteerAngle)  //funkcja oblicza kurs co 1 sekundê --> ustawiæ odpowiedni timer programowy
{
	if(ownship.speed<=20) 
	{    
		if(shipparam.calculatedROT<0) shipparam.calculatedROT+=10;
		if(shipparam.calculatedROT>0) shipparam.calculatedROT-=10; 
	}
	else
	{
		//************COMPUTE GIVEN ROT USING CURRENT STEER ANGLE*****************
		int32_t givenROT;
		if(currentSteerAngle == 0) 
		{
			givenROT = 0;
		}
		else
		{
			givenROT = shipmodel.maxROT/35*currentSteerAngle;// steer angle negative -> givenRot negative; steer angle positive -> givenRot positive
		}
		//***************************************************************************
		//**************GET givenROT and compute ROT difference**********************
		int16_t ROTDifference;
		ROTDifference = givenROT - shipparam.calculatedROT;
		//*********************COMPUTE CURRENT ROT***********************************
		shipparam.calculatedROT += ROTDifference;
		return 0;
	}//end else of if(ownship.speed<=...)
	return 0;
}//end computeROT()


uint16_t computeCourse(void) //call every 1s
{
	//static int32_t tempCourse; 
	int32_t tempCourse = ownship.course*10;
	static uint8_t ifFirstCall = 0;
	if(!ifFirstCall) {tempCourse = ownship.course *10; ifFirstCall=1;}
	int32_t ROTDifference = 0;
	ROTDifference = shipparam.calculatedROT - shipparam.currentROT; 
		
	if(ROTDifference > 0)
	{
		shipparam.currentROT += shipmodel.dROT_po_dT;
	}
	else if(ROTDifference < 0)
	{
		shipparam.currentROT -= shipmodel.dROT_po_dT;
	}
	else //ROTDifference == 0
	{
		;
	}
	
		tempCourse = tempCourse + shipparam.currentROT; //precyzja 0,01 stopnia
		if(tempCourse>=36000) tempCourse = 0;
		if(tempCourse<0) tempCourse = 35999;
	
	ownship.course = (tempCourse/10);	
	return 0;
}//end computeCourse()

void computeShipParameters(void)
{
	if(computingParametersTimer >= 100) //compute course and rot every 1s
	{
		computingParametersTimer = 0; 
		//shipparam.requiredSteerPos = convertFrom10bitValueToRequiredSteerAngle(); //dla trybu manual
		computeROT(shipparam.currentSteerPos);
		computeCourse();
		computePosition();
		if(prevTouched == 2) displaySteerLed(shipparam.currentSteerPos, 4, 1);
		else displaySteerLed(shipparam.currentSteerPos, 8, 9);
		autopilotPID(shipparam.requiredCourse);
		//setRoseDiskCourse();
		ws2812SendDataPORTD((uint8_t *)roseLed, ROSE_DISK_MAXPIX * 3, TARCZA_PIN);
		//ws2812SendDataPORTD((uint8_t *)speedLed, SPEED_MAXPIX * 3, PREDKOSC_PIN);
		ws2812SendDataPORTD((uint8_t *)steerLed, STEER_MAXPIX * 3, STER_PIN);
		prepareAndSendOWNDatagram();
		//*************************************************
		encoderNumberOfStepsPerSecond = 0; //zmienna potrzebna do obs³ugi enkodera(szybsza inkrementacja po szybkim krêceniu)
		encoderBiggerStepAllowed = encoderBiggerStepAllowed - 1;
		if(encoderBiggerStepAllowed<=10) encoderBiggerStepAllowed=10;
		//*************************************************
	}
	if(steerReactionTimer >= 40) //every 40 == 0.4s call function inside if
	{
		steerReactionTimer = 0;
		
		moveSteer(shipparam.requiredSteerPos);
	}
	if(engineReactionTimer >= shipmodel.speedAccelerationTime)
	{
		engineReactionTimer = 0;
		simulateChangingOfSpeed();
	}
	if(roseDiskStepTimer>=4)//co 40 ms
	{
		roseDiskStepTimer=0;
		setRoseDiskCourse();
	}
}//end computeShipParameters()

//***********Steer functions88*****************
void steerInit(void) {
ADMUX |= (1<<REFS0); //reference voltage = VCC
//set ADC channel by set MUX bits in ADMUX register (all MUXx = 0 -->channel ADC0)
ADMUX &= ~((1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0)); //reset MUXx (set 0)
DIDR0 = (1<<ADC0D); //disable digital pin connected to adc0

}

uint16_t readSteerWheelAngleFromADC(void) {
	
	
	ADCSRA |= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); //ADC enabled - prescaler 128 (for F_CPU=16MHz --> 
	//f sampling = 125kHz) 
	//sampling start
	PRR &= ~(1<<PRADC);// reset power reduction ADC bit to awake (if sleeping) ADC
	ADCSRA |= (1<<ADSC); // set ADC start Conversion bit (bit is reset after ADC measurement by hardware)
	//wait until measurement in finished
	while(ADCSRA & (1<<ADSC));
	//after measurement finish when ADSC=0, return 16 bit value form ADCH and ADCL
	return ADC;// ten zapis***(ADCH<<8) | ADCL;***NIE DZIA£A --DLACZEGO???
}

void convertFrom10bitValueToRequiredSteerAngle(void)
{
	shipparam.requiredSteerPos = ((readSteerWheelAngleFromADC()*10)/146)-35; 
	//steer wheel inertion from +3 to  -3 degrees
	if(shipparam.requiredSteerPos<=3 && shipparam.requiredSteerPos>=-3) shipparam.requiredSteerPos = 0;
}

void displaySteerLed(int8_t steerAngle, uint8_t positive, uint8_t negative)
{
	
	for(uint8_t i=0; i<STEER_MAXPIX; i++)
	{
		steerLed[i] = rgbColors[5];
	}
	
	if(steerAngle>=-35 && steerAngle<-30) 
	{
		steerLed[3] = rgbColors[negative];
		steerLed[4] = rgbColors[negative];
		steerLed[5] = rgbColors[negative];
		steerLed[6] = rgbColors[negative];
		steerLed[7] = rgbColors[negative];
		steerLed[8] = rgbColors[negative];
		steerLed[9] = rgbColors[negative];
	}
	if(steerAngle>=-30 && steerAngle<-25)
	{
		steerLed[4] = rgbColors[negative];
		steerLed[5] = rgbColors[negative];
		steerLed[6] = rgbColors[negative];
		steerLed[7] = rgbColors[negative];
		steerLed[8] = rgbColors[negative];
		steerLed[9] = rgbColors[negative];
	}
	if(steerAngle>=-25 && steerAngle<-20)
	{
		steerLed[5] = rgbColors[negative];
		steerLed[6] = rgbColors[negative];
		steerLed[7] = rgbColors[negative];
		steerLed[8] = rgbColors[negative];
		steerLed[9] = rgbColors[negative];
	}
	if(steerAngle>=-20 && steerAngle<-15)
	{
		steerLed[6] = rgbColors[negative];
		steerLed[7] = rgbColors[negative];
		steerLed[8] = rgbColors[negative];
		steerLed[9] = rgbColors[negative];
	}
	if(steerAngle>=-15 && steerAngle<-10)
	{
		steerLed[7] = rgbColors[negative];
		steerLed[8] = rgbColors[negative];
		steerLed[9] = rgbColors[negative];
	}
	if(steerAngle>=-10 && steerAngle<-5)
	{
		steerLed[8] = rgbColors[negative];
		steerLed[9] = rgbColors[negative];
	}
	if(steerAngle>=-5 && steerAngle<0)
	{
		
		steerLed[9] = rgbColors[negative];
	}
	if(steerAngle>0 && steerAngle<=5)
	{
		steerLed[10] = rgbColors[positive];
	}
		
	if(steerAngle>5 && steerAngle<=10)
	{
		
		steerLed[10] = rgbColors[positive];
		steerLed[11] = rgbColors[positive];
	}
	if(steerAngle>10 && steerAngle<=15)
	{
		
		steerLed[10] = rgbColors[positive];
		steerLed[11] = rgbColors[positive];
		steerLed[12] = rgbColors[positive];
	}
	if(steerAngle>15 && steerAngle<=20)
	{
		
		steerLed[10] = rgbColors[positive];
		steerLed[11] = rgbColors[positive];
		steerLed[12] = rgbColors[positive];
		steerLed[13] = rgbColors[positive];
	}
	if(steerAngle>20 && steerAngle<=25)
	{
		
		steerLed[10] = rgbColors[positive];
		steerLed[11] = rgbColors[positive];
		steerLed[12] = rgbColors[positive];
		steerLed[13] = rgbColors[positive];
		steerLed[14] = rgbColors[positive];
	}
	if(steerAngle>25 && steerAngle<=30)
	{
		
		steerLed[10] = rgbColors[positive];
		steerLed[11] = rgbColors[positive];
		steerLed[12] = rgbColors[positive];
		steerLed[13] = rgbColors[positive];
		steerLed[14] = rgbColors[positive];
		steerLed[15] = rgbColors[positive];
	}
	if(steerAngle>30 && steerAngle<=35)
	{
		
		steerLed[10] = rgbColors[positive];
		steerLed[11] = rgbColors[positive];
		steerLed[12] = rgbColors[positive];
		steerLed[13] = rgbColors[positive];
		steerLed[14] = rgbColors[positive];
		steerLed[15] = rgbColors[positive];
		steerLed[16] = rgbColors[positive];
	}
}

void moveSteer(int8_t requiredSteerPos) //calling every x sec--depends on timer 2
{
	int8_t ROTDifference = 0;
	ROTDifference = requiredSteerPos - shipparam.currentSteerPos;
	if(ROTDifference>0) shipparam.currentSteerPos += 1;
	else if(ROTDifference == 0) ;
	else shipparam.currentSteerPos -= 1;
	
}

int8_t autopilotPID(int32_t givenCourse)
{
int32_t requiredSteerPos = 0;
uint8_t k = 3; 
uint8_t ki = 0; 
uint8_t kd = 0; //controller settings adjusted experimentally (when 0 -> regulator is off)
//auxiliary variables
int32_t deviation; //deviation regulacji (deviation between requred course and actual course)
int32_t p, i, d, r;
static int32_t deviation_p = 0; //deviation in previous call
static int32_t deviationSum = 0; //sum of past regulatory errors

//calculation of the error
deviation = givenCourse - ownship.course; 
if(deviation>1800) deviation = -(3600-deviation);
if(deviation<-1800) deviation = 3600-deviation;

//determining the proportional component(p)
p = k * deviation;
 
//determination of the integral component (i)
deviationSum = deviationSum + deviation;
i = ki * deviationSum;
if(i > 2100000000 ) i = 2100000000;
if(i < -2100000000) i = -2100000000;

//determination of the differentiating component
d = kd * (deviation - deviation_p);

deviation_p = deviation; //remember the momentary value of errors for future calculations

r = p + i + d; //calculate the course change

if((r>=-35 && r<-10) || (r>10 && r<=35)) requiredSteerPos = r/5;
else if(r>=-10 && r<=10) requiredSteerPos = r/2;
else requiredSteerPos = r/35;

if(requiredSteerPos > 35) requiredSteerPos = 35;
if(requiredSteerPos < -35) requiredSteerPos = -35;

shipparam.requiredSteerPos = requiredSteerPos;

return 0;	
}
//*********end of functions to operate the rudder******************
//*****************************************************************
//*********functions for speed control*****************************
void simulateChangingOfSpeed(void)
{
	int32_t speedDifference = 0;   					  			//speed and current speed difference
	speedDifference = shipparam.requiredSpeed - ownship.speed;  //speed difference in knots *10
	if(speedDifference>0)  ownship.speed += 1;					//increase current speed by 0,1kt
	else if(speedDifference == 0);
	else  ownship.speed -= 1;                                   //if rz¹danaSpeed < aktualnaSpeed --> decrease ownship.speed o 0,1kt
	
	if(ownship.speed >= shipmodel.maxSpeed) ownship.speed = shipmodel.maxSpeed;
	if(ownship.speed <= -shipmodel.maxSpeed) ownship.speed = -shipmodel.maxSpeed;
}

void wyswietlenieNastawyPredkosciLED(int16_t rzadanaPredkosc, uint8_t positive, uint8_t negative) //nastawa predkosci od -8 do +8
{
	if(rzadanaPredkosc>=shipmodel.maxSpeed) rzadanaPredkosc = shipmodel.maxSpeed; //machanizam apobieaj¹cy zwiêkszeniu predkosci ponad max speed
	int8_t procentPredkosc;
	procentPredkosc = (rzadanaPredkosc*100)/(shipmodel.maxSpeed); //procentowy stosunek predkosci rzadanej do predkosci maksymalnej
	for(uint8_t i=0; i<SPEED_MAXPIX; i++)
	{
		speedLed[i] = rgbColors[5];
	}
	
	if(procentPredkosc<=100 && procentPredkosc>88)
	{
		speedLed[2] = rgbColors[positive];
		speedLed[3] = rgbColors[positive];
		speedLed[4] = rgbColors[positive];
		speedLed[5] = rgbColors[positive];
		speedLed[6] = rgbColors[positive];
		speedLed[7] = rgbColors[positive];
		speedLed[8] = rgbColors[positive];
		speedLed[9] = rgbColors[positive];
		
	}
	if(procentPredkosc<=88 && procentPredkosc>75)
	{
		speedLed[3] = rgbColors[positive];
		speedLed[4] = rgbColors[positive];
		speedLed[5] = rgbColors[positive];
		speedLed[6] = rgbColors[positive];
		speedLed[7] = rgbColors[positive];
		speedLed[8] = rgbColors[positive];
		speedLed[9] = rgbColors[positive];
		
	}
	if(procentPredkosc<=75 && procentPredkosc>63)
	{
		speedLed[4] = rgbColors[positive];
		speedLed[5] = rgbColors[positive];
		speedLed[6] = rgbColors[positive];
		speedLed[7] = rgbColors[positive];
		speedLed[8] = rgbColors[positive];
		speedLed[9] = rgbColors[positive];
	}
	if(procentPredkosc<=63 && procentPredkosc>50)
	{
		speedLed[5] = rgbColors[positive];
		speedLed[6] = rgbColors[positive];
		speedLed[7] = rgbColors[positive];
		speedLed[8] = rgbColors[positive];
		speedLed[9] = rgbColors[positive];
	}
	if(procentPredkosc<=50 && procentPredkosc>38)
	{
		speedLed[6] = rgbColors[positive];
		speedLed[7] = rgbColors[positive];
		speedLed[8] = rgbColors[positive];
		speedLed[9] = rgbColors[positive];
	}
	if(procentPredkosc<=38 && procentPredkosc>25)
	{
		speedLed[7] = rgbColors[positive];
		speedLed[8] = rgbColors[positive];
		speedLed[9] = rgbColors[positive];
	}
	if(procentPredkosc<=25 && procentPredkosc>13)
	{
		speedLed[8] = rgbColors[positive];
		speedLed[9] = rgbColors[positive];
	}
	if(procentPredkosc<=13 && procentPredkosc>0)
	{
		speedLed[9] = rgbColors[positive];
	}
	
	if(procentPredkosc>=-100 && procentPredkosc<-88)
	{
		speedLed[10] = rgbColors[negative];
		speedLed[11] = rgbColors[negative];
		speedLed[12] = rgbColors[negative];
		speedLed[13] = rgbColors[negative];
		speedLed[14] = rgbColors[negative];
		speedLed[15] = rgbColors[negative];
		speedLed[16] = rgbColors[negative];
		speedLed[17] = rgbColors[negative];
	}
	if(procentPredkosc>=-88 && procentPredkosc<-75)
	{
		speedLed[10] = rgbColors[negative];
		speedLed[11] = rgbColors[negative];
		speedLed[12] = rgbColors[negative];
		speedLed[13] = rgbColors[negative];
		speedLed[14] = rgbColors[negative];
		speedLed[15] = rgbColors[negative];
		speedLed[16] = rgbColors[negative];
	}
	if(procentPredkosc>=-75 && procentPredkosc<-63)
	{
		speedLed[10] = rgbColors[negative];
		speedLed[11] = rgbColors[negative];
		speedLed[12] = rgbColors[negative];
		speedLed[13] = rgbColors[negative];
		speedLed[14] = rgbColors[negative];
		speedLed[15] = rgbColors[negative];
	}
	if(procentPredkosc>=-63 && procentPredkosc<-50)
	{
		speedLed[10] = rgbColors[negative];
		speedLed[11] = rgbColors[negative];
		speedLed[12] = rgbColors[negative];
		speedLed[13] = rgbColors[negative];
		speedLed[14] = rgbColors[negative];
	}
	if(procentPredkosc>=-50 && procentPredkosc<-38)
	{
		speedLed[10] = rgbColors[negative];
		speedLed[11] = rgbColors[negative];
		speedLed[12] = rgbColors[negative];
		speedLed[13] = rgbColors[negative];
	}
	if(procentPredkosc>=-38 && procentPredkosc<-25)
	{
		speedLed[10] = rgbColors[negative];
		speedLed[11] = rgbColors[negative];
		speedLed[12] = rgbColors[negative];
	}
	if(procentPredkosc>=-25 && procentPredkosc<-13)
	{
		speedLed[10] = rgbColors[negative];
		speedLed[11] = rgbColors[negative];
	}
	if(procentPredkosc>=-13 && procentPredkosc<0)
	{
		speedLed[10] = rgbColors[negative];
	}
}

/////////////////////////////////////////////////////////////////////
//************************OBLICZNIE POZYCJI**************************
void computePosition(void)
{
	int32_t distance=0; //dystans pokonany w ci¹gu ostatniej sekundy
	int32_t deltaLat, deltaLong;
	distance = (((int32_t)ownship.speed*1000)/36)/60; //dystans w milach morskich z precyzj¹ do 6 miejsca po przecinku: wynik0,000001 bêdzie w zmiennej jako 1
	double courseRadians = ((double)ownship.course/573);
	double latitudeRadians = ((double)ownship.posLat/57295779);
	//oblicznie szerokoœci
	deltaLat = distance * cos(courseRadians); // delta z precyzj¹ do 0,000001
	ownship.posLat = ownship.posLat + deltaLat;//nowa pozycja z dok³adnoœci¹ do 0,000001
	//obliczanie dlugoœci
	int32_t zboczeniaNawig = distance * sin(courseRadians);
	deltaLong = zboczeniaNawig/cos(latitudeRadians);
	ownship.posLong = ownship.posLong + deltaLong;
	//char str[20];
	//ltoa(deltaLat, str, 10);
	//send_udp(buf, str, strlen(str), 8095, farip[0], 10110);
}

////////////////////////////////////////////////////////////////////
//transoprot PC3
//buzzer PC2
//A4988 driver connected to PCF8574P connected to AtMega thru I2C (adress: 0x70)
//A4988 information:
/*
	TRYB PRACY
	MS1	MS2	MS3
	0	0	0	-- PE£NY KROK
	1	0	0	--1/2 KROKU
	0	1	0	--1/4 KROKU
	1	1	0	--1/8 KROKU
	1	1	1	--1/16 KROKU

	A4988		CONNECTED PCF8574P PIN		DESCRIPTION
	EN			P0					        1-wy³¹czone wyjœcie silnika  / 0-w³¹czone wyjœcie silnika
	MS1			P1
	MS2			P2 
	MS3			P3 
	RESET		P4							1-NORMALNE DZIA£ANIE / 0-RESET
	SLEEP		P5							1- normalna praca / 0-sleep
	STEP		P6							PRZEJŒCIE NA 1 SKOK O JEDEN STEP
	DIR			P7							1-obrót w prawo / 0-obrót w lewo
*/
#define MOTOREN				 0
#define MOTORMS1			 1
#define MOTORMS2			 2
#define MOTORMS3			 3
#define MOTORRESET			 4
#define MOTORSLEEP			 5
#define MOTORSTEP			 6
#define MOTORDIR			 7
#define MOTORMAINSETTING	((1 << MOTORRESET)|(1 << MOTORSLEEP))//pe³en krok
//tarcza kursów
void tarczaInit(void)
{
	I2C_StartSelectWait(0x70); //wyœlij bit startu i adres ekspandera tarczy
	I2C_SendByte(MOTORMAINSETTING | (1 << MOTOREN)); //1/16 step , normal operation , motor's coil off
	I2C_Stop();
	I2C_WaitTillStopWasSent();
}

uint16_t stopnieNaLED(uint16_t stopnie)
{
	stopnie=stopnie+360-(ownship.course/10);
	if(stopnie >= 360) stopnie=stopnie-360;
	if(stopnie < 0) return 0;
	uint16_t liczba_w_tablicy_LED = 0;
	uint16_t wynikModulo = 0;
	if(stopnie == 0 ) stopnie = 1;
	for(uint16_t i = 0; i<(stopnie*10); i++)
	{
		wynikModulo = i % 38;
		if(wynikModulo == 0) liczba_w_tablicy_LED = liczba_w_tablicy_LED + 1;
	}
	return liczba_w_tablicy_LED;
}

void czujnikTarczyInit(void)
{
	DDRC &= ~(1<<PC2); //PC3 jako wejœcie
	PORTC &= ~(1<<PC2); //bez podci¹dania do VCC (rezystor podci¹daj¹cy jest na p³ytce pcb)
}

uint8_t fullStep = 1; // 1 - full step (no movement required)   0- micro step in progress 
uint8_t tarczaDir = 0; // 0-left   1-right
void moveRoseDiskByOneStep(uint8_t dir) //1 step == 0,1 degree with gear
{
	I2C_StartSelectWait(0x70);
		if(dir) {
		I2C_SendByte(MOTORMAINSETTING | (1 << MOTORSTEP)); //make one step 
		I2C_SendByte(MOTORMAINSETTING); //prepare for another step
		 } //je¿eli dir jest wiêksza od 0 to krêæ w prawo
	else { 
		I2C_SendByte(MOTORMAINSETTING | (1<<MOTORDIR)); //make one step 
		I2C_SendByte(MOTORMAINSETTING | (1 << MOTORSTEP) |(1<<MOTORDIR)); //make one step
		I2C_SendByte((MOTORMAINSETTING) |(1<<MOTORDIR)); //prepare for another step
		 } //krêæ w lewo 
	I2C_Stop();
	I2C_WaitTillStopWasSent();
	fullStep = 1; //wykonano krok
}

void motorCoilOff(void)
{
	I2C_StartSelectWait(0x70);
	I2C_SendByte(MOTORMAINSETTING | (1<<MOTOREN)); //make motor coil off
	I2C_Stop();
	I2C_WaitTillStopWasSent();
}



void setRoseDiskCourse(void)
{
	if(fullStep)
	{
		motorCoilOff();//wy³¹cz cewkê  silnika
		fullStep = 0; //zezwolenie na w³¹czenie silnika w nastêpnym wywolaniu(za 100ms)
	}
	else
	{
	int16_t ROTDifference;
	newRoseDiskCourse = ownship.course;
	ROTDifference = newRoseDiskCourse - roseCourse;
	if(ROTDifference>1800) ROTDifference = -(3600-ROTDifference);
	if(ROTDifference<-1800) ROTDifference = (3600-ROTDifference);
	if(ROTDifference > 0) 
	{
		tarczaDir = 1;
		moveRoseDiskByOneStep(tarczaDir);
		roseCourse+=1;
		if(roseCourse > 3600) roseCourse = 0;
	}
	else if(ROTDifference < 0)
	{
		tarczaDir = 0;
		moveRoseDiskByOneStep(tarczaDir);
		roseCourse -=1;
		if(roseCourse < 0) roseCourse = 3600;
	}
	else ; //gdy ro¿nica == 0 to nic nie rób
	}//end else from main if
}

void kalibracjaTarczy(void)
{
	while(!(PINC & (1<<PC2)))
	{
		moveRoseDiskByOneStep(1);
	}//end pêtli while
	roseCourse = 0;
}

//*******************BUZZER*********************
void buzzerInit(void)
{
	DDRC |= (1<<PC1); //PC1 jako wyjœcie
	PORTC &= ~(1<<PC1); //stan niski - buzzer nie pracuje
}

/////////////////////////////////////////////////////////////////////////////////////
//obs³uga enkodera
/////////////////////////////////////////////////////////////////////////////////////
#define ENC1DDR DDRD
#define ENC2DDR DDRD
#define ENC1PORT PORTD
#define ENC2PORT PORTD
#define ENC1PIN PIND
#define ENC2PIN PIND
#define ENC1SIGN1 (1<<PD2)
#define ENC1SIGN2 (1<<PD3)
#define ENC2SIGN1 (1<<PD1)
#define ENC2SIGN2 (1<<PD0)
//timer do obs³ugi enkoderów
void Timer2Init(void)
{
	TCCR2B=_BV(CS01);	//Preskaler CLKIO/8
	TIMSK2|=_BV(TOIE2);	//Odblokuj przerwanie nadmiaru timera 0
}

ISR(TIMER2_OVF_vect)
{
	ReadEncoder1();    //Odczytaj stan klawiszy
	ReadEncoder2();    //Odczytaj stan klawiszy
}

//*************
void enkoderyInit(void)
{
	ENC1DDR &= ~(ENC1SIGN1 | ENC1SIGN2); //make PD0 and PD1 as input
	ENC2DDR &= ~(ENC2SIGN1 | ENC2SIGN2); //make PD2 and PD3 as input
	ENC1PORT |= ENC1SIGN1 | ENC1SIGN2; //pull on PD0 AND PD1
	ENC2PORT |= ENC2SIGN1 | ENC2SIGN2; //pull on PD2 AND PD3
}

 int8_t enc_delta1;
 int8_t enc_delta2;

void ReadEncoder1()
{
	static int8_t last;
	static uint8_t laststate;
	static uint8_t counters[2];	//Tablica zawieraj¹ca liczniki
	int8_t newpos, diff;
	
	uint8_t state=ENC1PIN;
	if(((state^laststate) & ENC1SIGN1) && (counters[0]==0))
	{
		counters[0]=200;
		laststate&=(~ENC1SIGN1);
		laststate|=(state & ENC1SIGN1);
	}

	if(((state^laststate) & ENC1SIGN2) && (counters[1]==0))
	{
		counters[1]=200;
		laststate&=(~ENC1SIGN2);
		laststate|=(state & ENC1SIGN2);
	}
	
	for(uint8_t c=0;c<sizeof(counters)/sizeof(counters[0]);c++)
	if(counters[c])	counters[c]--;

	newpos=0;
	if((ENC1PIN & ENC1SIGN1)==0) newpos=3;
	if((ENC1PIN & ENC1SIGN2)==0) newpos^=1;	// konwersja kodu Graya na binarny
	diff=last-newpos;
	if(diff & 1)
	{				// bit 0 = krok
		last=newpos;
		enc_delta1+=(diff & 2)-1;	//bit 1 - kierunek
	}
}

void ReadEncoder2()
{
	static int8_t last;
	static uint8_t laststate;
	static uint8_t counters[2];	//Tablica zawieraj¹ca liczniki
	int8_t newpos, diff;
	
	uint8_t state=ENC2PIN;
	if(((state^laststate) & ENC2SIGN1) && (counters[0]==0))
	{
		counters[0]=200;
		laststate&=(~ENC2SIGN1);
		laststate|=(state & ENC2SIGN1);
	}

	if(((state^laststate) & ENC2SIGN2) && (counters[1]==0))
	{
		counters[1]=200;
		laststate&=(~ENC2SIGN2);
		laststate|=(state & ENC2SIGN2);
	}
	
	for(uint8_t c=0;c<sizeof(counters)/sizeof(counters[0]);c++)
	if(counters[c])	counters[c]--;

	newpos=0;
	if((ENC2PIN & ENC2SIGN1)==0) newpos=3;
	if((ENC2PIN & ENC2SIGN2)==0) newpos^=1;	// konwersja kodu Graya na binarny
	diff=last-newpos;
	if(diff & 1)
	{				// bit 0 = krok
		last=newpos;
		enc_delta2+=(diff & 2)-1;	//bit 1 - kierunek
	}
}
int8_t Read1StepEncoder(uint8_t numerEnkodera)
{
	int8_t val;
	if(numerEnkodera == 1) 
	{
		ReadEncoder1();
	    val=enc_delta1;
		enc_delta1=0;
	}
	else 
	{
		ReadEncoder2();
		val=enc_delta2;
		enc_delta2=0;	
	}
	
	return val;
}

int8_t Read2StepEncoder(uint8_t numerEnkodera)
{
	int8_t val;
	if(numerEnkodera == 1) 
	{
		ReadEncoder1();
		val=enc_delta1;
		enc_delta1=val & 1;
	}
	
	else 
	{
		ReadEncoder2();
		val=enc_delta2;
		enc_delta2=val & 1;
	}
	return val>>1;
}

int8_t Read4StepEncoder(uint8_t numerEnkodera)
{
	int8_t val;
	if(numerEnkodera == 1)
	{
		ReadEncoder1();
		val=enc_delta1;
		enc_delta1=val & 3;
	}
	else
	{
	ReadEncoder2();
	val=enc_delta2;
	enc_delta2=val & 3;	
	}
	return val>>2;
}


void uaktualnijKursAuto(void)
{
	static uint16_t lastCourse=0;
	int8_t ROTDifference = shipparam.requiredCourse-lastCourse;
	if(encoderNumberOfStepsPerSecond == 0) lastCourse=shipparam.requiredCourse;
	if( ROTDifference >= 15 || ROTDifference <= -15 ) encoderBiggerStepAllowed = 11;
	if(encoderBiggerStepAllowed>10) shipparam.requiredCourse += (Read4StepEncoder(1) * 10);
	else shipparam.requiredCourse += Read4StepEncoder(1);
	if(shipparam.requiredCourse>=3600) shipparam.requiredCourse = 0;
	if(shipparam.requiredCourse<0) shipparam.requiredCourse = 3599;
	encoderNumberOfStepsPerSecond=1;
}

void uaktualnijPredkoscAuto(void)
{
	
	shipparam.requiredSpeed += Read4StepEncoder(2); //odczytaj stan enkodera2 i dodaj do rzadanej predkosci
	if(shipparam.requiredSpeed >= shipmodel.maxSpeed) shipparam.requiredSpeed = shipmodel.maxSpeed;
	if(shipparam.requiredSpeed <= -shipmodel.maxSpeed) shipparam.requiredSpeed = -shipmodel.maxSpeed;
}

uint16_t odczytajPrzyciskiGlowne(void)
{
	uint16_t touched;
	static uint16_t pretouch = 1;
	if(TouchTimer >= 100)
	{
		touched = MPR_read_reg16bit(MPR_DEVICE_ADDRESS, MPR_R_TOUCHSTATUS);
	
	if(touched & 0x03) 
	{
		TouchTimer = 0; //nastêpne odczytanie przycisku za 1s
		pretouch = touched;
		buzzerTimer = 0; //zacznij odmerzaæ czas do wy³¹czenia buzzera
		BUZZER_ENABLE;      //w³¹cz buzzer
		return touched;
	}
	if(!(touched & 0x03))
	{
		TouchTimer = 90; //nastêpne odczytanie przycisku za 0,1s
	}
	}//end glownego if
	else
	{
		dlugoscDzwiekuBuzzer(3);	
	}
	return pretouch;
}

void dlugoscDzwiekuBuzzer(uint8_t czas)
{
	if(buzzerTimer >= czas) PORTC &= ~(1<<PC1); //wy³¹cz buzzer po (czas*10) ms
}

uint16_t odczytajPrzyciskiSterowania(void)
{
	uint16_t touchedSterowania;
	if(speedSetInManualTimer >= 100)
	{
		touchedSterowania = MPR_read_reg16bit(MPR_DEVICE_ADDRESS, MPR_R_TOUCHSTATUS);
		
		if(touchedSterowania & 0x0C) //odczytaj 2 starsze bity w 1 tetradzie
		{
			speedSetInManualTimer = 50; //nastêpne odczytanie przycisku za 0,5s
			buzzerTimer = 0; //zacznij odmerzaæ czas do wy³¹czenia buzzera
			BUZZER_ENABLE;      //w³¹cz buzzer
			return touchedSterowania;
		}
		if(!(touchedSterowania & 0x0C))
		{
			speedSetInManualTimer = 90; //nastêpne odczytanie przycisku za 0,1s
		}
	}//end glownego if
	else
	{
		dlugoscDzwiekuBuzzer(3);
	}
	return 0;
}

//##############################################################################################################
//#############################################ETHERNET FUNCTION################################################
//##############################################################################################################
// ustalamy adres MAC
uint8_t mymac[6] = {'N','A','V','D','E','C'};
// ustalamy adres IP urz¹dzenia
uint8_t myip[4] = {169,254,131,153};

// ustalamy porty UDP z jakich bêdziemy korzystaæ
// mo¿e ich byæ dowolna iloœæ
uint16_t myport[] = {8095,24478};
// ustalamy adres IP bramy domyœlnej w sieci LAN
//static uint8_t gwip[4] = {169,254,255,255};

// ustalamy wielkoœæ bufora dla ramek TCP/UDP

uint8_t buf[BUFFER_SIZE+1];

// wskaŸnik do funkcji zwrotnej w³asnego zdarzenia UDP_EVENT()
void (*mk_udp_event_callback)(uint8_t *peer_ip, uint16_t port,
uint16_t datapos, uint16_t len);

// funkcja do rejestracji funkcji zwrotnej w zdarzeniu UDP_EVENT()
void register_udp_event_callback(void (*callback)(uint8_t *peer_ip,
uint16_t port, uint16_t datapos, uint16_t len))
{
	mk_udp_event_callback = callback;
}


//---------------------------------- deklaracje zmiennych globalnych -------------------------------
// indeksy adresów IP w postaci przyjaznych nazw
enum ip_names {ip_pc, ip_sterownik1};
// adresy IP sterowników z którymi bêdziemy siê komunikowaæ
// za pomoc¹ protoko³u UDP  ----> pierwszy element tablicy to IP twojego komputera
// drugi element to IP np innego uk³adu ATB
uint8_t farip[2][4] = { {169,254,131,151}, {192,168,0,180} };
uint8_t PC_IP[4] = {169,254,131,151};



// separator tokenów we w³asnych ramkach przesy³anych przez UDP
char sep[] = ","; //w komunikatach z navdeca s¹ przecinki jako separatory tokenów natomiast kropki rozdzielaj¹ czêœc ca³kowit¹ od u³amkowej

//_______________________________________________________________________________________________________________
//*****************funkcje do obs³ugi komend**********************
//navconCommand $ROZETA
void parseROZETA(char *buffer)
{
	//uint8_t przygotowanieDoTarczyZielonej;
	char *ethernetPtr,*wsk1;
	uint16_t liczba_przedzialow, nr_przedzialu, wart1, wart2;
	uint16_t wart1LED, wart2LED;
	uint32_t ulamekWart1, ulamekWart2;
	char str[25];
	ethernetPtr = strtok_r(NULL, sep, &buffer); //wy³uskujemy ilosc przedzialow
	liczba_przedzialow = atol(ethernetPtr);
	ethernetPtr = strtok_r(NULL, sep, &buffer); //nr przedzialu
	nr_przedzialu = atol(ethernetPtr);
	ethernetPtr = strtok_r(NULL, sep, &buffer); //pierwsza wartosc
	//-----------------------
	strcpy(str, ethernetPtr);
	wsk1 = strtok(str, ".");
	wart1 = atol(wsk1);
	wsk1 = strtok(NULL, ",");
	ulamekWart1 = atol(wsk1);
	//-----------------------
	ethernetPtr = strtok_r(NULL,sep, &buffer);//druga wartosc
	//----------------------
	strcpy(str, ethernetPtr);
	wsk1 = strtok(str, ".");
	wart2 = atol(wsk1);
	wsk1 = strtok(NULL, ",");
	ulamekWart2 = atol(wsk1);
	//----------------------
	ethernetPtr = strtok_r(NULL, sep, &buffer);//ethernetPtr ->NULL
	
	wart1LED = stopnieNaLED(wart1);
	wart2LED = stopnieNaLED(wart2);
	if( nr_przedzialu == 1) //gdy przyjdzie pierwszy przedzail to wyzeruj tarczê nadaj kolor czerwony
	{
		for(uint8_t j = 0; j<ROSE_DISK_MAXPIX; j++)
		{
			roseLed[j] = rgbColors[1];  //wstaw na ca³ej tarczy kolor czerwony
		}
	}
	
	if(wart1LED>wart2LED)
	{//zamiana zmiennych wartosciami
		uint16_t zmiennaPomocnicza;
		zmiennaPomocnicza = wart1LED;
		wart1LED = wart2LED; 
		wart2LED = zmiennaPomocnicza;
		for(uint8_t i = 0; i<wart1LED; i++)
		{
			roseLed[i] = rgbColors[3];//yellow
		}
		for(uint8_t i = wart2LED; i<ROSE_DISK_MAXPIX; i++)
		{
			roseLed[i] = rgbColors[3];//yellow
		}
	}
	else
	{
		
	if(liczba_przedzialow==1 && nr_przedzialu==1 && wart1==0 && wart2==360)
	{
		for(uint8_t i = 0; i<ROSE_DISK_MAXPIX; i++)
		{
			roseLed[i] = rgbColors[8];//kolor zielony
		}
	}
	
	for(uint8_t i = wart1LED; i<wart2LED; i++)
	{
		roseLed[i] = rgbColors[3];
	}//end for
	}//end main else 
	//MAX7219_SendCourseAndSpeed(wart1LED, wart2LED);//wyœwietlenie testowe obliczonych wartoœci(do celów sprawdzaj¹cych - normalnie pozostawiæ zakomentarzowanym)
}

//SHIPMODEL command
void parseSHIPMODEL(char *buffer)
{
	char *ethernetPtr;
	uint8_t numerModelu;
	ethernetPtr = strtok_r(NULL, sep, &buffer); //wy³uskujemy numer modelu statku
	numerModelu = atoi(ethernetPtr);
	ethernetPtr = strtok_r(NULL, sep, &buffer); //ethernetPtr ->NULL
	switch(numerModelu)
	{
		case 1: {
			//#######PRZYPISANIE WARTOSCI STATKU Z FLASHA#################
			memcpy_P(&ownship, &flashOwnship1, sizeof(ownship));
			//przypisanie wartoœci okreœlaj¹cych zachowanie statku
			memcpy_P(&shipmodel, &shipmodel1, sizeof(shipmodel));
		} break;
		case 2: {
			//#######PRZYPISANIE WARTOSCI STATKU Z FLASHA#################
			memcpy_P(&ownship, &flashOwnship2, sizeof(ownship));
			//przypisanie wartoœci okreœlaj¹cych zachowanie statku
			memcpy_P(&shipmodel, &shipmodel2, sizeof(shipmodel));
		} break;
		case 3: {
			//#######PRZYPISANIE WARTOSCI STATKU Z FLASHA#################
			memcpy_P(&ownship, &flashOwnship3, sizeof(ownship));
			//przypisanie wartoœci okreœlaj¹cych zachowanie statku
			memcpy_P(&shipmodel, &shipmodel3, sizeof(shipmodel));
		} break;
		default: {
			//#######PRZYPISANIE WARTOSCI STATKU Z FLASHA#################
			memcpy_P(&ownship, &flashOwnship1, sizeof(ownship));
			//przypisanie wartoœci okreœlaj¹cych zachowanie statku
			memcpy_P(&shipmodel, &shipmodel1, sizeof(shipmodel));
		} break;
	}//end switch
	shipparam.currentROT=0;						//accurancy 0,01 degree ex currentROT=10 --> ROT=0,1 degree
	shipparam.calculatedROT=0;				    // ROT direction, only for first call
	shipparam.currentSteerPos=0;				//current steer position
	shipparam.requiredSpeed=ownship.speed;      //required speed 
	shipparam.speedSetOnConsole=ownship.speed;  //from -8 to +8, speed setting changed by cpacitive buttons
	shipparam.requiredCourse=ownship.course;
	shipparam.requiredSteerPos=0;
	
	int16_t roseDifferenceForRoseReset = roseCourse - 1800;
	uint16_t numOfStepsToReset;
	if(roseDifferenceForRoseReset>0) 
	{
		numOfStepsToReset = 3600-roseCourse+ownship.course; 
	 for(uint16_t i = 0; i<numOfStepsToReset; i++)
	 {
		 moveRoseDiskByOneStep(1);
		 _delay_ms(5);
		 roseCourse++;
		 if(roseCourse > 3600) roseCourse = 0;
	 }
	}
	if(roseDifferenceForRoseReset<0)
	{
		numOfStepsToReset = roseCourse-ownship.course;
		for(uint16_t i = 0; i<numOfStepsToReset; i++)
		{
			moveRoseDiskByOneStep(0);
			_delay_ms(5);
			roseCourse--;
			if(roseCourse < 0) roseCourse = 3599;
		}
	}
 motorCoilOff();
	
}
/******* w³asna funkcja u¿ytkownika w której mo¿emy reagowaæ na ramkê UDP ***********/
/****/
void udp_event_callback(uint8_t *peer_ip, uint16_t port, uint16_t datapos, uint16_t len) {
	//uint8_t i=0;
    char str[40];

    // pointer for strtok_r function
    char *ethernetPtr, *rest;
	uint8_t navconCommand;


    // jeœli ramka przysz³a na drugi numer portu z listy myport[]
    if(port == myport[1]) {
    	// odpowiadamy na zapytanie UDP po ten sam port
    	strcpy(str,"Ramka odpowiedzi UDP");
    	make_udp_reply_from_request(buf,str,strlen(str), port);

        // wysy³amy now¹ ramkê na dowolny inny port
    	strcpy((char*)&buf[datapos],"Nowa ramka UDP!");
        send_udp_prepare(buf, 1200, farip[ip_pc], 43500);
        send_udp_transmit(buf,strlen(str));
    }

    // jeœli ramka przysz³a na pierwszy numer portu z listy myport[]
    if(port == myport[0]) {
    	// sprawdzamy czy zawiera dane
    	ethernetPtr = strtok_r((char*)&buf[datapos + 1], sep, &rest); //buf[datapos + 1] ¿eby omin¹æ znak $ który wywala b³¹d w kompilacji
    	if( ethernetPtr ) {
    		// jeœli zawiera dane to sprawdzamy jaka navconCommand
			char charkom [10];
			strcpy(charkom, ethernetPtr);
			
			if(!(strcmp(charkom, "ROZETA"))) navconCommand = 0;
			else if(!(strcmp(charkom, "SHIPMODEL"))) navconCommand = 1;
			else if(!(strcmp(charkom, "set_lcd"))) navconCommand = 2;
			else navconCommand = 10;
    		switch( navconCommand ) {
    		// i w zale¿noœci od komendy wywo³ujemy odpowieni¹
    		// funkcjê parsuj¹c¹ z tabeli wskaŸników do funkcji parsuj¹cych
    		// przekazujemy jednoczeœnie wskaŸnik do pozosta³ej czêœci
    		// ³añcucha z danymi, który funkcje bêd¹ parsowaæ we w³asnym zakresie
    		case 0: parseROZETA(rest); break;
    		case 1: parseSHIPMODEL(rest); break;
    		case 2: ; break;
			case 10: ; break;
    		}
			
    	}
    }
}


/********** zdarzenie UDP EVENT ***********************/
void UDP_EVENT(uint16_t *port) {
	// zmienne tymczasowe (automatyczne)
    uint16_t plen;//, dat_p;
    uint16_t dport;
    uint8_t is_my_port=0;
    uint8_t i=0;


    uint8_t udp_data_len=0;


    // sprawdzamy czy istnieje nowy odebrany pakiet
    plen = enc28j60PacketReceive(BUFFER_SIZE, buf);
    // obs³uga ni¿szych warstw stosu TCP jak ICMP, ARP itp
    // m.inn t¹ drog¹ obs³ugiwane s¹ zewnêtrzne PING'i
    // w tym miejscu sprawdzane jest od razu czy odczytana ramka
    // jest posiada typ jaki obs³ugiwany jest przez ten stos TCP
    // oraz czy jest ona zaadresowana do nas (w numeru IP)
    packetloop_icmp_tcp(buf,plen);

    // jeœli d³ugoœæ odebranej ramki jest wiêksza ni¿ 0
    // to oznacza, ¿e mamy do czynienia z prawid³owo odebran¹
    // ramk¹ wraz z prawid³ow¹ sum¹ kontroln¹
    // sprawdzamy tak¿e, czy na pewno mamy do czynienia z ramk¹ UDP a nie np TCP
    if( plen && buf[IP_PROTO_P]==IP_PROTO_UDP_V ) {

    	// sprawdzamy do jakiego nr portu kierowana jest ta ramka UDP
    	dport = (buf[UDP_DST_PORT_H_P]<<8) | buf[UDP_DST_PORT_L_P];

    	// sprawdzamy w pêtli czy jest to ramka przeznaczona do
    	// portu nas³uchowego, które obs³uguje nasze urz¹dzenie
    	// listê dowolnej iloœci portów definiujemy w tablicy myport[]
    	// która przekazywana jest jako parametr do zdarzenia/funkcji UDP_EVENT()
    	do {
    		if( (is_my_port = (port[i++] == dport)) ) break;
    	} while (i<sizeof(port));

    	// sprawdzamy czy informacja przysz³a na obs³ugiwany przez nas port UDP
    	if ( is_my_port ){
			udp_data_len=buf[UDP_LEN_L_P]-UDP_HEADER_LEN;

			// jeœli wszystko siê zgadza to sprawdzamy, czy zarejestrowana jest
			// funkcja zwrotna przez u¿ytkownika. Jeœli nie jest zarejestrowana
			// to nie wykonana siê ¿adna akcja poza powy¿ej obs³ug¹
			// ni¿szych warstw stosu TCP (np PING czy ARP)
			// Jeœli jest zarejestrowana to zostanie wywo³ana z odpowiednimi parametrami
			// 1. podany bêdzie adres IP sk¹d nadesz³a ramka
			// 2. podany bêdzie port na jaki zosta³a do nas skierowana ramka
			// 3. indeks do bufora ca³ej ramki TCP, wskazuj¹cy na pocz¹tek danych w ramce UDP
			// 4. iloœæ danych w bajtach przes³anych w ramce UDP
			if(mk_udp_event_callback) (*mk_udp_event_callback)(&(buf[IP_SRC_P]), dport, UDP_DATA_P, udp_data_len);
        }
    }
}

void ping_callback(uint8_t *ip){
	//response from ping request
}

void prepareAndSendOWNDatagram(void)
{
	//przygotuj ramkê UDP
	char str [150] = {"$OWN"};
	char str1[21]; //tablica do przechowaywania reprezentacji wyliczonych wartoœci w postaci tekstu (jednorazowa przetrzymujemy 1 oblizon¹ wartoœæ)
	int32_t wartoscDoWyswUDP;  //bêd¹ tu wpisywane po kolei wszystkie wartoœci obliczone przez symulator
	strcat(str, ",0,");                                 //1. pozycja zintegrowana
	strcat(str, ownship.MMSI);							//2. MMSI
	strcat(str, ",");
	strcat(str, ownship.NAVSTATUS);						//3. status nawigacyjny
	strcat(str, ",");
	wartoscDoWyswUDP = shipparam.currentROT*6/10;
	itoa(wartoscDoWyswUDP, str1, 10);
	strcat(str, str1 );									//4. ROT - czeœæ ca³kowita
	strcat(str, ".");								
	wartoscDoWyswUDP = (shipparam.currentROT%100)*6/10;
	itoa(wartoscDoWyswUDP, str1, 10);
	strcat(str, str1 );									//4. ROT - czeœæ u³amkowa
	strcat(str, ",");									
	wartoscDoWyswUDP = ownship.speed/10;
	if(ownship.speed<0){strcat(str, "-"); wartoscDoWyswUDP = -wartoscDoWyswUDP;}//umozliwia interpretacje przez navdec				
	itoa(wartoscDoWyswUDP, str1, 10);
	strcat(str, str1 );									//5. SOG - czêœæ ca³kowita
	strcat(str, ".");	
	wartoscDoWyswUDP = ownship.speed%10;
	if(ownship.speed<0){ wartoscDoWyswUDP = -wartoscDoWyswUDP;}				
	itoa(wartoscDoWyswUDP, str1, 10);
	strcat(str, str1 );									//5. SOG - czêœæ u³amkowa
	strcat(str, ",");	
	wartoscDoWyswUDP = ownship.posLong/1000000;
	if(ownship.posLong<0){strcat(str, "-"); wartoscDoWyswUDP = -wartoscDoWyswUDP;}//umozliwia interpretacje przez navdec
	itoa(wartoscDoWyswUDP, str1, 10);
	strcat(str, str1 );									//6. LONG - czêœæ ca³kowita
	strcat(str, ".");
	wartoscDoWyswUDP = ownship.posLong%1000000;
	if(ownship.posLong<0){wartoscDoWyswUDP = -wartoscDoWyswUDP;}//umozliwia interpretacje przez navdec
	//zabezpieczenie przed ucinaniem poprzedzj¹cych zer po przecinku
	if(wartoscDoWyswUDP<100000 && wartoscDoWyswUDP>=10000)	    strcat(str, "0");
	else if(wartoscDoWyswUDP<10000 && wartoscDoWyswUDP>=1000)	strcat(str, "00");
	else if(wartoscDoWyswUDP<1000 && wartoscDoWyswUDP>=100)		strcat(str, "000");
	else if(wartoscDoWyswUDP<100 && wartoscDoWyswUDP>=10)		strcat(str, "0000");
	else if(wartoscDoWyswUDP<10 && wartoscDoWyswUDP>=1)			strcat(str, "00000");
	
	ltoa(wartoscDoWyswUDP, str1, 10);
	strcat(str, str1 );									//6. LONG- czêœæ u³amkowa
	strcat(str, ",");
	wartoscDoWyswUDP = ownship.posLat/1000000;
	if(ownship.posLat<0){strcat(str, "-"); wartoscDoWyswUDP = -wartoscDoWyswUDP;}//umozliwia interpretacje przez navdec
	itoa(wartoscDoWyswUDP, str1, 10);
	strcat(str, str1 );									//7. LAT - czêœæ ca³kowita
	strcat(str, ".");
	wartoscDoWyswUDP = ownship.posLat%1000000;
	if(ownship.posLat<0){wartoscDoWyswUDP = -wartoscDoWyswUDP;}//umozliwia interpretacje przez navdec
	//zabezpieczenie przed ucinaniem poprzedzj¹cych zer po przecinku
	if(wartoscDoWyswUDP<100000 && wartoscDoWyswUDP>=10000)		strcat(str, "0");
	else if(wartoscDoWyswUDP<10000 && wartoscDoWyswUDP>=1000)	strcat(str, "00");
	else if(wartoscDoWyswUDP<1000 && wartoscDoWyswUDP>=100)		strcat(str, "000");
	else if(wartoscDoWyswUDP<100 && wartoscDoWyswUDP>=10)		strcat(str, "0000");
	else if(wartoscDoWyswUDP<10 && wartoscDoWyswUDP>=1)			strcat(str, "00000");
	ltoa(wartoscDoWyswUDP, str1, 10);
	strcat(str, str1 );									//7. LAT- czêœæ u³amkowa
	strcat(str, ",");
	wartoscDoWyswUDP = ownship.course/10;
	itoa(wartoscDoWyswUDP, str1, 10);
	strcat(str, str1 );									//8. COG - czeœæ ca³kowita
	strcat(str, ".");
	wartoscDoWyswUDP = ownship.course%10;
	itoa(wartoscDoWyswUDP, str1, 10);
	strcat(str, str1 );									//8. COG - czeœæ u³amkowa
	strcat(str, ",");
	wartoscDoWyswUDP = ownship.course/10;
	itoa(wartoscDoWyswUDP, str1, 10);
	strcat(str, str1 );									//9. HEADING - czeœæ ca³kowita (TAKI JAK COG)
	strcat(str, ".");
	wartoscDoWyswUDP = ownship.course%10;
	itoa(wartoscDoWyswUDP, str1, 10);
	strcat(str, str1 );									//9. HEADING - czeœæ u³amkowa (TAKI JAK COG)
	strcat(str, ",");
	strcat(str, "0.0,0.0," );							//10 / 11 POZYCJA WSP KARTEZJAÑSKIE 
	strcat(str, ownship.PAS );							//12. PAS
	strcat(str, ",");
	strcat(str, ownship.TYP_COMMUNIKATU );				//13. TYP KOMUNIKATU
	strcat(str, ",");
	strcat(str, ownship.IMO_NUMBER );					//14. IMO
	strcat(str, ",");
	strcat(str, ownship.CALL_SIGN );					//15. CALLSIGN
	strcat(str, ",");
	strcat(str, ownship.SHIP_NAME);						//16. SHIP NAME
	strcat(str, ",");
	strcat(str, ownship.TYPE_OF_SHIP );					//17. TYP STATKU
	strcat(str, ",");
	strcat(str, ownship.DIM_A );						//18. DIMA
	strcat(str, ",");
	strcat(str, ownship.DIM_B );						//19. DIMB
	strcat(str, ",");
	strcat(str, ownship.DIM_C );						//20. DIMC
	strcat(str, ",");
	strcat(str, ownship.DIM_D );						//21. DIMD
	strcat(str, ",1,");									//22. PRZECINEK + TYP URZ¥DZENIA
	wartoscDoWyswUDP = ownship.ETA_month;
	itoa(wartoscDoWyswUDP, str1, 10);
	strcat(str, str1 );									//23. ETA MONTH
	strcat(str, ",");
	wartoscDoWyswUDP = ownship.ETA_day;
	itoa(wartoscDoWyswUDP, str1, 10);
	strcat(str, str1 );									//24. ETA DAY
	strcat(str, ",");
	wartoscDoWyswUDP = ownship.ETA_hour;
	itoa(wartoscDoWyswUDP, str1, 10);
	strcat(str, str1 );									//25. ETA HOUR
	strcat(str, ",");
	wartoscDoWyswUDP = ownship.ETA_minute;
	itoa(wartoscDoWyswUDP, str1, 10);
	strcat(str, str1 );									//26. ETA MINUTE
	strcat(str, ",");
	strcat(str, ownship.DRAUGHT);						//27. DRAUGHT
	strcat(str, ",");
	strcat(str, ownship.DESTINATION);					//28. DESTINATION	
	strcat(str, ",");
	//wyœlij ramkê UDP na adres ip znajduj¹cy siê w tablicy farip[] na port 24478 (odczytywane przez plik filetoNAVDEC, aby obliczaæ poprawne CPA i TCPA) 
	send_udp(buf, str, strlen(str), 8095, farip[0], 24478);
	//wyœlij ramkê UDP na adres ip znajduj¹cy siê w tablicy farip[] na port 10110 (odczytywane przez program NAVDEC)
	send_udp(buf, str, strlen(str), 8095, farip[0], 10110);
}

