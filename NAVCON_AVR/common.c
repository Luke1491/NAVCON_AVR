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
int8_t computeROT(int8_t currentSteerAngle)  //funkcja oblicza kurs co 1 sekund� --> ustawi� odpowiedni timer programowy
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
		encoderNumberOfStepsPerSecond = 0; //encoders - higher incrementation by 10 degrees
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
	return ADC;// ten zapis***(ADCH<<8) | ADCL;***not work - WHY???
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
	else  ownship.speed -= 1;                                   //if rz�danaSpeed < aktualnaSpeed --> decrease ownship.speed o 0,1kt
	
	if(ownship.speed >= shipmodel.maxSpeed) ownship.speed = shipmodel.maxSpeed;
	if(ownship.speed <= -shipmodel.maxSpeed) ownship.speed = -shipmodel.maxSpeed;
}

void showSpeedSettingLED(int16_t _requiredSpeed, uint8_t positive, uint8_t negative) //speed set from -8 to +8
{
	if(_requiredSpeed>=shipmodel.maxSpeed) _requiredSpeed = shipmodel.maxSpeed; //not allow to increase speed above max speed
	int8_t percentSpeed;
	percentSpeed = (_requiredSpeed*100)/(shipmodel.maxSpeed);
	for(uint8_t i=0; i<SPEED_MAXPIX; i++)
	{
		speedLed[i] = rgbColors[5];
	}
	
	if(percentSpeed<=100 && percentSpeed>88)
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
	if(percentSpeed<=88 && percentSpeed>75)
	{
		speedLed[3] = rgbColors[positive];
		speedLed[4] = rgbColors[positive];
		speedLed[5] = rgbColors[positive];
		speedLed[6] = rgbColors[positive];
		speedLed[7] = rgbColors[positive];
		speedLed[8] = rgbColors[positive];
		speedLed[9] = rgbColors[positive];
		
	}
	if(percentSpeed<=75 && percentSpeed>63)
	{
		speedLed[4] = rgbColors[positive];
		speedLed[5] = rgbColors[positive];
		speedLed[6] = rgbColors[positive];
		speedLed[7] = rgbColors[positive];
		speedLed[8] = rgbColors[positive];
		speedLed[9] = rgbColors[positive];
	}
	if(percentSpeed<=63 && percentSpeed>50)
	{
		speedLed[5] = rgbColors[positive];
		speedLed[6] = rgbColors[positive];
		speedLed[7] = rgbColors[positive];
		speedLed[8] = rgbColors[positive];
		speedLed[9] = rgbColors[positive];
	}
	if(percentSpeed<=50 && percentSpeed>38)
	{
		speedLed[6] = rgbColors[positive];
		speedLed[7] = rgbColors[positive];
		speedLed[8] = rgbColors[positive];
		speedLed[9] = rgbColors[positive];
	}
	if(percentSpeed<=38 && percentSpeed>25)
	{
		speedLed[7] = rgbColors[positive];
		speedLed[8] = rgbColors[positive];
		speedLed[9] = rgbColors[positive];
	}
	if(percentSpeed<=25 && percentSpeed>13)
	{
		speedLed[8] = rgbColors[positive];
		speedLed[9] = rgbColors[positive];
	}
	if(percentSpeed<=13 && percentSpeed>0)
	{
		speedLed[9] = rgbColors[positive];
	}
	
	if(percentSpeed>=-100 && percentSpeed<-88)
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
	if(percentSpeed>=-88 && percentSpeed<-75)
	{
		speedLed[10] = rgbColors[negative];
		speedLed[11] = rgbColors[negative];
		speedLed[12] = rgbColors[negative];
		speedLed[13] = rgbColors[negative];
		speedLed[14] = rgbColors[negative];
		speedLed[15] = rgbColors[negative];
		speedLed[16] = rgbColors[negative];
	}
	if(percentSpeed>=-75 && percentSpeed<-63)
	{
		speedLed[10] = rgbColors[negative];
		speedLed[11] = rgbColors[negative];
		speedLed[12] = rgbColors[negative];
		speedLed[13] = rgbColors[negative];
		speedLed[14] = rgbColors[negative];
		speedLed[15] = rgbColors[negative];
	}
	if(percentSpeed>=-63 && percentSpeed<-50)
	{
		speedLed[10] = rgbColors[negative];
		speedLed[11] = rgbColors[negative];
		speedLed[12] = rgbColors[negative];
		speedLed[13] = rgbColors[negative];
		speedLed[14] = rgbColors[negative];
	}
	if(percentSpeed>=-50 && percentSpeed<-38)
	{
		speedLed[10] = rgbColors[negative];
		speedLed[11] = rgbColors[negative];
		speedLed[12] = rgbColors[negative];
		speedLed[13] = rgbColors[negative];
	}
	if(percentSpeed>=-38 && percentSpeed<-25)
	{
		speedLed[10] = rgbColors[negative];
		speedLed[11] = rgbColors[negative];
		speedLed[12] = rgbColors[negative];
	}
	if(percentSpeed>=-25 && percentSpeed<-13)
	{
		speedLed[10] = rgbColors[negative];
		speedLed[11] = rgbColors[negative];
	}
	if(percentSpeed>=-13 && percentSpeed<0)
	{
		speedLed[10] = rgbColors[negative];
	}
}

/////////////////////////////////////////////////////////////////////
//************************POSITION COUNTING**************************
void computePosition(void)
{
	int32_t distance=0; //distance made in last one second
	int32_t deltaLat, deltaLong;
	distance = (((int32_t)ownship.speed*1000)/36)/60; //Nm distance -> precision to 0,000001 (0,000001 -> distance = 1)
	double courseRadians = ((double)ownship.course/573);
	double latitudeRadians = ((double)ownship.posLat/57295779);
	//latitude
	deltaLat = distance * cos(courseRadians); // precision up to 0,000001
	ownship.posLat = ownship.posLat + deltaLat;//new position with precision up to 0,000001
	//longitude
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
	OPERTING MODE
	MS1	MS2	MS3
	0	0	0	-- FULL STEP
	1	0	0	--1/2 STEP
	0	1	0	--1/4 STEP
	1	1	0	--1/8 STEP
	1	1	1	--1/16 STEP

	A4988		CONNECTED PCF8574P PIN		DESCRIPTION
	EN			P0					        1-motor off  / 0-motor on
	MS1			P1
	MS2			P2 
	MS3			P3 
	RESET		P4							1-normal operation / 0-RESET
	SLEEP		P5							1- normal operation  / 0-sleep
	STEP		P6							value 1 -> make one step
	DIR			P7							1-turn right / 0-turn left
*/
#define MOTOREN				 0
#define MOTORMS1			 1
#define MOTORMS2			 2
#define MOTORMS3			 3
#define MOTORRESET			 4
#define MOTORSLEEP			 5
#define MOTORSTEP			 6
#define MOTORDIR			 7
#define MOTORMAINSETTING	((1 << MOTORRESET)|(1 << MOTORSLEEP))//pe�en krok
//rose
void roseInit(void)
{
	I2C_StartSelectWait(0x70); //send start bit and expander addres
	I2C_SendByte(MOTORMAINSETTING | (1 << MOTOREN)); //1/16 step , normal operation , motor's coil off
	I2C_Stop();
	I2C_WaitTillStopWasSent();
}

uint16_t degreesToLED(uint16_t degrees)
{
	degrees=degrees+360-(ownship.course/10);
	if(degrees >= 360) degrees=degrees-360;
	if(degrees < 0) return 0;
	uint16_t LedArrayNumber = 0;
	uint16_t moduloResult = 0;
	if(degrees == 0 ) degrees = 1;
	for(uint16_t i = 0; i<(degrees*10); i++)
	{
		moduloResult = i % 38;
		if(moduloResult == 0) LedArrayNumber = LedArrayNumber + 1;
	}
	return LedArrayNumber;
}

void RoseSensorInit(void)
{
	DDRC &= ~(1<<PC2); //PC2 as input
	PORTC &= ~(1<<PC2); //no pullup (pullup made on PCB)
}

uint8_t fullStep = 1; // 1 - full step (no movement required)   0- micro step in progress 
uint8_t roseDir = 0; // 0-left   1-right
void moveRoseDiskByOneStep(uint8_t dir) //1 step == 0,1 degree with gear
{
	I2C_StartSelectWait(0x70);
		if(dir) {
		I2C_SendByte(MOTORMAINSETTING | (1 << MOTORSTEP)); //make one step 
		I2C_SendByte(MOTORMAINSETTING); //prepare for another step
		 } //rose turn right
	else { 
		I2C_SendByte(MOTORMAINSETTING | (1<<MOTORDIR)); //make one step 
		I2C_SendByte(MOTORMAINSETTING | (1 << MOTORSTEP) |(1<<MOTORDIR)); //make one step
		I2C_SendByte((MOTORMAINSETTING) |(1<<MOTORDIR)); //prepare for another step
		 } //rose turn left
	I2C_Stop();
	I2C_WaitTillStopWasSent();
	fullStep = 1; //step has made
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
		motorCoilOff();//turnoff motor coil
		fullStep = 0; //allow to enable motor in next call(for 100ms)
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
		roseDir = 1;
		moveRoseDiskByOneStep(roseDir);
		roseCourse+=1;
		if(roseCourse > 3600) roseCourse = 0;
	}
	else if(ROTDifference < 0)
	{
		roseDir = 0;
		moveRoseDiskByOneStep(roseDir);
		roseCourse -=1;
		if(roseCourse < 0) roseCourse = 3600;
	}
	else ; //if ROTDifference == 0 -> do nothing
	}//end else from main if
}

void roseCalibration(void)
{
	while(!(PINC & (1<<PC2)))
	{
		moveRoseDiskByOneStep(1);
	}//end while
	roseCourse = 0;
}

//*******************BUZZER*********************
void buzzerInit(void)
{
	DDRC |= (1<<PC1); //PC1 as output
	PORTC &= ~(1<<PC1); //low state - buzzer not working
}

/////////////////////////////////////////////////////////////////////////////////////
//encoder
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
//encoders timer
void Timer2Init(void)
{
	TCCR2B=_BV(CS01);	//Prescaler CLKIO/8
	TIMSK2|=_BV(TOIE2);	//enable overflow interrupt
}

ISR(TIMER2_OVF_vect)
{
	ReadEncoder1();
	ReadEncoder2();
}

//*************
void encodersInit(void)
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
	static uint8_t counters[2];	//program counters array
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
	if((ENC1PIN & ENC1SIGN2)==0) newpos^=1;	// grey to binaty conversion
	diff=last-newpos;
	if(diff & 1)
	{				// bit 0 = step
		last=newpos;
		enc_delta1+=(diff & 2)-1;	//bit 1 - direction
	}
}

void ReadEncoder2()
{
	static int8_t last;
	static uint8_t laststate;
	static uint8_t counters[2];
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
	if((ENC2PIN & ENC2SIGN2)==0) newpos^=1;
	diff=last-newpos;
	if(diff & 1)
	{
		last=newpos;
		enc_delta2+=(diff & 2)-1;
	}
}
int8_t Read1StepEncoder(uint8_t encoderNumber)
{
	int8_t val;
	if(encoderNumber == 1) 
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

int8_t Read2StepEncoder(uint8_t encoderNumber)
{
	int8_t val;
	if(encoderNumber == 1) 
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

int8_t Read4StepEncoder(uint8_t encoderNumber)
{
	int8_t val;
	if(encoderNumber == 1)
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


void updateCourseAuto(void)
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

void updateSpeedAuto(void)
{
	
	shipparam.requiredSpeed += Read4StepEncoder(2); //read encoder2 and add to requiredSpeed
	if(shipparam.requiredSpeed >= shipmodel.maxSpeed) shipparam.requiredSpeed = shipmodel.maxSpeed;
	if(shipparam.requiredSpeed <= -shipmodel.maxSpeed) shipparam.requiredSpeed = -shipmodel.maxSpeed;
}

uint16_t readMainButtons(void)
{
	uint16_t touched;
	static uint16_t pretouch = 1;
	if(TouchTimer >= 100)
	{
		touched = MPR_read_reg16bit(MPR_DEVICE_ADDRESS, MPR_R_TOUCHSTATUS);
	
	if(touched & 0x03) 
	{
		TouchTimer = 0; //next button read in 1s
		pretouch = touched;
		buzzerTimer = 0; //start count time to buzzer off
		BUZZER_ENABLE;      //buzzer off
		return touched;
	}
	if(!(touched & 0x03))
	{
		TouchTimer = 90; //next button read in 0,1s
	}
	}//end main if
	else
	{
		buzzerSoundTime(3);	
	}
	return pretouch;
}

void buzzerSoundTime(uint8_t time)
{
	if(buzzerTimer >= time) PORTC &= ~(1<<PC1); //buzzer off after (time*10) ms
}

uint16_t readSteerButtons(void)
{
	uint16_t touchedSteer;
	if(speedSetInManualTimer >= 100)
	{
		touchedSteer = MPR_read_reg16bit(MPR_DEVICE_ADDRESS, MPR_R_TOUCHSTATUS);
		
		if(touchedSteer & 0x0C) //read 2 older bits in 1 tetrade
		{
			speedSetInManualTimer = 50; //next button read in 0,5s
			buzzerTimer = 0; //start count time to buzzer off
			BUZZER_ENABLE;      //buzzer off
			return touchedSteer;
		}
		if(!(touchedSteer & 0x0C))
		{
			speedSetInManualTimer = 90; //next button read in 0,1s
		}
	}//end main if
	else
	{
		buzzerSoundTime(3);
	}
	return 0;
}

//##############################################################################################################
//#############################################ETHERNET FUNCTION################################################
//##############################################################################################################
// my MAC
uint8_t mymac[6] = {'N','A','V','D','E','C'};
// my IP 
uint8_t myip[4] = {169,254,131,153};

// set UDP ports to operate
// (there can be any amount)
uint16_t myport[] = {8095,24478};
// gateway ip in lan
//static uint8_t gwip[4] = {169,254,255,255};

//  TCP/UDP buffer size

uint8_t buf[BUFFER_SIZE+1];

// pointer to callback function in UDP_EVENT() event
void (*mk_udp_event_callback)(uint8_t *peer_ip, uint16_t port,
uint16_t datapos, uint16_t len);

// function to register callback function in UDP_EVENT() event
void register_udp_event_callback(void (*callback)(uint8_t *peer_ip,
uint16_t port, uint16_t datapos, uint16_t len))
{
	mk_udp_event_callback = callback;
}



// IP numbers alises
enum ip_names {ip_pc, ip_sterownik1};
// IP adresses drivers to communicate with
// UDP protocol  ----> first array element is connected PC
// second IP is another driver eq another PC or ABT board
uint8_t farip[2][4] = { {169,254,131,151}, {192,168,0,180} };
uint8_t PC_IP[4] = {169,254,131,151};



// tokens separator
char sep[] = ","; //In NAVDEC communications there are ',' as separate tokens and '.' as decimal in numbers

//_______________________________________________________________________________________________________________
//*****************command functions**********************
//navconCommand $ROZETA
void parseROZETA(char *buffer)
{
	//uint8_t przygotowanieDoTarczyZielonej;
	char *ethernetPtr,*ptr1;
	uint16_t sectorsNumbers, sectorNo, value1, value2;
	uint16_t value1LED, value2LED;
	uint32_t fractionValue1, fractionValue2;
	char str[25];
	ethernetPtr = strtok_r(NULL, sep, &buffer); //pull out numbers of sectors on rose
	sectorsNumbers = atol(ethernetPtr);
	ethernetPtr = strtok_r(NULL, sep, &buffer); //sector number
	sectorNo = atol(ethernetPtr);
	ethernetPtr = strtok_r(NULL, sep, &buffer); //first value
	//-----------------------
	strcpy(str, ethernetPtr);
	ptr1 = strtok(str, ".");
	value1 = atol(ptr1);
	ptr1 = strtok(NULL, ",");
	fractionValue1 = atol(ptr1);
	//-----------------------
	ethernetPtr = strtok_r(NULL,sep, &buffer);//second value
	//----------------------
	strcpy(str, ethernetPtr);
	ptr1 = strtok(str, ".");
	value2 = atol(ptr1);
	ptr1 = strtok(NULL, ",");
	fractionValue2 = atol(ptr1);
	//----------------------
	ethernetPtr = strtok_r(NULL, sep, &buffer);//ethernetPtr ->NULL
	
	value1LED = degreesToLED(value1);
	value2LED = degreesToLED(value2);
	if( sectorNo == 1) //when first sector apper -> reeset rose and paint to red color
	{
		for(uint8_t j = 0; j<ROSE_DISK_MAXPIX; j++)
		{
			roseLed[j] = rgbColors[1];  //set red color to all rose
		}
	}
	
	if(value1LED>value2LED)
	{
		uint16_t auxiliaryVariable;
		auxiliaryVariable = value1LED;
		value1LED = value2LED; 
		value2LED = auxiliaryVariable;
		for(uint8_t i = 0; i<value1LED; i++)
		{
			roseLed[i] = rgbColors[3];//yellow
		}
		for(uint8_t i = value2LED; i<ROSE_DISK_MAXPIX; i++)
		{
			roseLed[i] = rgbColors[3];//yellow
		}
	}
	else
	{
		
	if(sectorsNumbers==1 && sectorNo==1 && value1==0 && value2==360)
	{
		for(uint8_t i = 0; i<ROSE_DISK_MAXPIX; i++)
		{
			roseLed[i] = rgbColors[8];//green
		}
	}
	
	for(uint8_t i = value1LED; i<value2LED; i++)
	{
		roseLed[i] = rgbColors[3];
	}//end for
	}//end main else 
	//MAX7219_SendCourseAndSpeed(value1LED, value2LED);//test display (normal comment)
}

//SHIPMODEL command
void parseSHIPMODEL(char *buffer)
{
	char *ethernetPtr;
	uint8_t numerModelu;
	ethernetPtr = strtok_r(NULL, sep, &buffer); //ship model number
	numerModelu = atoi(ethernetPtr);
	ethernetPtr = strtok_r(NULL, sep, &buffer); //ethernetPtr ->NULL
	switch(numerModelu)
	{
		case 1: {
			//#######COPY FROM FLASH TO OWNSHIP VARIABLE################
			memcpy_P(&ownship, &flashOwnship1, sizeof(ownship));
			//SHIPBEHAVIOR FROM FLASH TO VARIABLE
			memcpy_P(&shipmodel, &shipmodel1, sizeof(shipmodel));
		} break;
		case 2: {
			//#######COPY FROM FLASH TO OWNSHIP VARIABLE#################
			memcpy_P(&ownship, &flashOwnship2, sizeof(ownship));
			//SHIPBEHAVIOR FROM FLASH TO VARIABLE
			memcpy_P(&shipmodel, &shipmodel2, sizeof(shipmodel));
		} break;
		case 3: {
			//#######COPY FROM FLASH TO OWNSHIP VARIABLE#################
			memcpy_P(&ownship, &flashOwnship3, sizeof(ownship));
			//SHIPBEHAVIOR FROM FLASH TO VARIABLE
			memcpy_P(&shipmodel, &shipmodel3, sizeof(shipmodel));
		} break;
		default: {
			//#######COPY FROM FLASH TO OWNSHIP VARIABLE#################
			memcpy_P(&ownship, &flashOwnship1, sizeof(ownship));
			//SHIPBEHAVIOR FROM FLASH TO VARIABLE
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
/******* FUNCTION FOR INCOMING UDP DATAGRAM ***********/
/****/
void udp_event_callback(uint8_t *peer_ip, uint16_t port, uint16_t datapos, uint16_t len) {
	//uint8_t i=0;
    char str[40];

    // pointer for strtok_r function
    char *ethernetPtr, *rest;
	uint8_t navconCommand;


    // if UDP datagram came on second element of myport[] array
    if(port == myport[1]) {
    	// replay to the same port
    	strcpy(str,"UDP replay");
    	make_udp_reply_from_request(buf,str,strlen(str), port);

        // send UDP to any port
    	strcpy((char*)&buf[datapos],"New UDP!");
        send_udp_prepare(buf, 1200, farip[ip_pc], 43500);
        send_udp_transmit(buf,strlen(str));
    }

    // if UDP datagram came on first element of PCmyport[] array
    if(port == myport[0]) {
    	// check if not empty
    	ethernetPtr = strtok_r((char*)&buf[datapos + 1], sep, &rest); //buf[datapos + 1] to eleiminate $ sign wchih couse compilation error
    	if( ethernetPtr ) {
    		//if UDP inot empty -> check what command 
			char charkom [10];
			strcpy(charkom, ethernetPtr);
			
			if(!(strcmp(charkom, "ROZETA"))) navconCommand = 0;
			else if(!(strcmp(charkom, "SHIPMODEL"))) navconCommand = 1;
			else if(!(strcmp(charkom, "set_lcd"))) navconCommand = 2;
			else navconCommand = 10;
    		switch( navconCommand ) {
    		case 0: parseROZETA(rest); break;
    		case 1: parseSHIPMODEL(rest); break;
    		case 2: ; break;
			case 10: ; break;
    		}
			
    	}
    }
}


/********** UDP EVENT ***********************/
void UDP_EVENT(uint16_t *port) {
    uint16_t plen;//, dat_p;
    uint16_t dport;
    uint8_t is_my_port=0;
    uint8_t i=0;


    uint8_t udp_data_len=0;


    // check in data is received
    plen = enc28j60PacketReceive(BUFFER_SIZE, buf);
    // lower TCP stack layer handling like ICMP, ARP etc.
    // external PINGs are handling  with this method (there more methods)
    packetloop_icmp_tcp(buf,plen);

    // if datagram in bigger than 0 means datagram 
	//is received correctly (with correct checksum)
    // check if there is UDP (not TCP)
    if( plen && buf[IP_PROTO_P]==IP_PROTO_UDP_V ) {

    	// check to what port goes UDP
    	dport = (buf[UDP_DST_PORT_H_P]<<8) | buf[UDP_DST_PORT_L_P];

    	// checki if datagram goes to handling port (definde in myport[])
    	do {
    		if( (is_my_port = (port[i++] == dport)) ) break;
    	} while (i<sizeof(port));

    	// check if datagram came on operational port
    	if ( is_my_port ){
			udp_data_len=buf[UDP_LEN_L_P]-UDP_HEADER_LEN;

			// if all is ok, then check if callback function is registered
			// if not - nothing will happen
			// if registered:
			// 1. sender IP adress
			// 2. UDP datagram receive port
			// 3. UDP buffer index point to start of the datagram
			// 4. number of bytes of UDP datagram
			if(mk_udp_event_callback) (*mk_udp_event_callback)(&(buf[IP_SRC_P]), dport, UDP_DATA_P, udp_data_len);
        }
    }
}

void ping_callback(uint8_t *ip){
	//response from ping request
}

void prepareAndSendOWNDatagram(void)
{
	//prepare UDP
	char str [150] = {"$OWN"};
	char str1[21]; //array for string representation of enumerate value (1 value each time)
	int32_t valueToShowUDP;  //for computed values
	strcat(str, ",0,");                                 //1. integrated position
	strcat(str, ownship.MMSI);							//2. MMSI
	strcat(str, ",");
	strcat(str, ownship.NAVSTATUS);						//3. nav status
	strcat(str, ",");
	valueToShowUDP = shipparam.currentROT*6/10;
	itoa(valueToShowUDP, str1, 10);
	strcat(str, str1 );									//4. ROT - intiger
	strcat(str, ".");								
	valueToShowUDP = (shipparam.currentROT%100)*6/10;
	itoa(valueToShowUDP, str1, 10);
	strcat(str, str1 );									//4. ROT - fractional number
	strcat(str, ",");									
	valueToShowUDP = ownship.speed/10;
	if(ownship.speed<0){strcat(str, "-"); valueToShowUDP = -valueToShowUDP;}//umozliwia interpretacje przez navdec				
	itoa(valueToShowUDP, str1, 10);
	strcat(str, str1 );									//5. SOG - intiger
	strcat(str, ".");	
	valueToShowUDP = ownship.speed%10;
	if(ownship.speed<0){ valueToShowUDP = -valueToShowUDP;}				
	itoa(valueToShowUDP, str1, 10);
	strcat(str, str1 );									//5. SOG - fractional number
	strcat(str, ",");	
	valueToShowUDP = ownship.posLong/1000000;
	if(ownship.posLong<0){strcat(str, "-"); valueToShowUDP = -valueToShowUDP;}//allow interpretate by NAVDEC
	itoa(valueToShowUDP, str1, 10);
	strcat(str, str1 );									//6. LONG - intiger
	strcat(str, ".");
	valueToShowUDP = ownship.posLong%1000000;
	if(ownship.posLong<0){valueToShowUDP = -valueToShowUDP;}//allow interpretate by NAVDEC
	//significant '0' security
	if(valueToShowUDP<100000 && valueToShowUDP>=10000)	    strcat(str, "0");
	else if(valueToShowUDP<10000 && valueToShowUDP>=1000)	strcat(str, "00");
	else if(valueToShowUDP<1000 && valueToShowUDP>=100)		strcat(str, "000");
	else if(valueToShowUDP<100 && valueToShowUDP>=10)		strcat(str, "0000");
	else if(valueToShowUDP<10 && valueToShowUDP>=1)			strcat(str, "00000");
	
	ltoa(valueToShowUDP, str1, 10);
	strcat(str, str1 );									//6. LONG-  fractional number
	strcat(str, ",");
	valueToShowUDP = ownship.posLat/1000000;
	if(ownship.posLat<0){strcat(str, "-"); valueToShowUDP = -valueToShowUDP;}//allow interpretate by NAVDEC
	itoa(valueToShowUDP, str1, 10);
	strcat(str, str1 );									//7. LAT - intiger
	strcat(str, ".");
	valueToShowUDP = ownship.posLat%1000000;
	if(ownship.posLat<0){valueToShowUDP = -valueToShowUDP;}//allow interpretate by NAVDEC
	//significant '0' security
	if(valueToShowUDP<100000 && valueToShowUDP>=10000)		strcat(str, "0");
	else if(valueToShowUDP<10000 && valueToShowUDP>=1000)	strcat(str, "00");
	else if(valueToShowUDP<1000 && valueToShowUDP>=100)		strcat(str, "000");
	else if(valueToShowUDP<100 && valueToShowUDP>=10)		strcat(str, "0000");
	else if(valueToShowUDP<10 && valueToShowUDP>=1)			strcat(str, "00000");
	ltoa(valueToShowUDP, str1, 10);
	strcat(str, str1 );									//7. LAT-  fractional number
	strcat(str, ",");
	valueToShowUDP = ownship.course/10;
	itoa(valueToShowUDP, str1, 10);
	strcat(str, str1 );									//8. COG - intiger
	strcat(str, ".");
	valueToShowUDP = ownship.course%10;
	itoa(valueToShowUDP, str1, 10);
	strcat(str, str1 );									//8. COG -  fractional number
	strcat(str, ",");
	valueToShowUDP = ownship.course/10;
	itoa(valueToShowUDP, str1, 10);
	strcat(str, str1 );									//9. HEADING - intiger (the same as COG)
	strcat(str, ".");
	valueToShowUDP = ownship.course%10;
	itoa(valueToShowUDP, str1, 10);
	strcat(str, str1 );									//9. HEADING -  fractional number (the same as COG)
	strcat(str, ",");
	strcat(str, "0.0,0.0," );							//10 / 11 position cartesian 
	strcat(str, ownship.PAS );							//12. PAS
	strcat(str, ",");
	strcat(str, ownship.TYP_COMMUNIKATU );				//13. communicate type
	strcat(str, ",");
	strcat(str, ownship.IMO_NUMBER );					//14. IMO
	strcat(str, ",");
	strcat(str, ownship.CALL_SIGN );					//15. CALLSIGN
	strcat(str, ",");
	strcat(str, ownship.SHIP_NAME);						//16. SHIP NAME
	strcat(str, ",");
	strcat(str, ownship.TYPE_OF_SHIP );					//17. KIND OF SHIP
	strcat(str, ",");
	strcat(str, ownship.DIM_A );						//18. DIMA
	strcat(str, ",");
	strcat(str, ownship.DIM_B );						//19. DIMB
	strcat(str, ",");
	strcat(str, ownship.DIM_C );						//20. DIMC
	strcat(str, ",");
	strcat(str, ownship.DIM_D );						//21. DIMD
	strcat(str, ",1,");									//22. comma + type of device
	valueToShowUDP = ownship.ETA_month;
	itoa(valueToShowUDP, str1, 10);
	strcat(str, str1 );									//23. ETA MONTH
	strcat(str, ",");
	valueToShowUDP = ownship.ETA_day;
	itoa(valueToShowUDP, str1, 10);
	strcat(str, str1 );									//24. ETA DAY
	strcat(str, ",");
	valueToShowUDP = ownship.ETA_hour;
	itoa(valueToShowUDP, str1, 10);
	strcat(str, str1 );									//25. ETA HOUR
	strcat(str, ",");
	valueToShowUDP = ownship.ETA_minute;
	itoa(valueToShowUDP, str1, 10);
	strcat(str, str1 );									//26. ETA MINUTE
	strcat(str, ",");
	strcat(str, ownship.DRAUGHT);						//27. DRAUGHT
	strcat(str, ",");
	strcat(str, ownship.DESTINATION);					//28. DESTINATION	
	strcat(str, ",");
	//send UDP to ip be in farip[] to port 24478 (read by  filetoNAVDEC file, for propper CPA and TCPA computing ) 
	send_udp(buf, str, strlen(str), 8095, farip[0], 24478);
	//send UDP to ip be in farip[] to port 10110 (read by NAVDEC)
	send_udp(buf, str, strlen(str), 8095, farip[0], 10110);
}

