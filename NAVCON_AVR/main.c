/*
 * SymulatorNAVDECmk1.c
 *
 * Created: 2017-05-10 18:27:20
 * Author : LUKE
 * edited 15-07-2019
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "mkuart.h"
#include "komendy_sym.h"
#include "common.h"
#include "MAX7219.h"
#include "I2CBase.h"
#include "ws2812b.h"
#include "common.h"
#include "mpr121.h"
#include "ip_arp_udp_tcp.h"
#include "enc28j60.h"
#include "net.h"




int main(void)
{
	
	//###################################################################
	//#############################colors################################
	 
	rgbColors[0].g=50; rgbColors[0].r=50; rgbColors[0].b=50;
	rgbColors[1].g=000; rgbColors[1].r=150; rgbColors[1].b=000;//red
	rgbColors[2].g=255; rgbColors[2].r=100; rgbColors[2].b=000;//orange
	rgbColors[3].g=100; rgbColors[3].r=255; rgbColors[3].b=000;//yellow
	rgbColors[4].g=150; rgbColors[4].r=000; rgbColors[4].b=000;//green
	rgbColors[5].g=000; rgbColors[5].r=000; rgbColors[5].b=000;//off
	rgbColors[6].g=000; rgbColors[6].r=000; rgbColors[6].b=255;//blue
	rgbColors[7].g=000; rgbColors[7].r=150; rgbColors[7].b=150;//violet
	rgbColors[8].g=040; rgbColors[8].r=000; rgbColors[8].b=000;//l. green
	rgbColors[9].g=000; rgbColors[9].r=40; rgbColors[9].b=000;//l. red
	
	////////////////////////////////////////////////
	enc28j60Init(mymac);// enc28j60
	// LEDs in  RJ45: green (LEDA) - link ---   yellow (LEDB) - rx/tx
	// enc28j60PhyWrite(PHLCON,0b0000 0100 0111 01 10);
	enc28j60PhyWrite(PHLCON,0x476);
	init_ip_arp_udp_tcp(mymac,myip,80); //  TCP stack inicjalization
	steerInit(); //ADC0 init
	MAX7219_Init(); //SPI init
	I2C_Init();
	timerInit(); 
	roseInit();
	RoseSensorInit();
	buzzerInit();
	mpr_reset();
	Timer2Init();
	encodersInit();
	roseCalibration(); //loop here -> logn execution but not matter
	
	//#######write ship data from flash#################
	memcpy_P(&ownship, &flashOwnship2, sizeof(ownship));
	memcpy_P(&shipmodel, &shipmodel2, sizeof(shipmodel));
	

	// zarejestrowanie w�asnej procedury obs�ugi/reakcji na ping
	register_ping_rec_callback(&ping_callback);

	// zarejestrowanie w�asnej procedury obs�ugi/reakcji na
	// reveive UDP packets
	register_udp_event_callback(udp_event_callback);
	//Zaptyanie ARP
	//client_arp_whohas(buf,PC_IP);
	//client_waiting_gw();
	//_delay_ms(2000);
	
	//autopilot on after restart
	prevTouched = 1;
	switchedToAuto = 1;
	sei();
	NAVCON_CALIBRATION();
    while (1) 
    {
		prevTouched = readMainButtons();
		//auto/manual
		switch(prevTouched & 0x03) {  //first auto / second manual 
			case 1: {
				if(switchedToAuto)
				 {
					 shipparam.requiredCourse = ownship.course; 
					 shipparam.requiredSpeed = ownship.speed;
				 }
				updateSpeedAuto();
				updateCourseAuto();
				MAX7219_SendCourseAndSpeed(shipparam.requiredCourse, shipparam.requiredSpeed);
				showSpeedSettingLED(shipparam.requiredSpeed, 8, 9);
				switchedToAuto = 0;
				} break;
				
			case 2:  {
				switchedToAuto = 1; 
				convertFrom10bitValueToRequiredSteerAngle();
				MAX7219_autopilotOFF(); //"lines" in autopilot displays
				uint8_t status = (uint8_t)readSteerButtons();
				if(status & 0x04) shipparam.requiredSpeed += shipmodel.maxSpeed/8; // + 1/8 maxspeed
				if(status & 0x08) shipparam.requiredSpeed -= shipmodel.maxSpeed/8; // - 1/8 maxspeed
				if(shipparam.requiredSpeed > shipmodel.maxSpeed) shipparam.requiredSpeed = shipmodel.maxSpeed;   
				if(shipparam.requiredSpeed < -shipmodel.maxSpeed) shipparam.requiredSpeed = -shipmodel.maxSpeed; 
				showSpeedSettingLED(shipparam.requiredSpeed, 4, 1);
			} break;
				
			      }//end switch
		UDP_EVENT(myport);
		computeShipParameters();
		I2C_SendCourseAndSpeed( ownship.course, ownship.speed, I2C_MAIN_DISPL_ADDR);	
		
		//ws2812SendDataPORTD((uint8_t *)roseLed, ROSE_DISK_MAXPIX * 3, TARCZA_PIN);
		ws2812SendDataPORTD((uint8_t *)speedLed, SPEED_MAXPIX * 3, PREDKOSC_PIN);
		//ws2812SendDataPORTD((uint8_t *)steerLed, STEER_MAXPIX * 3, STER_PIN);
		
		}
		
		
}

