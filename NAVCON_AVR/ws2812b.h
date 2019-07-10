/*
 * ws2812b.h
 *
 * Created: 2017-05-27 17:48:07
 *  Author: LUKE
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>


#ifndef WS2812B_H_
#define WS2812B_H_

///////////////////////////////////////////////////////////////////////
// Define Reset time in µs.
//
// This is the time the library spends waiting after writing the data.
//
// WS2813 needs 300 µs reset time
// WS2812 and clones only need 50 µs
//
///////////////////////////////////////////////////////////////////////

#define ws2812_resettime 100

///////////////////////////////////////////////////////////////////////
// Define I/O pin
///////////////////////////////////////////////////////////////////////


#define ws2812_port B     // Data port
#define ws2812_pin  2     // Data out pin


/*
 *  Structure of the LED array
 *
 * cRGB:     RGB  for WS2812S/B/C/D, SK6812, SK6812Mini, SK6812WWA, APA104, APA106
 * cRGBW:    RGBW for SK6812RGBW
 */

typedef struct  { uint8_t g; uint8_t r; uint8_t b; } cRGB;
struct cRGBW { uint8_t g; uint8_t r; uint8_t b; uint8_t w;};
	
#define ROSE_DISK_MAXPIX 95 //liczba ledów na tarczy g³ównej
#define STEER_MAXPIX 20 //liczba ledów na wskaŸniku wychylenia steru
#define SPEED_MAXPIX 20 //liczba ledów na wskaŸniku nastawy prêdkoœci

#define TARCZA_PIN (1<<5)   //PORTB pin do którego bêdzie pod³¹czona tarcza
#define PREDKOSC_PIN (1<<4) //PORTD pin do którego bedzie podl¹czony pasek z nastaw¹ prêdkoœci
#define STER_PIN (1<<6)     //PORTB pin do którego bêdzie pod³¹czony pasek z wychyleniem steru

extern cRGB rgbColors[10];
extern cRGB roseLed[ROSE_DISK_MAXPIX];
extern cRGB steerLed[STEER_MAXPIX];
extern cRGB speedLed[SPEED_MAXPIX];


/* User Interface
 * 
 * Input:
 *         ledarray:           An array of GRB data describing the LED rgbColors
 *         number_of_leds:     The number of LEDs to write
 *         pinmask (optional): Bitmask describing the output bin. e.g. _BV(PB0)
 *
 * The functions will perform the following actions:
 *         - Set the data-out pin as output
 *         - Send out the LED data 
 *         - Wait 50µs to reset the LEDs
 */



void ws2812_setleds     (cRGB  *ledarray, uint16_t number_of_leds);
void ws2812_setleds_pin (cRGB  *ledarray, uint16_t number_of_leds,uint8_t pinmask);
void ws2812_setleds_rgbw(struct cRGBW *ledarray, uint16_t number_of_leds);

/* 
 * Old interface / Internal functions
 *
 * The functions take a byte-array and send to the data output as WS2812 bitstream.
 * The length is the number of bytes to send - three per LED.
 */

void ws2812_sendarray     (uint8_t *array,uint16_t length);
void ws2812_sendarray_mask(uint8_t *array,uint16_t length, uint8_t pinmask);


/*
 * Internal defines
 */

#define CONCAT(a, b)            a ## b
#define CONCAT_EXP(a, b)   CONCAT(a, b)

#define ws2812_PORTREG  CONCAT_EXP(PORT,ws2812_port)
#define ws2812_DDRREG   CONCAT_EXP(DDR,ws2812_port)
//****************************G£ÓWNA FUNKCJA************************************

//funkcja u¿ywana w innej funkcji - normalnie nie bêdê jej u¿ywa³
void ws2812_wyslij_dane_PORTB(uint8_t *data,uint16_t datlen,uint8_t maskhi);
void ws2812_wyslij_dane_PORTC(uint8_t *data,uint16_t datlen,uint8_t maskhi);
void ws2812SendDataPORTD(uint8_t *data,uint16_t datlen,uint8_t maskhi);
//data --> wskaznik do tablicy
//datlen --> iloœæ bajtów do wys³ania (w strukturze o wielkoœci 3 bajtów ta zmienna musi byæ powiêkszona 3 razy)
//maskhi  --> numer pinu rejestru - mo¿e byæ np _BV(2) lub (1<<2)





#endif /* WS2812B_H_ */