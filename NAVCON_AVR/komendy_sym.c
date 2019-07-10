/*
 * komendy_sym.c  
 *
 *  Created on: 12-05-2017
 *      Author: £ukasz Cieplicki based on M.Kardaœ
 */
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>

#include "mkuart.h"

#include "komendy_sym.h"

#include "common.h"

#define AT_CNT 	8	// iloæ poleceñ AT


//----------- tablica z poleceniami AT i wskaŸnikami funkcji do ich obs³ugi --------------------
const TATCMD polecenia_at[AT_CNT] PROGMEM = {
	// { at_cmd } , { wskaŸnik do funkcji obs³ugi at },
		{"SYM", 		at_service},
		{"SYM+INFO", 	ati_service},
		{"SYM+LUM", 	at_lumin_service},
		{"SYP+POS", 	at_pos_service},
		{"SYM+OWN", 	at_own_service},
		{"SYM+TEST", 	at_test_service},
        {"SYM+OWNDEF", 	at_owndef_service},
		{"SYM+COURSE",  at_course_service}
};



//----------------- funkcja do analizowania danych odebranych z UART ------------------------------
void parse_uart_data( char * pBuf ) {

	int8_t (*_at_srv)(uint8_t inout, char * data);

	char * cmd_wsk;
	char * reszta;
	uint8_t i=0, len;

/*	// wywietlamy odebrane polecenia AT na LCD
	lcd_locate(0,0);
//	lcd_str("odebrano at cmd:");
	lcd_locate(1,0);
	lcd_str("                ");
	lcd_locate(1,0);
	lcd_str("[");
	lcd_str( pBuf );
    lcd_str("]");*/

	if ( strpbrk(pBuf, "=?"))	{
		// obs³uga poleceñ AT we/wy + parametry

		if ( strpbrk(pBuf, "?"))	{
			// zapytania do uk³adu w postaci: AT+CMD?

			cmd_wsk = strtok_r(pBuf, "?", &reszta);
			len = strlen(cmd_wsk);
			for(i=0;i<AT_CNT;i++) {
				if ( len && 0 == strncasecmp_P(cmd_wsk, polecenia_at[i].polecenie_at, len) ) {
					if( pgm_read_word(polecenia_at[i].polecenie_at) ) { // <--- UWAGA! w tekœcie ksi¹¿ki zabrak³o pgm_read_word()
						_at_srv = (void *)pgm_read_word( &polecenia_at[i].at_service );
						if( _at_srv) {
							if( _at_srv( 0, reszta ) < 0 ) uart_puts("ERROR\r\n");
						}
					}
					uart_puts("\r\n");
					break;
				}
			}

		} else {
			// ustawienia uk³adu w postaci: AT+CMD=parametry

			cmd_wsk = strtok_r(pBuf, "=", &reszta);
			len = strlen(cmd_wsk);
			for(i=0;i<AT_CNT;i++) {
				if ( len && 0 == strncasecmp_P(cmd_wsk, polecenia_at[i].polecenie_at, len) ) {
					if( pgm_read_word(polecenia_at[i].polecenie_at) ) { // <--- UWAGA! w tekœcie ksi¹¿ki zabrak³o pgm_read_word()
						_at_srv = (void *)pgm_read_word( &polecenia_at[i].at_service );
						if( _at_srv && ! _at_srv( 1, reszta ) ) uart_puts("OK\r\n");
						else uart_puts("ERROR\r\n");
					}
					break;
				}
			}
		}

	} else {
		// obs³uga poleceñ AT bez parametrów

		if( 0 == pBuf[0] ) uart_puts("\r\n");	// reakcja na znak CR lub CRLF z terminala
		else {
			for(i=0;i<AT_CNT;i++) {
				if ( 0 == strncasecmp_P(pBuf, polecenia_at[i].polecenie_at, strlen(pBuf)) ) {
					if( pgm_read_word(polecenia_at[i].polecenie_at) ) { // <--- UWAGA! w tekœcie ksi¹¿ki zabrak³o pgm_read_word()
						_at_srv = (void *)pgm_read_word( &polecenia_at[i].at_service );
						if( _at_srv) _at_srv(2,0);
					}
					break;
				}
			}
		}
	}

	if( AT_CNT == i ) uart_puts("ERROR\r\n");
}



//----------------- obs³uga poszczególnych komend AT ----------------------------------
int8_t at_service(uint8_t inout, char * params) {
	uart_puts("OK\r\n");
	return 0;
}

int8_t ati_service(uint8_t inout, char * params) {
	uart_puts("Symulator NAVDEC mk1 ver1.1.0\r\n");
	return 0;
}

int8_t at_lumin_service(uint8_t inout, char * params) {

//zmiana zmiennej odpowiadaj¹cej za intensywnoœæ œwiecenia

	return 0;
}

int8_t at_pos_service(uint8_t inout, char * params) {
	/*if( 1 == inout ) {
		if( '1' == params[0] ) ir_enable=1;
		else ir_enable=0;
		uart_puts("+IR: ");
		uart_putint(ir_enable,10);
		uart_puts("\r\n");
	} else if( !inout ) {
		uart_puts("+IR: ");
		uart_putint(ir_enable,10);
		uart_puts("\r\n");
	} else if( 2 == inout ) {
		uart_puts("AT+IR = (0-1)\r\n");
	}*/
	return 0;
}

int8_t at_own_service(uint8_t inout, char * params) {

	/*if( inout < 2 ) {
		if( 1 == inout ) {
			if( params[0] < '0' || params[0] > '1' ) return -1;
			if( '1' == params[0] ) LED_ON;
			else LED_OFF;
		}
		uart_puts("+LED: ");
		if( LED_PIN & LED ) uart_putint(1,10);
		else uart_putint(0,10);
		uart_puts("\r\n");
	} else if( 2 == inout ) {
		uart_puts("AT+LED = (0-1)\r\n");
	}*/
	return 0;

}


int8_t at_test_service( uint8_t inout, char * params ) {
	/*uint8_t y,x;
	char * wsk;

	if( 1 == inout ) {
		// sprawdzamy czy s¹ parametry, jeli nie to b³¹d
		if(!strlen(params)) return -1;
		// wy³uskujemy pierwszy parametr do przecinka
		wsk = strtok(params, ",");
		// sprawdzamy czy s¹ parametry, jeli nie to b³¹d
		if(!strlen(wsk)) return -1;
		// zamieniamy liczbê ASCII na wartoœæ dziesiêtn¹
		y=atoi(wsk);
		// jeœli Y  nie mieœci siê w zakresie od 0 do 3 to b³¹d
		if( !(y>=0 && y<4) ) return -1;
		// wy³uskujemy drugi parametr do przecinka
		wsk = strtok(0, ",");
		// sprawdzamy czy s¹ parametry, jeli nie to b³¹d
		if(!strlen(wsk)) return -1;
		// zamieniamy liczbê ASCII na wartoœæ dziesiêtn¹
		x=atoi(wsk);
		// jeœli X  nie mieœci siê w zakresie od 0 do 39 to b³¹d
		if( !(x>=0 && y<40) ) return -1;
		// wy³uskujemy trzeci parametr do przecinka lub do koñca ³añcucha
		wsk = strtok(0, ",");
		// sprawdzamy czy s¹ parametry, jeli nie to b³¹d
		if(!strlen(wsk)) return -1;
		// ustawiamy kursor w zadanym miejscu y,x
		lcd_locate(y,x);
		// wyœwietlamy tekst
		lcd_str( wsk );


	} else if( 2 == inout ) {
		uart_puts("AT+LCD = (0-3), (0-39),(TEXT)\r\n");
	} else {
		if( !inout ) return -1;
	}*/

	return 0;

}
int8_t at_owndef_service( uint8_t inout, char * params ){
    
    //kod obs³ugi funkcji
    return 0;
}

int8_t at_course_service( uint8_t inout, char * params )
{
	uint16_t course;
	char * wsk;

	if( 1 == inout ) {
		// sprawdzamy czy s¹ parametry, jeli nie to b³¹d
		if(!strlen(params)) return -1;
		// wy³uskujemy pierwszy parametr do przecinka
		wsk = strtok(params, ",");
		// sprawdzamy czy s¹ parametry, jeli nie to b³¹d
		if(!strlen(wsk)) return -1;
		// zamieniamy liczbê ASCII na wartoœæ dziesiêtn¹
		course=atoi(wsk);
		// jeœli course nie mieœci siê w zakresie od 0 do 3599 to b³¹d
		if( !(course>=0 && course<3599) ) return -1;
	    shipparam.requiredCourse = course;


	} else if( 2 == inout ) {
		uart_puts("course= ");
		uart_putint(ownship.course, 10);
		uart_puts("\n\r");
	} else {
		if( !inout ) return -1;
	}
	return 0;
}