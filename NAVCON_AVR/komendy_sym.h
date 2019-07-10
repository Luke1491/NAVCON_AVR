/*
 * komendy_sym.h  
 *
 *  Created on: 12-05-2017
 *      Author: £ukasz Cieplicki based on M.Kardaœ
 */

#ifndef KOMENDY_SYM_H_
#define KOMENDY_SYM_H_



// definicja typu strukturalnego
typedef struct {
	char polecenie_at[11];
	int8_t (* at_service)(uint8_t inout, char * params);
} const TATCMD;


// deklaracje zmiennych zewnêtrznych
extern TATCMD polecenia_at[];


// deklaracje funkcji
void parse_uart_data( char * pBuf );

int8_t at_service(uint8_t inout, char * params);
int8_t ati_service(uint8_t inout, char * params);
int8_t at_lumin_service(uint8_t inout, char * params);
int8_t at_pos_service(uint8_t inout, char * params);
int8_t at_own_service(uint8_t inout, char * params);
int8_t at_test_service( uint8_t inout, char * params );
int8_t at_owndef_service( uint8_t inout, char * params );
int8_t at_course_service( uint8_t inout, char * params );

#endif /* KOMENDY_SYM_H_ */

