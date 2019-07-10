/*
 * common.h
 *
 * Created: 2017-05-24 20:52:28
 *  Author: LUKE
 */ 


#ifndef COMMON_H_
#define COMMON_H_

typedef struct                  //struktura do przechowywania zmiennych podstawowych
{
	int32_t course;             //aktualny kurs statku w stopniach (pomno�ony przez 10)
	int16_t speed;              //aktualna pr�dko�� dok�adno�� 0,1 kt (pr�dko�� pomno�ona przez 10)
	int32_t posLong;            //d�ugo�� geograficzna pomno�ona przez 6 (bez przecinka)
	int32_t posLat;             //szeroko�� geagraficzna pomno�ona przez 6 (bez przecinka)
	char MMSI[10];              //9 digits in ""
	char NAVSTATUS[2];          //0d 0 do 15
	char PAS[3];
	char TYP_COMMUNIKATU[2];
	char IMO_NUMBER[8];         //7 digits in ""
	char CALL_SIGN[8];
	char SHIP_NAME[21];
	char TYPE_OF_SHIP[3];       // 1 do 99
	char DIM_A[4];              //D�UGO��
	char DIM_B[4];               //
	char DIM_C[4];
	char DIM_D[4];
	uint8_t ETA_month;          //1 d0 12
	uint8_t ETA_day;            //1 do 31
	uint8_t ETA_hour;           //0 do 24
	uint8_t ETA_minute ;        //0 do 59
	char DRAUGHT[5];
	char DESTINATION[21];       //ca�� tablic� wype�nia� do ko�ca znakami @
} OWNSHIP ;
typedef struct
{
	uint8_t lumin;              // warto�ci od 1 do 5 --> intensywno�� �wiecenia di�d
	uint8_t mode;               //tryby dzienny, nocny, itp
	
}SETTING;
typedef struct
{
	uint16_t maxROT;                //maksymalny rate of turn na sekunde(0,01 stopnia/sek)
	uint8_t dROT_po_dT;             //inercja statku (wielko�� zmian ROT w czasie 0,01 stopnia / sek)
	int16_t maxSpeed;               //maksymalna pr�dko�� przy maksymalnej nastawie
	uint16_t speedAccelerationTime;  //czas po kt�rym pr�dko�� zwi�ksz/zmniejszy si� o 0,1 kt (0,01s)
}SHIPMODEL;

typedef struct
{
	int32_t currentROT;	          //dok�adno�� 0,01stopnia np 10-->ROT = 0,1stopnia
	int32_t calculatedROT;             // kierunek ROT aktualny inicjowany przez -5 tylko do pierwszego rozruchu funkcji
	int8_t currentSteerPos;   //aktualne polozenie p�etwy sterowej
	int32_t requiredSpeed;             //rz�dana pr�dko�� w oparciu o nastaw� pr�dko�ci speedSetOnConsole
	int8_t speedSetOnConsole;              //0d -8 do +8 nastawa pr�dko�ci zmieniana dotykowymi przyciskami
	int32_t requiredCourse;
	int8_t requiredSteerPos;    // od -35 do +35
	
	
}SHIPPARAM;


extern SHIPMODEL shipmodel;
extern OWNSHIP ownship;
extern SHIPPARAM shipparam;
extern const OWNSHIP flashOwnship1;
extern const OWNSHIP flashOwnship2;
extern const OWNSHIP flashOwnship3;
extern const SHIPMODEL shipmodel1;
extern const SHIPMODEL shipmodel2;
extern const SHIPMODEL shipmodel3;
extern volatile uint16_t computingParametersTimer, steerReactionTimer, engineReactionTimer, speedSetInManualTimer, buzzerTimer, TouchTimer, roseDiskStepTimer;


extern uint32_t newRoseDiskCourse;
extern int16_t ROTDifference;

extern uint8_t touched;
extern uint16_t prevTouched;
extern uint8_t switchedToAuto;  
extern uint8_t switchedToManual;

void NAVCON_CALIBRATION(void);

void steerInit(void);                              //inicjacja steru
uint16_t readSteerWheelAngleFromADC(void);                   //funkcja dokonuj�ca pomiaru na kanale ADC0 (pin PF0) - mierz�ca wychylenia steru
void convertFrom10bitValueToRequiredSteerAngle(void);              // zmienia shipparam.requiredSteerPos
void displaySteerLed(int8_t steerAngle,uint8_t positive, uint8_t negative);
void moveSteer(int8_t requiredSteerPos);     //symuluje ruch steru (czas uzale�niony od steerReactionTimer

void timerInit(void);                                              //inicjalizacja 8bitowego TIMER0 do precyzyjnej obs�ugi r�nych funkcji

int8_t computeROT(int8_t currentSteerAngle);                  //oblicz rz�dany ROT w oparciu o wychylenie steru oraz charakterystyki statku
uint16_t computeCourse(void);                                         //oblicz kurs na podstawie rot wyliczonego oraz aktualnego
void computeShipParameters(void);   //uruchamiaj dwie powy�sze funkcje co 1s w oparciu o zmienn� timer1

void simulateChangingOfSpeed(void);             //symuluje zmian� pr�dko�ci w oparciu o rz�dan� pr�dko�c oraz timer
void wyswietlenieNastawyPredkosciLED(int16_t rzadanaPredkosc, uint8_t positive, uint8_t negative);              //ustawia zmienn� shipparam.speedSetOnConsole

int8_t autopilotPID(int32_t givenCourse);       //funkcja zmienia rz�dany wychylenie steru na podstawie kursu zadanego oraz ownship.course (kursu aktualnego)
 
 void computePosition(void);
 
void tarczaInit(void);                         //zainicjuj expander I2C pcf8574
void moveRoseDiskByOneStep(uint8_t dir);   //rusz tarcz� o 0,1 stopnia w kierunku zale�nym od dir (1-->prawo   0-->lewo)
void setRoseDiskCourse(void);
void motorCoilOff(void);
void kalibracjaTarczy(void);
void buzzerInit(void);                         //inicjacja buzzera (uzy� na poczatku main)

void Timer2Init(void);                         //inicjacja timera do obs�ugi enkoderow (u�y� na pocz�tku main)
void enkoderyInit(void);                       //inicjacja pin�w jako wwej�cia do enkoderow z podci�daniem (u�yc na pocz�tku main)
void ReadEncoder1();
void ReadEncoder2();
 extern int8_t enc_delta1;
 extern int8_t enc_delta2;

int8_t Read1StepEncoder(uint8_t numerEnkodera);
int8_t Read2StepEncoder(uint8_t numerEnkodera);
int8_t Read4StepEncoder(uint8_t numerEnkodera);
 
void uaktualnijKursAuto(void);       // funkja do odczytu nastawy enkodera i zapisania w zmiennej shipparm.requiredCourse
void uaktualnijPredkoscAuto(void);   // funkja do odczytu nastawy enkodera i zapisania w zmiennej shipparm.requiredSpeed

uint16_t odczytajPrzyciskiGlowne(void);
void dlugoscDzwiekuBuzzer(uint8_t czas); //w�acza buzzer na okres okre�lony w argumencie (wartosc 1 --> 0,01s)
uint16_t odczytajPrzyciskiSterowania(void);

void czujnikTarczyInit(void);            //inicjalizacja czujnika
void sprawdzKursNaTarczy(void);          //musi by� wywo�ywana w p�tli

//########################################################################################################
//ETHERNET FUNCTIONS
void udp_event_callback(uint8_t *peer_ip, uint16_t port, uint16_t datapos, uint16_t len);
void UDP_EVENT(uint16_t *port);
// funkcja do rejestracji funkcji zwrotnej w zdarzeniu UDP_EVENT()
void register_udp_event_callback(void (*callback)(uint8_t *peer_ip,
uint16_t port, uint16_t datapos, uint16_t len));
void (*mk_udp_event_callback)(uint8_t *peer_ip, uint16_t port,
uint16_t datapos, uint16_t len);

void ping_callback(uint8_t *ip);
extern void UDP_EVENT(uint16_t *port);
void prepareAndSendOWNDatagram(void);
//****************************ethernet viariables:**************************************
extern uint8_t mymac[6];
extern uint8_t myip[4];
extern uint16_t myport[];
#define BUFFER_SIZE 650  //bufor for UDP data
extern uint8_t buf[BUFFER_SIZE+1];
extern uint8_t farip[2][4];
extern uint8_t PC_IP[4];
extern char sep[];

#endif /* COMMON_H_ */