/*
 * common.h
 *
 * Created: 2017-05-24 20:52:28
 *  Author: LUKE
 * EDIT: 15.07.2019
 */ 


#ifndef COMMON_H_
#define COMMON_H_

typedef struct               
{
	int32_t course;             //current course (times 10 -> ex: 100 means 10 degrees)
	int16_t speed;              //current speed accuracy 0,1 kt (times 10 -> ex: 100 means 10 knots)
	int32_t posLong;            //longitude times 1000000 (accurancy 0.000001)
	int32_t posLat;              //latitude times 1000000 (accurancy 0.000001)
	char MMSI[10];              //9 digits in ""
	char NAVSTATUS[2];          //0d 0 do 15
	char PAS[3];
	char TYP_COMMUNIKATU[2];
	char IMO_NUMBER[8];         //7 digits in ""
	char CALL_SIGN[8];
	char SHIP_NAME[21];
	char TYPE_OF_SHIP[3];       // 1 do 99
	char DIM_A[4];              //length
	char DIM_B[4];               //width
	char DIM_C[4];
	char DIM_D[4];
	uint8_t ETA_month;          //1 to 12
	uint8_t ETA_day;            //1 to 31
	uint8_t ETA_hour;           //0 to 24
	uint8_t ETA_minute ;        //0 to 59
	char DRAUGHT[5];
	char DESTINATION[21];       //destination name unused array indexes fill with '@'
} OWNSHIP ;
typedef struct
{
	uint8_t lumin;              // 1 to 5 led lumination level
	uint8_t mode;               //mode ed bright day, day, night etc.
	
}SETTING;
typedef struct
{
	uint16_t maxROT;                //max rate of turn per sec(0,01 deg/sec)
	uint8_t dROT_po_dT;             //ship innertion(delta rot 0,01 per sec)
	int16_t maxSpeed;               
	uint16_t speedAccelerationTime;  //speed changin time (by 0,01 kt)
}SHIPMODEL;

typedef struct
{
	int32_t currentROT;	          //accurancy 0,01deg ex: 10-->ROT = 0,1deg
	int32_t calculatedROT;        
	int8_t currentSteerPos;   	 
	int32_t requiredSpeed;        //required spped based on speedSetOnConsole
	int8_t speedSetOnConsole;     //form -8 to +8 manipulated on capacitive touch on console
	int32_t requiredCourse;
	int8_t requiredSteerPos;      // from -35 to +35
	
	
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

void steerInit(void);                      
uint16_t readSteerWheelAngleFromADC(void);                   
void convertFrom10bitValueToRequiredSteerAngle(void);              // edited  shipparam.requiredSteerPos
void displaySteerLed(int8_t steerAngle,uint8_t positive, uint8_t negative);
void moveSteer(int8_t requiredSteerPos);     					   //steer time simulation (depend of steerReactionTimer)

void timerInit(void);      

int8_t computeROT(int8_t currentSteerAngle);                
uint16_t computeCourse(void);                                 
void computeShipParameters(void); 

void simulateChangingOfSpeed(void);             
void showSpeedSettingLED(int16_t _requiredSpeed, uint8_t positive, uint8_t negative);    //changing shipparam.speedSetOnConsole

int8_t autopilotPID(int32_t givenCourse); 
 
void computePosition(void);
uint16_t degreesToLED(uint16_t degrees);
void roseInit(void);                         //initiate I2C pcf8574
void moveRoseDiskByOneStep(uint8_t dir);
void setRoseDiskCourse(void);
void motorCoilOff(void);
void roseCalibration(void);
void buzzerInit(void);                         //call in begin of main

void Timer2Init(void);                         //timer for encoders (call in begin of main)
void encodersInit(void);                       //call in begin of main
void ReadEncoder1();
void ReadEncoder2();
 extern int8_t enc_delta1;
 extern int8_t enc_delta2;

int8_t Read1StepEncoder(uint8_t encoderNumber);
int8_t Read2StepEncoder(uint8_t encoderNumber);
int8_t Read4StepEncoder(uint8_t encoderNumber);
 
void updateCourseAuto(void);         // read encoders and set to shipparm.requiredCourse
void updateSpeedAuto(void);  		 // read encoders and set to shipparm.requiredSpeed

uint16_t readMainButtons(void);
void buzzerSoundTime(uint8_t time);  //buzzer beep time (1 --> 0,01sec)
uint16_t readSteerButtons(void);

void RoseSensorInit(void);            
//void sprawdzKursNaTarczy(void);          //musi by� wywo�ywana w p�tli

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