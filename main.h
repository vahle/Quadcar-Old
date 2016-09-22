#ifndef _MAIN_H_
#define _MAIN_H_
#define DBG

/* Things to change for 33s13p pack 

SKIPPED_CELL should take care of 7 empty tinys of 5th BMS

#define BATTERY_NUMRIGHTCELLS				17
#define BATTERY_NUMLEFTCELLS					16

battdata should be 40 element struct:

struct {
    unsigned short voltage;
    signed short temp;
	unsigned char flags;
} battdata[40];




*/



//Function prototypes

void startup(void);
void io_init(void);
void handlePrecharge(void);
void handleCutoffConditions(void);
void handleBatteryPacks(void);
void handleEmeter(void);
void handleGPS(void);
void handleMotor(void);
void handleRA(void);
void handleArrayEmeter(void);
void handleRearView(void);

void shutdown(void);

void playTone(unsigned char);
void turnLEDGreen(void);
void turnLEDRed(void);

void enableMPPT(void);
void disableMPPT(void);
void handleMPPT(void);
void turnOnRelay(unsigned char relay);
void turnOffRelay(unsigned char relay);
void timerInit(int hertz);
uint8_t isSkippedCell(uint8_t index);

void canInit(void);
void timerService(void);
void fanPWM_init(void);

void handleBattMsg(unsigned int canmsgid);
void checkBattery(void);
void printBattData(void);
void fanSetSpeed(void);


//Constants
#define PRECHARGE_STATUS_START				1
#define PRECHARGE_STATUS_CHARGING			2
#define PRECHARGE_STATUS_CHARGED			3
#define PRECHARGE_STATUS_ENABLE				4
#define PRECHARGE_STATUS_FINISHED			5

#define PORTA_DIRECTION						0b00011110                                                        
#define PORTA_PULLUP_AND_INIT_VAL			0b11100011                

//#define PORTB_DIRECTION						0b01100000
//#define PORTB_PULLUP_AND_INIT_VAL			0b10111111

//change OCRO0A to output for PWM FAN test 6/27/3014
#define PORTB_DIRECTION                        0b11100000
#define PORTB_PULLUP_AND_INIT_VAL            0b00111111

#define PORTC_DIRECTION						0b00000010                                           
#define PORTC_PULLUP_AND_INIT_VAL			0b11111111         

#define PORTD_DIRECTION						0b10101000                                            
#define PORTD_PULLUP_AND_INIT_VAL			0b01010111 

#define PORTE_DIRECTION						0b00000000
#define PORTE_PULLUP_AND_INIT_VAL			0b11111111

#define PORTF_DIRECTION						0b00001010
#define PORTF_PULLUP_AND_INIT_VAL			0b11110101 

#define PORTG_DIRECTION						0b11111                                           
#define PORTG_PULLUP_AND_INIT_VAL			0b00000

#define LED_PORT							PORTA
#define LED_PIN								PINA
#define LED_PLUS_BV							BV(1)
#define LED_MINUS_BV						BV(2)

#define OFF_SWITCH_PIN						PINC
#define OFF_SWITCH_BV						BV(4)

#define RELAY_MOTOR							1
#define RELAY_MOTOR_PORT					PORTF
#define RELAY_MOTOR_BV						BV(1)

#define RELAY_MOTOR_PRECHARGE				3
#define RELAY_MOTOR_PRECHARGE_PORT			PORTF
#define RELAY_MOTOR_PRECHARGE_BV			BV(3)

#define MPPT_ENABLE_PORT					PORTC
#define MPPT_ENABLE_BV						BV(1)

#define CUTOFF_THREAT_LEVEL_MAX				100
#define CUTOFF_THREAT_LEVEL_INC				25
#define CUTOFF_THREAT_LEVEL_DEC				5

#define TONE_QUIET_BATTERY					0b10000000
#define TONE_MAX_VOLTAGE					0b10100000
#define TONE_MIN_VOLTAGE					0b10101000
#define TONE_MAX_TEMP						0b10101010
#define TONE_MIN_TEMP						0b10101010
#define TONE_CHARGING						0b11100000
#define STUPID_CURRENT						0b11011000
#define UNSAFE_CURRENT						0b11111100

#define BUZZER_PORT							PORTA
#define BUZZER_BV							(BV(4)|BV(3))

#define BATTERY_INIT						0
#define BATTERY_TRIGGER_SAMPLE				1
#define BATTERY_COLLECT_DATA				2
#define BATTERY_AWAIT_DATA					3
#define BATTERY_SUCCESS						9

/*change to 17 and 16 for 33s13p pack */
#define BATTERY_NUMRIGHTCELLS				20
#define BATTERY_NUMLEFTCELLS				20
#define BATTERY_NUMCELLS					40 // (BATTERY_NUMRIGHTCELLS+BATTERY_NUMLEFTCELLS)
#define NUM_OF_SKIPPEDCELLS					7
const uint8_t skippedCellIdx[NUM_OF_SKIPPEDCELLS] = {4, 9, 14, 19, 24, 34, 39};
#define NUM_CELLS_FOUR_PACKS				32
#define INDEX_LAST_CELL						38
//#define INDEX_TEST_CELL						35

// cell flag mask
#define CELLFLAG_SHUNTON							0x01
#define CELLFLAG_BADADCHIGH							0x02
#define CELLFLAG_BADADCLOW							0x04
#define CELLFLAG_BADWINDOW							0x08
#define CELLFLAG_NO_CELL							0x10	// if master doesn't receive anything from the cell. this bit is set by the master (at90can)
#define CELLFLAG_BADTEMP							0x20
#define CELLFLAG_WINDOWRECVERR						0x40	// if the cell receives a wrong window from master. this bit is set by the master (at90can)
#define CELLFLAG_CSERROR							0x80	// if the cell communication to the master is corrupted, this bit is set by the master (at90can)


#define MAX_VOLTAGE 						4200
#define MIN_VOLTAGE 						2750
#define MAX_TEMP	 						500
#define MIN_TEMP	 						100
#define BATTERY_BAD_INC 					34
#define BATTERY_VT_INC						100
#define BATTERY_OK_DEC 						5
#define CURRENT_BAD_INC 					100
#define CURRENT_OK_DEC 						5
#define CURRENT_DISCHARGE_LIMIT				400
#define CURRENT_CHARGE_LIMIT				120
#define BATTERY_MISS_INC					5			//for filtering 20 4510 messages (missing data)

#define MOTOR_INIT							0
#define MOTOR_START							1
#define MOTOR_LISTEN						2
#define MOTOR_SCUCCESS						3

#define ra_enable()							(PORTG &= 0b11111110)
#define ra_disable()						(PORTG |= 0b00000001)

//#define SKIPPED_CELL						20
//do we need skipped cell?

//global variables
short							cutoff_threat_level;
unsigned char 						shutdownmem;		//this gets initialized to zero

float motor_current;
float motor_voltage;
float motor_speed;
float motor_rpm;
float ipm_b_temp;
float dsp_temp;
float motor_i_sum;
unsigned int  motor_i_n;
float motor_amphr;
unsigned char motor_flag;
unsigned char motor_limit;
unsigned char motor_print_flag;

//addition of second motor controller
float motor_current_2;
float motor_voltage_2;
float motor_speed_2;
float motor_rpm_2;
float ipm_b_temp_2;
float dsp_temp_2;
float motor_i_sum_2;
unsigned int  motor_i_n_2;
float motor_amphr_2;
unsigned char motor_flag_2;
unsigned char motor_limit_2;
unsigned char motor_print_flag_2;

volatile unsigned char	hour;
volatile unsigned char	minute;
volatile unsigned char	second;
volatile unsigned char	tenth;

volatile unsigned char	tonebuff;
volatile unsigned char	batterytimer;
volatile unsigned char  battery_print_flag;


unsigned char           precharge_status;
unsigned char           battery_status;
unsigned int           	battery_messageid;
volatile unsigned char	batsampleflag;
volatile unsigned char	motorsampleflag;

struct {
    unsigned short voltage;
    signed short temp;
	unsigned char flags;
} battdata[40];


emeterStruct			battery;
gpsStruct			gps;

st_cmd_t			sample_msg;
st_cmd_t			battdata_rcv;
st_cmd_t			battdata_rqst;
st_cmd_t			motordataAV_msg;
st_cmd_t			motordataSp_msg;
st_cmd_t			motordataTemp_msg;
st_cmd_t			motorFlag_msg;
st_cmd_t			motordataAmphr_msg;
st_cmd_t			mppt_msg;
st_cmd_t				array_emeter_msg;
st_cmd_t				lights_msg;

st_cmd_t			motordataAV_msg_2;
st_cmd_t			motordataSp_msg_2;
st_cmd_t			motordataTemp_msg_2;
st_cmd_t			motorFlag_msg_2;
st_cmd_t			motordataAmphr_msg_2;

unsigned char			sample_msg_buff[DATA_MSG_DLC];
unsigned char			battdata_rcv_buff[DATA_MSG_DLC];
unsigned char			battdata_rqst_buff[DATA_MSG_DLC];
unsigned char			motordataAV_msg_buff[DATA_MSG_DLC];
unsigned char			motordataSp_msg_buff[DATA_MSG_DLC];
unsigned char			motordataTemp_msg_buff[DATA_MSG_DLC];
unsigned char			motorFlag_msg_buff[DATA_MSG_DLC];
unsigned char			motordataAmphr_msg_buff[DATA_MSG_DLC];
unsigned char			mppt_msg_buff[DATA_MSG_DLC];
unsigned char			array_emeter_msg_buff[DATA_MSG_DLC];
unsigned char			lights_msg_buff[DATA_MSG_DLC];
volatile uint8_t 		lightsbyte;

//adding second motor controller
unsigned char			motordataAV_msg_buff_2[DATA_MSG_DLC];
unsigned char			motordataSp_msg_buff_2[DATA_MSG_DLC];
unsigned char			motordataTemp_msg_buff_2[DATA_MSG_DLC];
unsigned char			motorFlag_msg_buff_2[DATA_MSG_DLC];
unsigned char			motordataAmphr_msg_buff_2[DATA_MSG_DLC];

#endif
