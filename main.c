#include <ctype.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include <avr/wdt.h> 
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include <util/delay.h>

#include "global.h"
#include "../../SEVTlib/uart2.h"
#include "../../SEVTlib/rprintf.h"
#include "../../SEVTlib/can_lib.h"
#include "../../SEVTlib/eleanorcan.h"

#include "emeter.h"
#include "gps.h"
#include "main.h"


/* Things to change for 33s13p pack

HandleBatteryPacks should iterate from BATT to BATTE+1

battdata population using canmsgid in HandleBattMsg

take care of skipped cell in checkBattery

*/

/***************** main() *****************/

int main(void) {

	startup();
	
	while(1) {
	
		handlePrecharge();

		handleCutoffConditions();
		
		//this has been changed to work with the new BMS
		handleBatteryPacks();
		
		handleMotor();
		
		handleEmeter();
		
		handleArrayEmeter();
		
		handleGPS();
		
		handleMPPT();
	
/***************LEGACY FUNCTIONS***********
		//handleRA();

		//handleRearView(); 
*******************************************/

	}
	
	return 1;
}

void startup(void) {
	unsigned char mcusaved = MCUSR;
	MCUSR = 0;

	//Initalize IO
	io_init();
	
	// initialize UART channels for 9600 baud
	uartInit();
	uartSetBaudRate(1,GPS_BAUD_RATE);
	uartSetBaudRate(0,EMETER_BAUD_RATE);
	rprintfInit(uart1SendByte);
	
	
	// Clear MCU status registers
	// MCUSR provides information on which reset source caused an MCU reset
	MCUSR = 0x00; //why do we write again?
	
	
	tonebuff = 0;
	
	

	// initialize CAN
	canInit();
	
	batsampleflag = 0;
	motor_print_flag = 0;
	battery_print_flag = 0;
	motor_i_sum =0.0;
	motor_i_n = 0;
	motor_amphr = 0.0;
	
	//initialize Emeter
	emeterInit(&battery);
	
	//initialize GPS
	gpsInit(&gps);
	
	
	// initialize software status variables
	cutoff_threat_level = 0;
	shutdownmem = 0;
	
	precharge_status = PRECHARGE_STATUS_START;
	battery_status = BATTERY_INIT;
	
	// turn LED Red at start
	turnLEDRed();
	
	timerInit(10);
	
	fanPWM_init();
	
	rprintf("Welcome to Arcturus, bitches. 0x%x\n\n",mcusaved);

	sei();
	
}

void io_init(void){
	PORTA = PORTA_PULLUP_AND_INIT_VAL;
	DDRA  = PORTA_DIRECTION;
	
	PORTB = PORTB_PULLUP_AND_INIT_VAL;
	DDRB  = PORTB_DIRECTION;
	
	PORTC = PORTC_PULLUP_AND_INIT_VAL;
	DDRC  = PORTC_DIRECTION;
	
	PORTD = PORTD_PULLUP_AND_INIT_VAL;
	DDRD  = PORTD_DIRECTION;
	
	PORTE = PORTE_PULLUP_AND_INIT_VAL;
	DDRE  = PORTE_DIRECTION;
	
	PORTF = PORTF_PULLUP_AND_INIT_VAL;
	DDRF  = PORTF_DIRECTION;
	
	PORTG = PORTG_PULLUP_AND_INIT_VAL;
	DDRG  = PORTG_DIRECTION;
}

void handlePrecharge(void) {
	turnLEDGreen();
	
	
	switch(precharge_status) {
	case PRECHARGE_STATUS_START:
		rprintf("$Starting Precharge\n");
		disableMPPT();
		_delay_ms(200);
		
		turnLEDGreen();
		
		turnOnRelay(RELAY_MOTOR_PRECHARGE);
		
		
		precharge_status = PRECHARGE_STATUS_CHARGING;
		break;

	case PRECHARGE_STATUS_CHARGING:
		if (second<=4) {
			playTone(TONE_CHARGING);
			// flush out bad measurements
			cutoff_threat_level = 0;
		} else {
			rprintf("$Fully Charged...after five seconds.\n");
			if (cutoff_threat_level>0) {
				rprintf("$Something's Wrong.  Shutting down.\n");
				shutdown();
			} else { 
				precharge_status = PRECHARGE_STATUS_CHARGED;
			}
		}
		break;
	case PRECHARGE_STATUS_CHARGED:
		rprintf("$Turn On Motor Relay\n");
		turnOnRelay(RELAY_MOTOR);
		_delay_ms(100);
		rprintf("$Turn Off Motor Precharge Relay\n");
		turnOffRelay(RELAY_MOTOR_PRECHARGE);
		rprintf("$Wait 1 second\n");
		_delay_ms(500);
		precharge_status = PRECHARGE_STATUS_ENABLE;
		break;
	case PRECHARGE_STATUS_ENABLE:
		rprintf("\n\n$Precharge Status Enable\n\n");
		enableMPPT();
		
		turnLEDGreen();
		
		precharge_status = PRECHARGE_STATUS_FINISHED;
		break;
	case PRECHARGE_STATUS_FINISHED:
		break;
	default:
		rprintf("E:Invalid precharge state\n");
		break;
	}
}

void handleCutoffConditions(void) {
	if((OFF_SWITCH_PIN & OFF_SWITCH_BV)) { //if Offswitch on Dash is pressed
		shutdownmem++;
	} else {
		shutdownmem = 0;
	}
	if (shutdownmem>3) {
		rprintf("$Shutdown Requested (via Off Switch)\n");
		shutdown();	
	}

	// Precharge Handles its own threat level conditions.. 
	// So only check the threat level if we are no longer precharging
	if (precharge_status == PRECHARGE_STATUS_FINISHED) {
		if(cutoff_threat_level > CUTOFF_THREAT_LEVEL_MAX) {
			rprintf("$Threat Level = %d\n",cutoff_threat_level);
			shutdown();
		} else if(cutoff_threat_level > 0) { 
			turnLEDRed();
			battery_print_flag = 1;
		} else {
			turnLEDGreen();
		}
	}	
}

void handleBatteryPacks(void) {
	unsigned char i,status;

	switch(battery_status) {
	case BATTERY_INIT:
		sample_msg.cmd = CMD_ABORT;
		battdata_rqst.cmd = CMD_ABORT;
		battdata_rcv.cmd = CMD_ABORT;
		can_cmd(&sample_msg);
		can_cmd(&battdata_rqst);
		can_cmd(&battdata_rcv);
		for (i=0; i<32; i++) {
			battdata[i].voltage = 0;
			battdata[i].temp = 0;
			battdata[i].flags = 0;
		}
		if (batsampleflag) {  //bit set every second in timer routine
			battery_status = BATTERY_TRIGGER_SAMPLE;
			batsampleflag = 0;
		}
		break;
	case BATTERY_TRIGGER_SAMPLE:
		for (i=0; i<DATA_MSG_DLC; i++) sample_msg_buff[i]=0;
		sample_msg.pt_data = &sample_msg_buff[0];
		sample_msg.ctrl.ide = 0; //standard 11 bit ID
		sample_msg.dlc = 0; //no data field for remote tx req
		sample_msg.id.std = SAMPLE_BITCHES;
		sample_msg.cmd = CMD_TX_REMOTE;
		if (can_cmd(&sample_msg) != CAN_CMD_ACCEPTED) {
			rprintf("E:Couldn't send sampmsg.\n");
			cutoff_threat_level += BATTERY_BAD_INC;
			playTone(TONE_QUIET_BATTERY);
			battery_status = BATTERY_INIT;
			break;		
		}
		for (i=0; i<100; i++) {
			_delay_us(100);
			status = can_get_status(&sample_msg);
			if (status == CAN_STATUS_COMPLETED) {
				break;
			}
		}
		if (status == CAN_STATUS_COMPLETED) {
			battery_status = BATTERY_AWAIT_DATA;
			batterytimer = 0;
		} else {
			rprintf("E:Sampmsg aborted status = %d.\n",status);
			cutoff_threat_level += BATTERY_BAD_INC;
			playTone(TONE_QUIET_BATTERY);
			battery_status = BATTERY_INIT;
		}
		break;
	case BATTERY_AWAIT_DATA:
		if (batterytimer>5) {
			battery_status = BATTERY_COLLECT_DATA;
			battery_messageid = (BATTA<<BATTBITS);
		}
		break;
	case BATTERY_COLLECT_DATA:
		//set up the data receive MOB
		while (battery_messageid!=(((BATTD+1)<<BATTBITS))) { // iterates through BATTA - BATTD
		
//// For 33s13p pack 	
		//while (battery_messageid!=(((BATTE+1)<<BATTBITS))) { // iterates through BATTA - BATTE
		
/*		Need to also change code in eleanorcan.h
*/
	
			for (i=0; i<DATA_MSG_DLC; i++) {
				battdata_rcv_buff[i] = 0;
			}
			battdata_rcv.pt_data = &battdata_rcv_buff[0];
			battdata_rcv.ctrl.ide = 0;
			battdata_rcv.dlc = DATA_MSG_DLC;
			battdata_rcv.id.std = battery_messageid;
			battdata_rcv.cmd = CMD_RX_DATA_MASKED;
			if (can_cmd(&battdata_rcv) != CAN_CMD_ACCEPTED) {
				rprintf("E:Couldn't create batrcv %d.\n",battery_messageid);
				cutoff_threat_level += BATTERY_BAD_INC;
				playTone(TONE_QUIET_BATTERY);
				battery_status = BATTERY_INIT;
				break;		
			}
			//set up the data request MOBS
			battdata_rqst.pt_data = &battdata_rqst_buff[0];
			battdata_rqst.ctrl.ide = 0;
			battdata_rqst.dlc = DATA_MSG_DLC;
			battdata_rqst.id.std = battery_messageid;
			battdata_rqst.cmd = CMD_TX_REMOTE;
			if (can_cmd(&battdata_rqst) != CAN_CMD_ACCEPTED) {
				rprintf("E:Couldn't create batrqst %d.\n",battery_messageid);
				cutoff_threat_level += BATTERY_BAD_INC;
				playTone(TONE_QUIET_BATTERY);
				battery_status = BATTERY_INIT;
				break;		
			}
			for (i=0; i<100; i++) {
				_delay_us(100);
				status = can_get_status(&battdata_rqst);
				if (status == CAN_STATUS_COMPLETED) {
					break;
				}
			}
			if (status != CAN_STATUS_COMPLETED) {
				rprintf("E:batrqst %d aborted status %d.\n",battery_messageid,status);
				cutoff_threat_level += BATTERY_BAD_INC;
				playTone(TONE_QUIET_BATTERY);
				battery_status = BATTERY_INIT;
				break;
			}
			for (i=0; i<100; i++) {
				_delay_us(100);
				status = can_get_status(&battdata_rcv);
				if (status == CAN_STATUS_COMPLETED) {
					break;
				}
			}
			if (status == CAN_STATUS_COMPLETED) {
				handleBattMsg(battery_messageid);
			} else {
				rprintf("E:batrcv %x aborted.\n",battery_messageid);
				cutoff_threat_level += BATTERY_BAD_INC;
				playTone(TONE_QUIET_BATTERY);
				battery_status = BATTERY_INIT;
				break;
			}
			battery_messageid++;
			if ((battery_messageid&BATTBITMASK)==NUMBATTMSGS) {		//assumes BATTBITS is 4
				battery_messageid >>= BATTBITS;
				battery_messageid++;
				battery_messageid <<= BATTBITS;
			}
		}
		if (battery_status == BATTERY_INIT) {
			break;
		} else {
			checkBattery();
			battery_status = BATTERY_SUCCESS;
		}
		break;
	case BATTERY_SUCCESS:
		if(battery_print_flag){
			printBattData();
			//battery_print_flag = 0;
			
			fanSetSpeed();//Check speed every 2 seconds
		}		
		battery_status = BATTERY_INIT;
		break;
	default:
		rprintf("E:Invalid battery state\n");
		break;
	}
}


//need to change batt data size to 5*8 = 40 
void handleBattMsg(unsigned int canmsgid) {
	unsigned char ind,val;
	unsigned char battid = canmsgid>>BATTBITS;
	canmsgid = canmsgid&BATTBITMASK;

	if (canmsgid<2) {
		ind = (battid&3)*10+canmsgid*4;
		for (val=0; val<4; val++) {
			battdata[ind+val].voltage  = (battdata_rcv_buff[val*2]<<8) | battdata_rcv_buff[val*2+1]; //combining high byte and low byte
		}
	} else if (canmsgid<4) {
		ind = (battid&3)*10+(canmsgid-2)*4;
		for (val=0; val<4; val++) {
			battdata[ind+val].temp  = (battdata_rcv_buff[val*2]<<8) | battdata_rcv_buff[val*2+1];
		}
	} else if (canmsgid<5) {
		ind = (battid&3)*10;
		for (val=0; val<8; val++) {
			battdata[ind+val].flags = battdata_rcv_buff[val];
		}
	} else if (canmsgid<6) {
		ind = (battid&3)*10 + 8;
		for (val=0; val<2; val++) {
			battdata[ind+val].voltage  = (battdata_rcv_buff[val*2]<<8) | battdata_rcv_buff[val*2+1]; //combining high byte and low byte
		}
		ind = (battid&3)*10 + 8;
		for (val=2; val<4; val++) {
			battdata[ind+val-2].temp  = (battdata_rcv_buff[val*2]<<8) | battdata_rcv_buff[val*2+1];
		}
	} else {
		ind = (battid&3)*10 + 8;
		for (val=0; val<2; val++) {
			battdata[ind+val].flags = battdata_rcv_buff[val];		
	}
}
	
}


void checkBattery(void) {


	uint8_t i,flag=0;
	uint8_t filter_flag = 0;
	
	for (i=0; i<BATTERY_NUMCELLS; i++) {
/*#ifdef INDEX_TEST_CELL
		if(i < NUM_CELLS_FOUR_PACKS || i == INDEX_LAST_CELL || i == INDEX_TEST_CELL){
#else
		if(i < NUM_CELLS_FOUR_PACKS || i == INDEX_LAST_CELL){
#endif*/
		
		if(!isSkippedCell(i)){
			if (battdata[i].voltage > MAX_VOLTAGE) {
				
				//add filter for missed batt reading
				if (battdata[i].flags & (CELLFLAG_CSERROR | CELLFLAG_NO_CELL | CELLFLAG_WINDOWRECVERR | CELLFLAG_BADADCLOW)) {
					filter_flag = 1;
				}
				
				else{
					playTone(TONE_MAX_VOLTAGE);
					flag = 1;
				}
				
				rprintf("E:Cell v too high\n");
				rprintf("E:index %d, voltage %d\n", i, battdata[i].voltage);
			}
			
			if (battdata[i].voltage < MIN_VOLTAGE) {
				
				//add filter for missed batt reading
				if (battdata[i].flags & (CELLFLAG_CSERROR | CELLFLAG_NO_CELL | CELLFLAG_WINDOWRECVERR | CELLFLAG_BADADCLOW)) {
					filter_flag = 2;
				}
				
				else{
					playTone(TONE_MIN_VOLTAGE);
					flag = 2;
				}
				
				rprintf("E:Cell v too low\n");
				rprintf("E: index %d, voltage %d\n", i, battdata[i].voltage);
			}
		}
	
		
/*#ifdef INDEX_TEST_CELL
		if(i < NUM_CELLS_FOUR_PACKS){
//		if(i < NUM_CELLS_FOUR_PACKS || i == INDEX_TEST_CELL){
#else
		if(i < NUM_CELLS_FOUR_PACKS){
#endif*/

		if(!(isSkippedCell(i) || i == 29)){  //TODO; remove 29 when the last thermistor is addeds
			if (battdata[i].temp > MAX_TEMP) {
			
				
				//cell communication error
				if (((battdata[i].flags & (CELLFLAG_CSERROR | CELLFLAG_NO_CELL | CELLFLAG_WINDOWRECVERR | CELLFLAG_BADADCLOW)) || battdata[i].temp == 4510)){
					filter_flag = 3;
				}
				
				else {
					playTone(TONE_MAX_TEMP);
					flag = 3;
				}
				
				rprintf("E:Temp too high\n");
				rprintf("E: index %d, temp %d\n", i, battdata[i].temp);
			}
		
			if (battdata[i].temp < MIN_TEMP) {
				//add filter for missed batt reading
				if (battdata[i].flags & (CELLFLAG_CSERROR | CELLFLAG_NO_CELL | CELLFLAG_WINDOWRECVERR | CELLFLAG_BADADCLOW)) {
					filter_flag = 4;
				}
				
				else{
					playTone(TONE_MIN_TEMP);
					flag = 4;
				}
				
				rprintf("E:Temp too low\n");
				rprintf("E: index %d, temp %d\n", i, battdata[i].temp);
			}
		}
		
		if (flag) {
			cutoff_threat_level += BATTERY_VT_INC;
			break;
		}
		
		if (filter_flag) {
			cutoff_threat_level += BATTERY_MISS_INC;
			break;
		}
	}
	
	if ((!flag) && (!filter_flag)) {
		cutoff_threat_level -= BATTERY_OK_DEC;
		if (cutoff_threat_level<0) {
			cutoff_threat_level = 0;
		}
	}
}

void handleMotor(void){
	unsigned char status,i;
	float motorSp;
	float motorSp_2;
	
	status = can_get_status(&motordataAV_msg);
	if (status == CAN_STATUS_COMPLETED) {
		motor_voltage = * ((float*) &motordataAV_msg_buff[0]);
		motor_current = * ((float*) &motordataAV_msg_buff[4]);
		motor_i_sum += motor_current;
		motor_i_n ++;
		for (i=0; i<DATA_MSG_DLC; i++) motordataAV_msg_buff[i]=0;
		motordataAV_msg.pt_data = &motordataAV_msg_buff[0];
		motordataAV_msg.dlc = DATA_MSG_DLC;
		motordataAV_msg.id.std = MOTORDATAAV_MSG_ID;
		motordataAV_msg.cmd = CMD_RX_DATA_MASKED;
		can_cmd(&motordataAV_msg);
	} else if (status == CAN_STATUS_ERROR) {
		for (i=0; i<DATA_MSG_DLC; i++) motordataAV_msg_buff[i]=0;
		motordataAV_msg.pt_data = &motordataAV_msg_buff[0];
		motordataAV_msg.dlc = DATA_MSG_DLC;
		motordataAV_msg.id.std = MOTORDATAAV_MSG_ID;
		motordataAV_msg.cmd = CMD_RX_DATA_MASKED;
		can_cmd(&motordataAV_msg);
	}
	
	status = can_get_status(&motordataSp_msg);
	if (status == CAN_STATUS_COMPLETED) {
		motor_rpm = * ((float*) &motordataSp_msg_buff[0]);
		motorSp = * ((float*) &motordataSp_msg_buff[4]);
		// 2.24 corrects for units from m/s to mph. 1.05 corrects for wheel size.
		//motor_speed = motorSp * 2.24 * 1.0;
		motor_speed = motorSp * 2.24;
		
		for (i=0; i<DATA_MSG_DLC; i++) motordataSp_msg_buff[i]=0;
		motordataSp_msg.pt_data = &motordataSp_msg_buff[0];
		motordataSp_msg.dlc = DATA_MSG_DLC;
		motordataSp_msg.id.std = MOTORDATASP_MSG_ID;
		motordataSp_msg.cmd = CMD_RX_DATA_MASKED;
		can_cmd(&motordataSp_msg);
	} else if (status == CAN_STATUS_ERROR) {
		for (i=0; i<DATA_MSG_DLC; i++) motordataSp_msg_buff[i]=0;
		motordataSp_msg.pt_data = &motordataSp_msg_buff[0];
		motordataSp_msg.dlc = DATA_MSG_DLC;
		motordataSp_msg.id.std = MOTORDATASP_MSG_ID;
		motordataSp_msg.cmd = CMD_RX_DATA_MASKED;
		can_cmd(&motordataSp_msg);
	}
	//
	//status = can_get_status(&motordataAmphr_msg);
	//if (status == CAN_STATUS_COMPLETED) {
		//motor_amphr = * ((float*) &motordataAmphr_msg_buff[0]);
		//
		//for (i=0; i<DATA_MSG_DLC; i++) motordataAmphr_msg_buff[i]=0;
		//motordataAmphr_msg.pt_data = &motordataAmphr_msg_buff[0];
		//motordataAmphr_msg.dlc = DATA_MSG_DLC;
		//motordataAmphr_msg.id.std = MOTORDATAAMPHR_MSG_ID;
		//motordataAmphr_msg.cmd = CMD_RX_DATA_MASKED;
		//can_cmd(&motordataAmphr_msg);
		//} else if (status == CAN_STATUS_ERROR) {
		//for (i=0; i<DATA_MSG_DLC; i++) motordataAmphr_msg_buff[i]=0;
		//motordataAmphr_msg.pt_data = &motordataAmphr_msg_buff[0];
		//motordataAmphr_msg.dlc = DATA_MSG_DLC;
		//motordataAmphr_msg.id.std = MOTORDATAAMPHR_MSG_ID;
		//motordataAmphr_msg.cmd = CMD_RX_DATA_MASKED;
		//can_cmd(&motordataAmphr_msg);
	//}
/*	status = can_get_status(&motordataTemp_msg);
	if (status == CAN_STATUS_COMPLETED) {
		ipm_b_temp = * (float*) &motordataTemp_msg_buff[4]; 
		dsp_temp = * (float*) &motordataTemp_msg_buff[0];
		for (i=0; i<DATA_MSG_DLC; i++) motordataTemp_msg_buff[i]=0;
		motordataTemp_msg.pt_data = &motordataTemp_msg_buff[0];
		motordataTemp_msg.dlc = DATA_MSG_DLC;
		motordataTemp_msg.id.std = MOTORDATATEMP_MSG_ID;
		motordataTemp_msg.cmd = CMD_RX_DATA_MASKED;
		can_cmd(&motordataTemp_msg);
	} else if (status == CAN_STATUS_ERROR) {
		for (i=0; i<DATA_MSG_DLC; i++) motordataTemp_msg_buff[i]=0;
		motordataTemp_msg.pt_data = &motordataTemp_msg_buff[0];
		motordataTemp_msg.dlc = DATA_MSG_DLC;
		motordataTemp_msg.id.std = MOTORDATATEMP_MSG_ID;
		motordataTemp_msg.cmd = CMD_RX_DATA_MASKED;
		can_cmd(&motordataTemp_msg);
	}*/

	#ifdef DBG
	status = can_get_status(&motorFlag_msg);
	if (status == CAN_STATUS_COMPLETED) {
		motor_flag = motorFlag_msg_buff[2];
		motor_limit = motorFlag_msg_buff[0];
		for (i=0; i<DATA_MSG_DLC; i++) motorFlag_msg_buff[i]=0;
		motorFlag_msg.pt_data = &motorFlag_msg_buff[0];
		motorFlag_msg.dlc = DATA_MSG_DLC;
		motorFlag_msg.id.std = MOTORFLAG_MSG_ID;
		motorFlag_msg.cmd = CMD_RX_DATA_MASKED;
		can_cmd(&motorFlag_msg);
	} else if (status == CAN_STATUS_ERROR) {
		for (i=0; i<DATA_MSG_DLC; i++) motorFlag_msg_buff[i]=0;
		motorFlag_msg.pt_data = &motorFlag_msg_buff[0];
		motorFlag_msg.dlc = DATA_MSG_DLC;
		motorFlag_msg.id.std = MOTORFLAG_MSG_ID;
		motorFlag_msg.cmd = CMD_RX_DATA_MASKED;
		can_cmd(&motorFlag_msg);	
	}
	#endif
	
	//new code here//////////////////////////////////////////////////////////////////////////////////////
	//for second motor controller
	status = can_get_status(&motordataAV_msg_2);
	if (status == CAN_STATUS_COMPLETED) {
		motor_voltage_2 = * ((float*) &motordataAV_msg_buff_2[0]);
		motor_current_2 = * ((float*) &motordataAV_msg_buff_2[4]);
		motor_i_sum_2 += motor_current_2;
		motor_i_n_2 ++;
		for (i=0; i<DATA_MSG_DLC; i++) motordataAV_msg_buff_2[i]=0;
		motordataAV_msg_2.pt_data = &motordataAV_msg_buff_2[0];
		motordataAV_msg_2.dlc = DATA_MSG_DLC;
		motordataAV_msg_2.id.std = MOTORDATAAV_MSG_ID_2;
		motordataAV_msg_2.cmd = CMD_RX_DATA_MASKED;
		can_cmd(&motordataAV_msg_2);
	} else if (status == CAN_STATUS_ERROR) {
		for (i=0; i<DATA_MSG_DLC; i++) motordataAV_msg_buff_2[i]=0;
		motordataAV_msg_2.pt_data = &motordataAV_msg_buff_2[0];
		motordataAV_msg_2.dlc = DATA_MSG_DLC;
		motordataAV_msg_2.id.std = MOTORDATAAV_MSG_ID_2;
		motordataAV_msg_2.cmd = CMD_RX_DATA_MASKED;
		can_cmd(&motordataAV_msg_2);
	}
	
	status = can_get_status(&motordataSp_msg_2);
	if (status == CAN_STATUS_COMPLETED) {
		motor_rpm_2 = * ((float*) &motordataSp_msg_buff_2[0]);
		motorSp_2 = * ((float*) &motordataSp_msg_buff_2[4]);
		// 2.24 corrects for units from m/s to mph. 1.05 corrects for wheel size.
		//motor_speed_2 = motorSp_2 * 2.24 * 1.0;
		motor_speed_2 = motorSp_2 * 2.24;
			
		for (i=0; i<DATA_MSG_DLC; i++) motordataSp_msg_buff_2[i]=0;
		motordataSp_msg_2.pt_data = &motordataSp_msg_buff_2[0];
		motordataSp_msg_2.dlc = DATA_MSG_DLC;
		motordataSp_msg_2.id.std = MOTORDATASP_MSG_ID_2;
		motordataSp_msg_2.cmd = CMD_RX_DATA_MASKED;
		can_cmd(&motordataSp_msg_2);
	} else if (status == CAN_STATUS_ERROR) {
		for (i=0; i<DATA_MSG_DLC; i++) motordataSp_msg_buff_2[i]=0;
		motordataSp_msg_2.pt_data = &motordataSp_msg_buff_2[0];
		motordataSp_msg_2.dlc = DATA_MSG_DLC;
		motordataSp_msg_2.id.std = MOTORDATASP_MSG_ID_2;
		motordataSp_msg_2.cmd = CMD_RX_DATA_MASKED;
		can_cmd(&motordataSp_msg_2);
	}
	//
	//status = can_get_status(&motordataAmphr_msg_2);
	//if (status == CAN_STATUS_COMPLETED) {
		//motor_amphr_2 = * ((float*) &motordataAmphr_msg_buff_2[0]);
		//
		//for (i=0; i<DATA_MSG_DLC; i++) motordataAmphr_msg_buff_2[i]=0;
		//motordataAmphr_msg_2.pt_data = &motordataAmphr_msg_buff_2[0];
		//motordataAmphr_msg_2.dlc = DATA_MSG_DLC;
		//motordataAmphr_msg_2.id.std = MOTORDATAAMPHR_MSG_ID_2;
		//motordataAmphr_msg_2.cmd = CMD_RX_DATA_MASKED;
		//can_cmd(&motordataAmphr_msg_2);
		//} else if (status == CAN_STATUS_ERROR) {
		//for (i=0; i<DATA_MSG_DLC; i++) motordataAmphr_msg_buff_2[i]=0;
		//motordataAmphr_msg_2.pt_data = &motordataAmphr_msg_buff_2[0];
		//motordataAmphr_msg_2.dlc = DATA_MSG_DLC;
		//motordataAmphr_msg_2.id.std = MOTORDATAAMPHR_MSG_ID_2;
		//motordataAmphr_msg_2.cmd = CMD_RX_DATA_MASKED;
		//can_cmd(&motordataAmphr_msg_2);
	//}
	//
/*	status = can_get_status(&motordataTemp_msg);
	if (status == CAN_STATUS_COMPLETED) {
		ipm_b_temp = * (float*) &motordataTemp_msg_buff[4]; 
		dsp_temp = * (float*) &motordataTemp_msg_buff[0];
		for (i=0; i<DATA_MSG_DLC; i++) motordataTemp_msg_buff[i]=0;
		motordataTemp_msg.pt_data = &motordataTemp_msg_buff[0];
		motordataTemp_msg.dlc = DATA_MSG_DLC;
		motordataTemp_msg.id.std = MOTORDATATEMP_MSG_ID;
		motordataTemp_msg.cmd = CMD_RX_DATA_MASKED;
		can_cmd(&motordataTemp_msg);
	} else if (status == CAN_STATUS_ERROR) {
		for (i=0; i<DATA_MSG_DLC; i++) motordataTemp_msg_buff[i]=0;
		motordataTemp_msg.pt_data = &motordataTemp_msg_buff[0];
		motordataTemp_msg.dlc = DATA_MSG_DLC;
		motordataTemp_msg.id.std = MOTORDATATEMP_MSG_ID;
		motordataTemp_msg.cmd = CMD_RX_DATA_MASKED;
		can_cmd(&motordataTemp_msg);
	}*/

	#ifdef DBG
	status = can_get_status(&motorFlag_msg_2);
	if (status == CAN_STATUS_COMPLETED) {
		motor_flag_2 = motorFlag_msg_buff_2[2];
		motor_limit_2 = motorFlag_msg_buff_2[0];
		for (i=0; i<DATA_MSG_DLC; i++) motorFlag_msg_buff_2[i]=0;
		motorFlag_msg_2.pt_data = &motorFlag_msg_buff_2[0];
		motorFlag_msg_2.dlc = DATA_MSG_DLC;
		motorFlag_msg_2.id.std = MOTORFLAG_MSG_ID_2;
		motorFlag_msg_2.cmd = CMD_RX_DATA_MASKED;
		can_cmd(&motorFlag_msg_2);
	} else if (status == CAN_STATUS_ERROR) {
		for (i=0; i<DATA_MSG_DLC; i++) motorFlag_msg_buff_2[i]=0;
		motorFlag_msg_2.pt_data = &motorFlag_msg_buff_2[0];
		motorFlag_msg_2.dlc = DATA_MSG_DLC;
		motorFlag_msg_2.id.std = MOTORFLAG_MSG_ID;
		motorFlag_msg_2.cmd = CMD_RX_DATA_MASKED;
		can_cmd(&motorFlag_msg_2);	
	}
	#endif
	//new code end///////////////////////////////////////////////////////////////////


	//print statements for the first motor controller
	if (motor_print_flag){
		rprintf("M1:");
		rprintfFloat(5, (double)motor_voltage);
		rprintf(" ");
		rprintfFloat(5, (double)motor_current);
		rprintf(" "); 
		rprintfFloat(5, (double)motor_speed);
		rprintf(" ");
		rprintfFloat(5, (double)motor_rpm);
		rprintf(" ");
		if(motor_i_n != 0){
			motor_amphr += motor_i_sum / motor_i_n / 1800.0;
		}
		rprintfFloat(6, (double)motor_amphr);
		rprintf(" %d", motor_i_n);
		if (motor_i_n==0) rprintf(" 999");
		rprintf("\n");
		motor_i_sum = 0.0;
		motor_i_n = 0;

		//rprintf("IPM B Temp:"); 
		//rprintfFloat(5, (double)ipm_b_temp);
		//rprintf(" DSP Temp:");
		//rprintfFloat(5, (double)dsp_temp);
		//rprintf("\n");
		#ifdef DBG
		rprintf("$Flag: %x Limit: %x\n", motor_flag, motor_limit); 
		#endif
		motor_print_flag = 0;
		
		//new code start here//////////////////////////////////////////////////////////////////////////////
		//print statements for the second motor controller
		rprintf("M2:");
		rprintfFloat(5, (double)motor_voltage_2);
		rprintf(" ");
		rprintfFloat(5, (double)motor_current_2);
		rprintf(" ");
		rprintfFloat(5, (double)motor_speed_2);
		rprintf(" ");
		rprintfFloat(5, (double)motor_rpm_2);
		rprintf(" ");
		if(motor_i_n_2 != 0){
			motor_amphr_2 += motor_i_sum_2 / motor_i_n_2 / 1800.0;
		}
		rprintfFloat(6, (double)motor_amphr_2);
		rprintf(" %d", motor_i_n_2);
		if (motor_i_n_2==0) rprintf(" 999");
		rprintf("\n");
		motor_i_sum_2 = 0.0;
		motor_i_n_2 = 0;

		//temp left out
		
		#ifdef DBG
		rprintf("$Flag: %x Limit: %x\n", motor_flag_2, motor_limit_2);
		#endif
		motor_print_flag_2 = 0;
		
		double motor_current_sum;
		motor_current_sum = (double)motor_current + (double)motor_current_2;
		rprintf("Motor Current Sum: ");
		rprintfFloat(5, motor_current_sum);
		rprintf("\n");
		//new code end//////////////////////////////////////////////////////
	}
}

void printBattData(void) {
	unsigned char i;
	rprintf("VT:");
	for (i=0; i<BATTERY_NUMRIGHTCELLS; i++) {
//		rprintf(" %d", battdata[i].voltage);
		
		if(!isSkippedCell(i)){
			rprintf(" %d",battdata[i].voltage);
		}
	}
	rprintf("\nVB:");
	for (i=BATTERY_NUMRIGHTCELLS; i<BATTERY_NUMCELLS; i++) {
//		rprintf(" %d", battdata[i].voltage);
		
		if(!isSkippedCell(i)){
			rprintf(" %d",battdata[i].voltage);
		}
	}
	rprintf("\nTT:");
	for (i=0; i<BATTERY_NUMRIGHTCELLS; i++) {
//		rprintf(" %d", battdata[i].temp);
		
		if(!(isSkippedCell(i) || i ==29)){  //TODO; remove 29 when the last thermistor is addeds
			rprintf(" %d",battdata[i].temp);
		}
	}
	rprintf("\nTB:");
	for (i=BATTERY_NUMRIGHTCELLS; i<BATTERY_NUMCELLS; i++) {
//		rprintf(" %d", battdata[i].temp);
		
		if(!(isSkippedCell(i) || i ==29)){  //TODO; remove 29 when the last thermistor is addeds
			rprintf(" %d",battdata[i].temp);
		}
	}
	rprintf("\nF:");
	for (i=0; i<BATTERY_NUMCELLS; i++) {
//		rprintf(" %d", battdata[i].flags);
		if(!isSkippedCell(i)){
			rprintf(" %2x",battdata[i].flags);
		}
	}
	rprintfCRLF();
	
	rprintf("B:%d %d %d %d %d\n",hour,minute,second,tenth,cutoff_threat_level);
}

void handleMPPT(void){
	unsigned char i, status;
	unsigned short iout;
	
	status = can_get_status(&mppt_msg);
	if (status == CAN_STATUS_COMPLETED) {
		iout = (mppt_msg_buff[3]<<8)|mppt_msg_buff[4];
		rprintf("S:%c %d %d %u %d\n", mppt_msg_buff[0], mppt_msg_buff[1], mppt_msg_buff[2], iout, mppt_msg_buff[7]);
		for (i=0; i<DATA_MSG_DLC; i++) mppt_msg_buff[i] = 0;
		mppt_msg.pt_data = &mppt_msg_buff[0];
		mppt_msg.dlc = DATA_MSG_DLC;
		mppt_msg.id.std = MPPT_MSG_ID;
		mppt_msg.cmd = CMD_RX_MPPT_MASKED;
		can_cmd(&mppt_msg);
	} else if (status == CAN_STATUS_ERROR) {
		for (i=0; i<DATA_MSG_DLC; i++) mppt_msg_buff[i] = 0;
		mppt_msg.pt_data = &mppt_msg_buff[0];
		mppt_msg.dlc = DATA_MSG_DLC;
		mppt_msg.id.std = MPPT_MSG_ID;
		mppt_msg.cmd = CMD_RX_MPPT_MASKED;
		can_cmd(&mppt_msg);
	}
}

void handleRA(void) {
if(motor_speed<20) ra_enable();
else ra_disable();

}

void handleRearView(void) {

}

void canInit(void) {
	int i;

	can_init(1);
		
	for (i=0; i<DATA_MSG_DLC; i++) motordataAV_msg_buff[i]=0;
	motordataAV_msg.pt_data = &motordataAV_msg_buff[0];
	motordataAV_msg.dlc = DATA_MSG_DLC;
	motordataAV_msg.id.std = MOTORDATAAV_MSG_ID;
	motordataAV_msg.cmd = CMD_RX_DATA_MASKED;
	can_cmd(&motordataAV_msg);
	
	for (i=0; i<DATA_MSG_DLC; i++) motordataSp_msg_buff[i]=0;
	motordataSp_msg.pt_data = &motordataSp_msg_buff[0];
	motordataSp_msg.dlc = DATA_MSG_DLC;
	motordataSp_msg.id.std = MOTORDATASP_MSG_ID;
	motordataSp_msg.cmd = CMD_RX_DATA_MASKED;
	can_cmd(&motordataSp_msg);
	
	//for (i=0; i<DATA_MSG_DLC; i++) motordataAmphr_msg_buff[i]=0;
	//motordataAmphr_msg_buff.pt_data = &motordataAmphr_msg_buff[0];
	//motordataAmphr_msg.dlc = DATA_MSG_DLC;
	//motordataAmphr_msg.id.std = MOTORDATAAMPHR_MSG_ID;
	//motordataAmphr_msg.cmd = CMD_RX_DATA_MASKED;
	//can_cmd(&motordataAmphr_msg);
		//
	for (i=0; i<DATA_MSG_DLC; i++) motordataTemp_msg_buff[i]=0;
	motordataTemp_msg.pt_data = &motordataTemp_msg_buff[0];
	motordataTemp_msg.dlc = DATA_MSG_DLC;
	motordataTemp_msg.id.std = MOTORDATATEMP_MSG_ID;
	motordataTemp_msg.cmd = CMD_RX_DATA_MASKED;
	can_cmd(&motordataTemp_msg);
	
	for (i=0; i<DATA_MSG_DLC; i++) motorFlag_msg_buff[i]=0;
	motorFlag_msg.pt_data = &motorFlag_msg_buff[0];
	motorFlag_msg.dlc = DATA_MSG_DLC;
	motorFlag_msg.id.std = MOTORFLAG_MSG_ID;
	motorFlag_msg.cmd = CMD_RX_DATA_MASKED;
	can_cmd(&motorFlag_msg);
	
	
	//new code starts here////////////////////////////////////////////////////////////////////
	//for the second motor controller
	for (i=0; i<DATA_MSG_DLC; i++) motordataAV_msg_buff_2[i]=0;
	motordataAV_msg_2.pt_data = &motordataAV_msg_buff_2[0];
	motordataAV_msg_2.dlc = DATA_MSG_DLC;
	motordataAV_msg_2.id.std = MOTORDATAAV_MSG_ID_2;
	motordataAV_msg_2.cmd = CMD_RX_DATA_MASKED;
	can_cmd(&motordataAV_msg_2);
		
	for (i=0; i<DATA_MSG_DLC; i++) motordataSp_msg_buff_2[i]=0;
	motordataSp_msg_2.pt_data = &motordataSp_msg_buff_2[0];
	motordataSp_msg_2.dlc = DATA_MSG_DLC;
	motordataSp_msg_2.id.std = MOTORDATASP_MSG_ID_2;
	motordataSp_msg_2.cmd = CMD_RX_DATA_MASKED;
	can_cmd(&motordataSp_msg_2);
	
	//for (i=0; i<DATA_MSG_DLC; i++) motordataAmphr_msg_buff_2[i]=0;
	//motordataAmphr_msg_buff_2.pt_data = &motordataAmphr_msg_buff_2[0];
	//motordataAmphr_msg_2.dlc = DATA_MSG_DLC;
	//motordataAmphr_msg_2.id.std = MOTORDATAAMPHR_MSG_ID_2;
	//motordataAmphr_msg_2.cmd = CMD_RX_DATA_MASKED;
	//can_cmd(&motordataAmphr_msg_2);
		
	for (i=0; i<DATA_MSG_DLC; i++) motordataTemp_msg_buff_2[i]=0;
	motordataTemp_msg_2.pt_data = &motordataTemp_msg_buff_2[0];
	motordataTemp_msg_2.dlc = DATA_MSG_DLC;
	motordataTemp_msg_2.id.std = MOTORDATATEMP_MSG_ID_2;
	motordataTemp_msg_2.cmd = CMD_RX_DATA_MASKED;
	can_cmd(&motordataTemp_msg_2);
		
	for (i=0; i<DATA_MSG_DLC; i++) motorFlag_msg_buff_2[i]=0;
	motorFlag_msg_2.pt_data = &motorFlag_msg_buff_2[0];
	motorFlag_msg_2.dlc = DATA_MSG_DLC;
	motorFlag_msg_2.id.std = MOTORFLAG_MSG_ID_2;
	motorFlag_msg_2.cmd = CMD_RX_DATA_MASKED;
	can_cmd(&motorFlag_msg_2);
	//new code ends///////////////////////////////////////////////////////////////////////////

	for (i=0; i<DATA_MSG_DLC; i++) mppt_msg_buff[i] = 0;
	mppt_msg.pt_data = &mppt_msg_buff[0];
	mppt_msg.dlc = DATA_MSG_DLC;
	mppt_msg.id.std = MPPT_MSG_ID;
	mppt_msg.cmd = CMD_RX_MPPT_MASKED;
	can_cmd(&mppt_msg);
	
	//setup data buffer for array_emeter messages
	for (i=0; i<DATA_MSG_DLC; i++) array_emeter_msg_buff[i] = 0;
	array_emeter_msg.pt_data = &array_emeter_msg_buff[0];
	array_emeter_msg.dlc = DATA_MSG_DLC;
	array_emeter_msg.id.std = ARRAY_EMETER_MSG_ID;
	array_emeter_msg.cmd = CMD_RX_DATA_MASKED;
	can_cmd(&array_emeter_msg);

	//setup lights/rearview message so we can disable/enable rear view
	for (i=0; i<DATA_MSG_DLC; i++) lights_msg_buff[i] = 0;
	lights_msg.pt_data = &lights_msg_buff[0];
	lights_msg.ctrl.ide = 0;			//standard 
	lights_msg.dlc = LIGHTS_MSG_DLC;
	lights_msg.id.std = LIGHTS_MSG_ID;
	lights_msg.cmd = CMD_RX_DATA_MASKED;
	can_cmd(&lights_msg);
	 
}


void handleEmeter(void) {
	unsigned char c, i;
	for (i=0; i<50; i++) {
		if (uartReceiveByte(EMETER_UART,&c)) {		//possible loss of data if main loop is too long
			emeterProcessInput(&battery,c);		//change to while loop if necessary
		} else {
			break;
		}
		if(battery.hasValidData) {
			battery.hasValidData=0;
			emeterRPrintf(&battery);
			
			
			//TODO: use CURRENT_CHARGE_LIMIT and CURRENT_DISCHARGE_LIMIT macros
			if (battery.deciamps<-300 || battery.deciamps>1500) {
			//if (battery.deciamps<-150 || battery.deciamps>400) {
				playTone(UNSAFE_CURRENT);
				//increment threat level (turn off car immediately)
				cutoff_threat_level += CURRENT_BAD_INC;
				rprintf("E:I > 150 or < -30\n");
			} else {
				cutoff_threat_level -= CURRENT_OK_DEC;
				if (cutoff_threat_level<0) {
					cutoff_threat_level = 0;
				}
			}
			
			//very high current - warning!
			//won't turn off the car, but close
			if (battery.deciamps>1400 || battery.deciamps<-250) {
				playTone(STUPID_CURRENT);
				rprintf("E:I > 140 or < -25 warning\n");
			}
			break;
		}
	}
}



void handleArrayEmeter(void) {
	unsigned char i, status;
	uint16_t emeter_deciamps, emeter_decivolts;
	
	
	uint16_t emeter_amps;
	uint8_t emeter_deci_only;
	
	uint16_t emeter_volts;
	uint8_t emeter_decivolts_only;
	 
	
	status = can_get_status(&array_emeter_msg);
	if (status == CAN_STATUS_COMPLETED) {
		emeter_deciamps = (array_emeter_msg_buff[0]<<8) | array_emeter_msg_buff[1];
		emeter_decivolts = (array_emeter_msg_buff[2]<<8) | array_emeter_msg_buff[3];
		
		
		emeter_deci_only = emeter_deciamps % 10; 
		emeter_amps = emeter_deciamps / 10;
		
		emeter_decivolts_only = emeter_decivolts % 10;
		emeter_volts = emeter_decivolts / 10;
		
		
		rprintf("A:%u.%u %u.%u\n", emeter_amps, emeter_deci_only, emeter_volts, emeter_decivolts_only);
		
		
		//rprintf("A:%u %u\n", emeter_deciamps, emeter_decivolts);
		
		
		for (i=0; i<DATA_MSG_DLC; i++) array_emeter_msg_buff[i] = 0;
		array_emeter_msg.pt_data = &array_emeter_msg_buff[0];
		array_emeter_msg.dlc = DATA_MSG_DLC;
		array_emeter_msg.id.std = ARRAY_EMETER_MSG_ID;
		array_emeter_msg.cmd = CMD_RX_DATA_MASKED;
		can_cmd(&array_emeter_msg);
	} else if (status == CAN_STATUS_ERROR) {
		for (i=0; i<DATA_MSG_DLC; i++) array_emeter_msg_buff[i] = 0;
		array_emeter_msg.pt_data = &array_emeter_msg_buff[0];
		array_emeter_msg.dlc = DATA_MSG_DLC;
		array_emeter_msg.id.std = ARRAY_EMETER_MSG_ID;
		array_emeter_msg.cmd = CMD_RX_DATA_MASKED;
		can_cmd(&array_emeter_msg);
	}
	
}



void handleGPS(void) {
	unsigned char c,i;
	for (i=0; i<50; i++) {
		if (uartReceiveByte(GPS_UART,&c)) {
			gpsProcessInput(&gps,c);	
			
		} else {
			break;
		}
		if(gps.hasValidData) {
			gps.hasValidData=0;
			gpsRPrintf(&gps);
		} else {
			break;
		}
	}
}

void enableMPPT(void) {
	MPPT_ENABLE_PORT &= ~MPPT_ENABLE_BV;
}

void disableMPPT(void) {
	MPPT_ENABLE_PORT |= MPPT_ENABLE_BV; // toggles ThreatOff 
}

void playTone(unsigned char tone) {
	if (tonebuff) {
		return;
	}
	tonebuff = tone;
}


//Switched LEDGreen and LEDRed macros for Valkyrie version boards
//due to different green/red direction on bi-directional LED
void turnLEDGreen(void) {
	// make LED+ low
	// make LED- high
	LED_PORT &= ~LED_PLUS_BV;
	LED_PORT |= LED_MINUS_BV;
}

void turnLEDRed(void) {
	// make LED+ high
	// make LED- low
	LED_PORT |= LED_PLUS_BV;
	LED_PORT &= ~LED_MINUS_BV;	
}

void turnOnRelay(unsigned char relay) {
	switch(relay) {
		case RELAY_MOTOR_PRECHARGE:
			RELAY_MOTOR_PRECHARGE_PORT |= RELAY_MOTOR_PRECHARGE_BV;
			break;
		case RELAY_MOTOR:
			RELAY_MOTOR_PORT |= RELAY_MOTOR_BV;
			break;
		default:
			rprintf("E:turnOnRelay: Invalid Relay\n");		
	}
	
}

void turnOffRelay(unsigned char relay) {
	switch(relay) {
		case RELAY_MOTOR_PRECHARGE:
			RELAY_MOTOR_PRECHARGE_PORT &= ~RELAY_MOTOR_PRECHARGE_BV;
			break;
		case RELAY_MOTOR:
			RELAY_MOTOR_PORT &= ~RELAY_MOTOR_BV;
			break;
		default:
			rprintf("E:turnOffRelay: Invalid Relay\n");

	}
}

void shutdown(void) {
	unsigned int counter = 0;
	uint8_t i;
	
	cli();
	disableMPPT();
	//_delay_ms(150);		//must be longer than 100ms (delay between sending of motor control messages)
	for (i=0;i<10;i++){		//delay 1.2 s
		_delay_ms(12);
	}
	
	turnOffRelay(RELAY_MOTOR);
	turnOffRelay(RELAY_MOTOR_PRECHARGE);
	BUZZER_PORT |= BUZZER_BV;
	turnLEDRed();

	while(1) {
		counter++;
		rprintf("$Shutting Down %d\n",counter);
	}
}
 
/* set up a timer interrupt routine to update buzzer, and clock */
void timerInit(int hertz) {
	tenth = 0;
	second = 0;
	minute = 0;
	hour = 0;

	// Configure timer1 for CTC mode
	TCCR1B |= (1 << WGM12);							// configure timer 1 for CTC mode (p 138)
	TCCR1B |= (1 << CS12);								// start timer at 8000000/256 = 31250 Hz (p 139)
	TIMSK1 |= (1 << OCIE1A);							// Enable CTC interrupt
	// set up a .04 second timer for motor and buzzing functions
	// .04 seconds is 25 Hz, which means we want timer to reset at 31250/25 = 1250 cycles
	OCR1A = 31250/hertz;
	TCNT1 = 0;
}

void timerService(void) {
	static uint8_t sample_counter = 0;
	
	if(sample_counter == 28)
	{
		sample_counter = 0;
		motor_print_flag = 1;
		batsampleflag = 1;
	}
	else
	{
		sample_counter++;
	}
	
	
	batterytimer++;
	if (tenth==9) {
		tenth = 0;
		if(second==5 || second==11 || second==17 || second==23 || second==29 || 
			second==35 || second==41 || second==47 || second==53 || second==59){
			battery_print_flag = 1;
		}
		if (second==59) {
			second = 0;
			if (minute==59) {
				minute = 0;
				hour++;
			} else {
				minute++;
			}
		} else {
			second++;
		}
	} else {
		tenth++;
		if (tenth&1) {
			if (tonebuff) {
				if (tonebuff&1) {
					BUZZER_PORT |= BUZZER_BV;
					LED_PORT ^= LED_PLUS_BV;
					LED_PORT ^= LED_MINUS_BV;
				} else {
					BUZZER_PORT &= ~BUZZER_BV;
					LED_PORT ^= LED_PLUS_BV;
					LED_PORT ^= LED_MINUS_BV;
				}
				tonebuff>>=1;
			} else {
				BUZZER_PORT &= ~BUZZER_BV;
			}
		}
	}
}

/* set up PWM output to control fans */
void fanPWM_init(void) {
	//OCR0A is PINB 7, must have DDR set to output
	//configure timer0 for fast PWM mode
	TCCR0A |=
	(1 << WGM01) |                // configure timer 0 for fast PWM (p 106)
	(1 << WGM00) |
	(1 << COM0A1) |                // for fast PWM mode: Clear OC0A on compare match. Set OC0A at TOP
	(0 << COM0A0) |                    //non-inverter  PWM
	
	//(0 << COM0A1) |                // OCOA toggles on each compare match for 50% duty cycle
	//(1 << COM0A0) |
	
	
	(0 << CS02) |                // select clk_I/O with no prescaler for 8000000/256 = 31250 Hz
	(0 << CS01) |
	(1 << CS00);
	
	//todo: comment back in
	//OCR0A = 128;                         //
	OCR0A = 0;
	
	//TCNT0 = 0;                    // start pwm at 50% (128/256)
	
	//test fan overcurrent
	//do sweep of fins
	
	
	
}

void fanSetSpeed()
{
	int avgTemp = 0;
	double speedPercent;
	int speed;
	
	for (int i=0; i<BATTERY_NUMCELLS; i++)				//Check all temps to find maximum 
	{
		if((!(isSkippedCell(i) || i ==29)) && battdata[i].temp != 4510) {  //TODO; remove 29 when the last thermistor is addeds
			avgTemp += battdata[i].temp;
		}
	}
	avgTemp /= BATTERY_NUMCELLS - NUM_OF_SKIPPEDCELLS - 1; //remove when thermistor in
	
	if(avgTemp <= 250 )
	{
		speedPercent = 50;
	}
	else if(avgTemp > 250 && avgTemp < 400){
		speedPercent = ((avgTemp-250)/150.)*50+50;
	}
	else
	{
		speedPercent = 100;
	}
	/*
	if(maxTemp <= 350 )
	{
		OCR0A =  128;
	}
	else if(maxTemp > 350 && maxTemp <= 400)
	{
		OCR0A = 150;
	}
	else if(maxTemp > 400 && maxTemp < 430){
		OCR0A = 192;
	}
	else
	{
		OCR0A = 255;
	}
	*/	
	
	//changed to be lower than 255 so fans don't draw too much power
	//speed = (int) ((175 / 100.)*speedPercent);
	speed = (int) ((190 / 100.)*speedPercent);
	OCR0A = speed;
	rprintf("avgTemp: %d\nOCR0A: %d\n",avgTemp,OCR0A);
}

uint8_t isSkippedCell(uint8_t index)
{
	uint8_t i;

	for(i = 0; i < NUM_OF_SKIPPEDCELLS; i++ )
	{
		if(index == skippedCellIdx[i])
		{
			return 1;
		}
	}
	
	return 0;
}

ISR (TIMER1_COMPA_vect, ISR_NOBLOCK) {
	timerService();
}
