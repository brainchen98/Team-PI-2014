/*
	Master code for TEAM PI.
	Created by Brian Chen 03/04/2014
	Last Modified by Brian Chen 15/04/2014 1:38pm

	Changelog:
		0.10 - Initial version. Basic i2c functionality between slave1 and master for TSOP1140s
		0.20 - Added support for MPU9150 using the MPU9150Lib from gitHub
		0.30 - Added partial sensor status support for debugging. LED blinks fast when the teensy detects an error

	Beta 0.30 (C) TEAM PI 2014

	To compile this program for Teensy 3.0 in VS or Atmel Studio with Visual Micro, add the following to the DEFINES PROJECT property
		F_CPU=48000000;USB_SERIAL;LAYOUT_US_ENGLISH
*/

#include "i2c_t3.h"
#include <i2cAnything.h>
#include <cmps10.h>

#include <EEPROM.h>
#include "I2Cdev.h"
#include "MPU9150Lib.h"
#include "CalLib.h"
#include <dmpKey.h>
#include <dmpmap.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>

#include <pwmMotor.h>
#include <omnidrive.h>
#include <debugSerial.h>

//teensy crash "prevention"
#define RESTART_ADDR 0xE000ED0C
#define READ_RESTART() (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))

#define WATCHDOG_INTERVAL 500000	//50 ms


#define LED 13
//debug//////////////////////////////////////////////////////////////
#define DEBUG_SERIAL true
#define DEBUGSERIAL_BAUD 115200

#define BT_TX 0
#define BT_RX 1
#define BT_SERIAL Serial1

/////////////////////////////////////////////////////////////////////
//i2c////////////////////////////////////////////////////////////////
#define I2C_RATE						I2C_RATE_400	//400khz i2c rate
#define SLAVE1_ADDRESS					0x31
#define SLAVE2_ADDRESS					0x32

	//i2c errors
	#define I2C_STAT_SUCCESS				0
	#define I2C_STAT_ERROR_DATA_LONG		1
	#define I2C_STAT_ERROR_RECV_ADDR_NACK	2
	#define I2C_STAT_ERROR_RECV_DATA_NACK	3
	#define I2C_STAT_ERROR_UNKNOWN			4

	//cmps10 error

//commands///////////////////////////////////////////////////////////
#define COMMAND_ANGLE_FLOAT				0
#define COMMAND_ANGLE_ADV_FLOAT			1
#define COMMAND_STRENGTH				2
#define COMMAND_RESULTS					3
#define COMMAND_TSOP_PINS				4

//tsop///////////////////////////////////////////////////////////////
#define TSOP_COUNT						20

//cmps///////////////////////////////////////////////////////////////
#define CMPS_ADDRESS					0x60

//mpu9150////////////////////////////////////////////////////////////

#define DEVICE_TO_USE					0	//0 - 0x68	1 - 0x69

#define MPU_UPDATE_RATE					160
//  MAG_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the magnetometer data
//  MAG_UPDATE_RATE should be less than or equal to the MPU_UPDATE_RATE

#define MAG_UPDATE_RATE					80
#define MPU_LPF_RATE					80	//low pass filter rate between 5 and 188hz

//mpu dmp fusion options
#define  MPU_MAG_MIX_GYRO_ONLY          0                   // just use gyro yaw
#define  MPU_MAG_MIX_MAG_ONLY           1                   // just use magnetometer and no gyro yaw
#define  MPU_MAG_MIX_GYRO_AND_MAG       10                  // a good mix value 
#define  MPU_MAG_MIX_GYRO_AND_SOME_MAG  50                  // mainly gyros with a bit of mag correction 

//motors/////////////////////////////////////////////////////////////
#define MOTOR_PWM_FREQ 16000	//16khz frequency

//motor pins according to Eagle schematic
#define MOTORA_PWM 20
#define MOTORB_PWM 21
#define MOTORC_PWM 22
#define MOTORD_PWM 23

#define MOTORA_BRK 6
#define MOTORB_BRK 17
#define MOTORC_BRK 16
#define MOTORD_BRK 15

#define MOTORA_DIR 2
#define MOTORB_DIR 3
#define MOTORC_DIR 4
#define MOTORD_DIR 5

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

IntervalTimer watchDog;	//timer for watchdog to get out of crash

unsigned long now = 0;
unsigned long lBlinkTime = 0;
uint16_t waitTime = 1000;
bool on = true;

debugSerial dSerial;

//IR/TSOP////////////////////////////////////////////////////////////

float IRAngle = 0;
float IRAngleAdv = 0;
uint8_t IRStrength = 0;
uint8_t IRResults[TSOP_COUNT];

//cmps & mpu/////////////////////////////////////////////////////////
CMPS10 cmps(CMPS_ADDRESS);
float cmpsBearing;

MPU9150Lib mpu;
float mpuBearing;
bool mpuSuccess = true;
unsigned long mpuLSuccessTime;

//omnidrive//////////////////////////////////////////////////////////
PMOTOR motorA(MOTORA_PWM, MOTORA_DIR, MOTORA_BRK, true, MOTOR_PWM_FREQ);
PMOTOR motorB(MOTORB_PWM, MOTORB_DIR, MOTORB_BRK, true, MOTOR_PWM_FREQ);
PMOTOR motorC(MOTORC_PWM, MOTORC_DIR, MOTORC_BRK, true, MOTOR_PWM_FREQ);

OMNIDRIVE robot(motorA, motorB, motorC);

//errors reporting///////////////////////////////////////////////////
struct status {
	uint8_t i2cLine = I2C_STAT_SUCCESS;
	uint8_t cmps = 0;
	uint8_t mpu = 0;
	uint8_t slave1 = 0;
	uint8_t slave2 = 0;
	uint8_t motors = 0;
};

status stat;


void setup()
{
	pinMode(LED, OUTPUT);	
	digitalWrite(LED, HIGH);
	dSerial.begin(DEBUGSERIAL_BAUD);
	initI2C();	
	delay(200);	//short delay to wait for other i2c devices to start
	initMPU();
	delay(50);
	if (!cmps.initialise()){
		//cmps failed to initialise
		stat.cmps = 2;
	}
	//watchDog.begin(reset, WATCHDOG_INTERVAL);
}

void loop()
{	
	unsigned long start = millis();
	for (int i = 0; i < 1000; i++){
		//reset watchdog. If this isn't done, the watchdog will reset the teensy after 50ms
		resetWatchDog();
		timings();
		stat.i2cLine = Wire.getError();
		
		//read slave1	

		stat.slave1 = I2CGet(SLAVE1_ADDRESS, COMMAND_ANGLE_ADV_FLOAT, 4, IRAngleAdv);
		stat.slave1 = I2CGet(SLAVE1_ADDRESS, COMMAND_STRENGTH, 1, IRStrength);

		//read mpu
		mpuSuccess = mpu.read();
		if (mpuSuccess){
			mpuBearing = mpu.m_fusedEulerPose[VEC3_Z] * RAD_TO_DEGREE;
			mpuLSuccessTime = now;
			stat.mpu = 0;
		}
		if (stat.mpu != 0){
			//mpu error. Try and connect to mpu
			initMPU();
		}
		//read cmps
		stat.cmps = cmps.getBearingR(cmpsBearing);

		//check for all errors
		if (!chkErr()){
			//there's an error!
			waitTime = 50;	//fast blinking
		}
		else{
			waitTime = 500;	//medium blinking
		}
#if(DEBUG_SERIAL)
		serialDebug();
		serialStatus();
#endif
	}
	unsigned long finish = millis();
	float hertz = 1000 * 1000 / (finish - start);
}

void serialDebug(){
	dSerial.append("IRAngle:" + String(IRAngle));
	dSerial.append(" IRAngleAdv:" + String(IRAngleAdv));
	dSerial.append(" IRStrength:" + String(IRStrength));
	dSerial.append(" IMU Bearing:" + String(mpuBearing));
	dSerial.append(" CMPS Bearing:" + String(cmpsBearing));
	dSerial.append(" |");
	for (int i = 0; i < TSOP_COUNT; i++){
		dSerial.append(String(IRResults[i]) + " ");
	}
	dSerial.writeBuffer();
}

void serialStatus(){
	dSerial.append("Status: ");
	dSerial.append("I2C "); dSerial.append(stat.i2cLine);
	dSerial.append("\tCMPS "); dSerial.append(stat.cmps);
	dSerial.append("\tMPU "); dSerial.append(stat.mpu);
	dSerial.append("\tS1 "); dSerial.append(stat.slave1);
	dSerial.append("\tS2 "); dSerial.append(stat.slave2);
	dSerial.append("\tMotors "); dSerial.append(stat.motors);
	dSerial.writeBuffer();
}

bool chkErr(){
	if (stat.i2cLine != 0){ return false; }
	if (stat.cmps != 0){ return false; }
	if (stat.mpu != 0){ return false; }
	if (stat.slave1 != 0){ return false; }
	if (stat.slave2 != 0){ return false; }
	if (stat.motors != 0){ return false; }
	return true;
}

void timings(){
	now = millis();
	if (now - lBlinkTime >= waitTime){
		if (on){
			//turn off
			digitalWrite(LED, LOW);
			on = false;
		}
		else{
			//turn on
			digitalWrite(LED, HIGH);
			on = true;
		}
		lBlinkTime = now;
	}	
	if (now - mpuLSuccessTime >= 500){
		//trigger mpu error
		stat.mpu = 1;
	}
}

/////////////////////////////////////////////////////////////////////
//i2c functions//////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

//initiate i2c
void initI2C(){
	Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE);	
}

void initMPU(){
	mpu.selectDevice(DEVICE_TO_USE);	
	mpu.useAccelCal(true);
	mpu.useMagCal(true);
	mpu.init(MPU_UPDATE_RATE, 5, MAG_UPDATE_RATE, MPU_LPF_RATE);   // start the MPU
}

/////////////////////////////////////////////////////////////////////
//pid and movement control///////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
//teensy crash/hang handling/////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

//reset function for teensy
inline void reset(){
	WRITE_RESTART(0x5FA0004);
}

//resets the watchDog interrupt
void resetWatchDog(){
	watchDog.end();
	watchDog.begin(reset, WATCHDOG_INTERVAL);
}
