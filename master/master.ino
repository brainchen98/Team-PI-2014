/*
	Master code for TEAM PI.
	Created by Brian Chen 03/04/2014
	Last Modified by Brian Chen 07/04/2014 3:09pm

	Beta 0.10 (C) TEAM PI 2014

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

#define LED 13
//debug//////////////////////////////////////////////////////////////
#define DEBUG_SERIAL true
#define DEBUGSERIAL_BAUD 115200

//i2c////////////////////////////////////////////////////////////////
#define I2C_RATE						I2C_RATE_400
#define SLAVE1_ADDRESS					0x31

	//i2c errors
	#define I2C_STAT_SUCCESS				0
	#define I2C_STAT_ERROR_DATA_LONG		1
	#define I2C_STAT_ERROR_RECV_ADDR_NACK	2
	#define I2C_STAT_ERROR_RECV_DATA_NACK	3
	#define I2C_STAT_ERROR_UNKNOWN			4

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

//mpu fused euler positions
#define X 0
#define Y 1
#define Z 2
/////////////////////////////////////////////////////////////////////

unsigned long now = 0;
unsigned long then = 0;
uint16_t waitTime = 1000;
bool on = true;

debugSerial dSerial;

//IR/TSOP////////////////////////////////////////////////////////////

float IRAngle = 0;
float IRAngleAdv = 0;
uint8_t IRStrength = 0;
uint8_t IRResults[TSOP_COUNT];

CMPS10 cmps(CMPS_ADDRESS);
float cmpsBearing;

MPU9150Lib mpu;
float mpuBearing;

//reset function for teensy
void(*resetFunc) (void) = 0;	//declare reset function at address 0

void setup()
{
	pinMode(LED, OUTPUT);	
	dSerial.begin(DEBUGSERIAL_BAUD);
	initI2C();
	delay(150);
	initMPU();
	delay(100);	//short delay to wait for other i2c devices to start		
	cmps.initialise();
}

void loop()
{	
	timings();
	//I2CChk_Error();
	
	//read slave1
	//getSlave1();

	//read mpu
	if (mpu.read()){		
		waitTime = 100;
		mpuBearing = mpu.m_fusedEulerPose[VEC3_Z] * RAD_TO_DEGREE;		
	}	
	else{
		waitTime = 500;
	}
	//read cmps
	//cmps.getBearingR(cmpsBearing);
	
	#if(DEBUG_SERIAL)
		serialDebug();
	#endif
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
	dSerial.append("\n");	
	dSerial.writeBuffer();
}

void timings(){
	now = millis();
	if (now - then >= waitTime){
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
		then = now;
	}	
}

/////////////////////////////////////////////////////////////////////
//i2c functions//////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

//initiate i2c
void initI2C(){
	Wire.begin();
	//Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE);	
}

void initMPU(){
	mpu.selectDevice(DEVICE_TO_USE);	
	mpu.useAccelCal(true);
	mpu.useMagCal(true);
	mpu.init(MPU_UPDATE_RATE, 5, MAG_UPDATE_RATE, MPU_LPF_RATE);   // start the MPU
	//mpu.init(MPU_UPDATE_RATE);
}

void I2CChk_Error(){
	uint8_t i2cError = Wire.getError();
	if (i2cError != 0){
		//i2c error
		dSerial.print("I2C error:"); dSerial.print(i2cError); dSerial.print("  ");
		switch (i2cError){
		case I2C_STAT_ERROR_DATA_LONG:
			dSerial.println("I2C_STAT_ERROR_DATA_LONG"); break;
		case I2C_STAT_ERROR_RECV_ADDR_NACK:
			dSerial.println("I2C_STAT_ERROR_RECV_ADDR_NACK"); break;
		case I2C_STAT_ERROR_RECV_DATA_NACK:
			dSerial.println("I2C_STAT_ERROR_RECV_DATA_NACK"); break;
		case I2C_STAT_ERROR_UNKNOWN:
			dSerial.println("I2C_STAT_ERROR_UNKNOWN"); break;
		}
		//Wire.finish();
		//initI2C();
	}
}

void getSlave1(){
	I2CGet(SLAVE1_ADDRESS, COMMAND_ANGLE_FLOAT, 4, IRAngle);
	I2CGet(SLAVE1_ADDRESS, COMMAND_ANGLE_ADV_FLOAT, 4, IRAngleAdv);
	I2CGet(SLAVE1_ADDRESS, COMMAND_RESULTS, 20, IRResults);
	I2CGet(SLAVE1_ADDRESS, COMMAND_STRENGTH, 1, IRStrength);
}