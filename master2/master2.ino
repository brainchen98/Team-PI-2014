/*
Master code for TEAM PI.
Created by Brian Chen 03/04/2014
Last Modified by Brian Chen 05/04/2014 8:03am

Beta 0.10 (C) TEAM PI 2014

F_CPU=48000000;USB_SERIAL;LAYOUT_US_ENGLISH
*/

#include "i2c_t3.h"
#include <i2cAnything.h>
#include <cmps10.h>

#include "I2Cdev.h"
#include "MPU9150Lib.h"
#include "CalLib.h"
#include <dmpKey.h>
#include <dmpmap.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <EEPROM.h>

#include <pwmMotor.h>
#include <omnidrive.h>
#include <debugSerial.h>


//debug//////////////////////////////////////////////////////////////
#define DEBUG_SERIAL true
#define DEBUGSERIAL_BAUD 115200

//i2c////////////////////////////////////////////////////////////////
#define I2C_RATE I2C_RATE_100
#define SLAVE1_ADDRESS 0x31

//commands///////////////////////////////////////////////////////////
#define COMMAND_ANGLE_FLOAT 0
#define COMMAND_ANGLE_ADV_FLOAT 1
#define COMMAND_STRENGTH 2
#define COMMAND_RESULTS 3
#define COMMAND_TSOP_PINS 4

//tsop///////////////////////////////////////////////////////////////
#define TSOP_COUNT 20

//cmps///////////////////////////////////////////////////////////////
#define CMPS_ADDRESS 0x60

//mpu9150////////////////////////////////////////////////////////////

#define  DEVICE_TO_USE    0

//  MPU_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the sensor data and DMP output

#define MPU_UPDATE_RATE  (20)

//  MAG_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the magnetometer data
//  MAG_UPDATE_RATE should be less than or equal to the MPU_UPDATE_RATE

#define MAG_UPDATE_RATE  (10)

//  MPU_MAG_MIX defines the influence that the magnetometer has on the yaw output.
//  The magnetometer itself is quite noisy so some mixing with the gyro yaw can help
//  significantly. Some example values are defined below:

#define  MPU_MAG_MIX_GYRO_ONLY          0                   // just use gyro yaw
#define  MPU_MAG_MIX_MAG_ONLY           1                   // just use magnetometer and no gyro yaw
#define  MPU_MAG_MIX_GYRO_AND_MAG       10                  // a good mix value 
#define  MPU_MAG_MIX_GYRO_AND_SOME_MAG  50                  // mainly gyros with a bit of mag correction 

//  MPU_LPF_RATE is the low pas filter rate and can be between 5 and 188Hz

#define MPU_LPF_RATE   40

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port


MPU9150Lib mpu;


/////////////////////////////////////////////////////////////////////

debugSerial dSerial;

float bearing = 0;
float bearingAdv = 0;
uint8_t strength = 0;
uint8_t results[TSOP_COUNT];

CMPS10 cmps(CMPS_ADDRESS);

void setup()
{
	dSerial.begin(DEBUGSERIAL_BAUD);
	//initI2C();
	//initMPU();	
	Wire.begin();
	mpu.selectDevice(DEVICE_TO_USE);                        // only really necessary if using device 1
	mpu.init(MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_AND_MAG, MAG_UPDATE_RATE, MPU_LPF_RATE);   // start the MPU
}

void loop()
{
	I2CGet(SLAVE1_ADDRESS, COMMAND_ANGLE_FLOAT, 4, bearing);
	I2CGet(SLAVE1_ADDRESS, COMMAND_ANGLE_ADV_FLOAT, 4, bearingAdv);		
	I2CGet(SLAVE1_ADDRESS, COMMAND_RESULTS, 20, results);
	I2CGet(SLAVE1_ADDRESS, COMMAND_STRENGTH, 1, strength);

	if (mpu.read()){
		mpu.printAngles(mpu.m_fusedEulerPose);                 // print the output of the data fusion
		Serial.println("HI");
	}
	else{
		//Serial.println("Can't read");
	}

#if(DEBUG_SERIAL)
	//serialDebug();
#endif
}

void serialDebug(){
	dSerial.append("Angle:" + String(bearing));
	dSerial.append(" AngleAdv:" + String(bearingAdv));
	dSerial.append(" Strength:" + String(strength));
	dSerial.append(" |");
	for (int i = 0; i < TSOP_COUNT; i++){
		dSerial.append(String(results[i]) + " ");
	}
	dSerial.append("\n");
	dSerial.writeBuffer();
}

/////////////////////////////////////////////////////////////////////
//i2c functions//////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

//initiate i2c
void initI2C(){
	//Wire.begin();
	Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE);	
}

void initMPU(){
	mpu.selectDevice(DEVICE_TO_USE);
	mpu.init(MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_AND_MAG, MAG_UPDATE_RATE, MPU_LPF_RATE);   // start the MPU
	//mpu.init(MPU_UPDATE_RATE);
}
