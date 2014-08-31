#include <i2c_t3.h>
#include "I2Cdev.h"
#include "MPU9150Lib.h"
#include "CalLib.h"
#include <dmpKey.h>
#include <dmpmap.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <EEPROM.h>

//  DEVICE_TO_USE selects whether the IMU at address 0x68 (default) or 0x69 is used
//    0 = use the device at 0x68
//    1 = use the device at ox69

#define LED 13

#define  DEVICE_TO_USE    0

MPU9150Lib MPU;                                              // the MPU object

//  MPU_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the sensor data and DMP output

#define MPU_UPDATE_RATE  (100)

//  MAG_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the magnetometer data
//  MAG_UPDATE_RATE should be less than or equal to the MPU_UPDATE_RATE

#define MAG_UPDATE_RATE  (50)

//  MPU_MAG_MIX defines the influence that the magnetometer has on the yaw output.
//  The magnetometer itself is quite noisy so some mixing with the gyro yaw can help
//  significantly. Some example values are defined below:

#define  MPU_MAG_MIX_GYRO_ONLY          0                   // just use gyro yaw
#define  MPU_MAG_MIX_MAG_ONLY           1                   // just use magnetometer and no gyro yaw
#define  MPU_MAG_MIX_GYRO_AND_MAG       10                  // a good mix value 
#define  MPU_MAG_MIX_GYRO_AND_SOME_MAG  50                  // mainly gyros with a bit of mag correction 

//  MPU_LPF_RATE is the low pas filter rate and can be between 5 and 188Hz

#define MPU_LPF_RATE   50

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

#define  SERIAL_PORT_SPEED  115200

unsigned long now = 0;
unsigned long then = 0;
uint16_t waitTime = 500;
bool on = true;

void setup()
{
	pinMode(LED, OUTPUT);
	Serial.begin(SERIAL_PORT_SPEED);
	Serial.print("Arduino9150 starting using device "); Serial.println(DEVICE_TO_USE);
	Wire.begin();
	delay(100);
	MPU.selectDevice(DEVICE_TO_USE);                        // only really necessary if using device 1
	MPU.init(MPU_UPDATE_RATE, 15, MAG_UPDATE_RATE, MPU_LPF_RATE);   // start the MPU
	
}

void loop()
{
	timings();
	MPU.selectDevice(DEVICE_TO_USE);                         // only needed if device has changed since init but good form anyway
	if (MPU.read()) {                                        // get the latest data if ready yet
		waitTime = 100;

		//  MPU.printQuaternion(MPU.m_rawQuaternion);              // print the raw quaternion from the dmp
		//  MPU.printVector(MPU.m_rawMag);                         // print the raw mag data
		//  MPU.printVector(MPU.m_rawAccel);                       // print the raw accel data
		//  MPU.printAngles(MPU.m_dmpEulerPose);                   // the Euler angles from the dmp quaternion
		//  MPU.printVector(MPU.m_calAccel);                       // print the calibrated accel data
		//  MPU.printVector(MPU.m_calMag);                         // print the calibrated mag data
		MPU.printAngles(MPU.m_fusedEulerPose);                 // print the output of the data fusion
		Serial.println();
	}
	else{
		waitTime = 500;
	}
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