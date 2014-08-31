/*
	Master code for TEAM PI.
	Created by Brian Chen 03/04/2014
	Last Modified by Brian Chen 01/06/2014 4:22pm
	... Forever modified by Brian Chen.

	Changelog:
		0.10 - Initial version. Basic i2c functionality between slave1 and master for TSOP1140s
		0.20 - Added support for MPU9150 using the MPU9150Lib from gitHub
		0.30 - Added partial sensor status support for debugging. LED blinks fast when the
		       teensy detects an error
		0.40 - Added watchdog timer/interrupt (http://en.wikipedia.org/wiki/Watchdog_timer),
		       requiring the program to reset the interrupt every 50ms (WATCHDOG_INTERVAL).
			   Uses the IntervalTimer class/object built into the Teensy core libraries.
		0.50 - Added chasing ball component
		0.60 - Added loop frequency calculations

	Beta 0.50 (C) TEAM PI 2014

	To compile this program for Teensy 3.0 in VS or Atmel Studio with Visual Micro, add the
	following to the DEFINES PROJECT property
		F_CPU=48000000;USB_SERIAL;LAYOUT_US_ENGLISH
*/

#include <i2c_t3.h>
//#include <Wire.h>
#include <i2cAnything.h>
#include <cmps10.h>
#include <SRF08.h>

//#include <EEPROM.h>
//#include "I2Cdev.h"
//#include "MPU9150Lib.h"
//#include "CalLib.h"
//#include <dmpKey.h>
//#include <dmpmap.h>
//#include <inv_mpu.h>
//#include <inv_mpu_dmp_motion_driver.h>

#include <pwmMotor.h>
#include <omnidrive.h>
#include <debugSerial.h>

//general////////////////////////////////////////////////////////////
#define MAXSPEED 50

//teensy crash/hang handling/////////////////////////////////////////
#define WATCHDOG_ENABLED true
//reset function
#define RESTART_ADDR 0xE000ED0C
#define READ_RESTART() (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))

//debug//////////////////////////////////////////////////////////////
#define LED 13

#define DEBUG_SERIAL true
#define DEBUGSERIAL_BAUD 115200

#define BT_TX 0
#define BT_RX 1
#define BT_SERIAL Serial1

/////////////////////////////////////////////////////////////////////
//i2c////////////////////////////////////////////////////////////////
#define I2C_RATE						I2C_RATE_100	//400khz i2c rate
#define SLAVE1_ADDRESS					0x31
#define SLAVE2_ADDRESS					0x32

//i2c errors
#define I2C_STAT_SUCCESS				0
#define I2C_STAT_ERROR_DATA_LONG		1
#define I2C_STAT_ERROR_RECV_ADDR_NACK	2
#define I2C_STAT_ERROR_RECV_DATA_NACK	3
#define I2C_STAT_ERROR_UNKNOWN			4

//commands///////////////////////////////////////////////////////////
//slave1
#define COMMAND_ANGLE_FLOAT				0
#define COMMAND_ANGLE_ADV_FLOAT			1
#define COMMAND_STRENGTH				2
#define COMMAND_RESULTS					3
#define COMMAND_TSOP_PINS				4
#define COMMAND_BINDEX					5

//slave2
#define COMMAND_LCD_PRINT				0
#define COMMAND_LCD_ERASE				1
#define COMMAND_LCD_LINE				2
#define COMMAND_LCD_RECT				3
#define COMMAND_LSENSOR1				10
#define COMMAND_LSENSOR2				11
#define COMMAND_GOAL_ANGLE				20
#define COMMAND_GOAL_X					21

//tsop///////////////////////////////////////////////////////////////
#define TSOP_COUNT						20

//cmps///////////////////////////////////////////////////////////////
#define CMPS_ADDRESS					0x60

//ultrasonics (SRF08)////////////////////////////////////////////////
#define US_RANGE 200 //range of ultrasonics in cm set un setup code. Improves ultrasonic refresh rate.

#define US_FRONT_ADDRESS 0x70
#define US_RIGHT_ADDRESS 0x71

//mpu9150////////////////////////////////////////////////////////////
#define MPU_ENABLED false
#define RECONNECT_WHEN_LOST				false

#define DEVICE_TO_USE					0	//0 - 0x68	1 - 0x69
#if(DEVICE_TO_USE == 0)
#define MPU_ADDRESS 0x68
#else
#define MPU_ADDRESS 0x69
#endif

#define MPU_UPDATE_RATE					40

//  MAG_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the magnetometer data,
//  should be less than or equal to the MPU_UPDATE_RATE
#define MAG_UPDATE_RATE					40
#define MPU_LPF_RATE					0	//low pass filter rate between 5 and 188hz

//mpu dmp fusion options
#define  MPU_MAG_MIX_GYRO_ONLY          0                   // just use gyro yaw
#define  MPU_MAG_MIX_MAG_ONLY           1                   // just use magnetometer and no gyro yaw
#define  MPU_MAG_MIX_GYRO_AND_MAG       10                  // a good mix value 
#define  MPU_MAG_MIX_GYRO_AND_SOME_MAG  50                  // mainly gyros with a bit of mag correction 

//motors/////////////////////////////////////////////////////////////
#define MOTOR_PWM_FREQ 750000	//16khz frequency

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

#if(WATCHDOG_ENABLED)
IntervalTimer watchDog;	//timer for watchdog to get out of crash
unsigned long WATCHDOG_INTERVAL = 1000000;	//watchdog interval in ms
#endif

unsigned long now = 0;
unsigned long nowMicros = 0, lastTimeMicros = 0;
uint16_t pgmFreq = 0;

unsigned long lBlinkTime = 0;
unsigned long initPrgmTime;
uint16_t waitTime = 1000;
bool on = true;

debugSerial dSerial;

//IR/TSOP////////////////////////////////////////////////////////////

float IRAngle = 0;
float IRAngleAdv = 0;
uint8_t IRStrength = 0;
uint8_t IRResults[TSOP_COUNT];

//ultrasonics////////////////////////////////////////////////////////
SRF08 usFront(US_FRONT_ADDRESS);	//front ultrasonic
SRF08 usRight(US_RIGHT_ADDRESS);	//right ultrasonic
int16_t usFrontRange = 255, usRightRange = 255;
uint8_t usFrontVersion, usRightVersion;

//cmps & mpu/////////////////////////////////////////////////////////
CMPS10 cmps(CMPS_ADDRESS);
float cmpsBearing;
uint16_t cmpsVersion = 255;

//MPU9150Lib mpu;

float mpuBearing;
bool mpuSuccess = true;
unsigned long mpuLSuccessTime;

uint16_t mpuReconnectAttempts = 0;

//omnidrive//////////////////////////////////////////////////////////
PMOTOR motorC(MOTORA_PWM, MOTORA_DIR, MOTORA_BRK, true, MOTOR_PWM_FREQ);
PMOTOR motorB(MOTORB_PWM, MOTORB_DIR, MOTORB_BRK, true, MOTOR_PWM_FREQ);
PMOTOR motorA(MOTORC_PWM, MOTORC_DIR, MOTORC_BRK, true, MOTOR_PWM_FREQ);

OMNIDRIVE robot(motorA, motorB, motorC);

//pid
unsigned long lastPIDTime = 0;
int16_t correction;
float kp = 0.6;
float kd = 1.2;
float error = 0, lerror = 0;
float proportional = 0, derivative = 0;

//orbit
float kOrbit = 1.6; //orbiting constant. Orbit direction = tsop direction * kOrbit. 1 < kOrbit < 3. The greater the value, the larger the orbit.

//goals//////////////////////////////////////////////////////////////
int16_t goalAngle;

//errors reporting///////////////////////////////////////////////////
struct status {
	uint8_t i2cLine = I2C_STAT_SUCCESS;
	uint8_t cmps = I2C_STAT_SUCCESS;
	uint8_t usFront = I2C_STAT_SUCCESS;
	uint8_t usRight = I2C_STAT_SUCCESS;
	uint8_t usLeft = I2C_STAT_SUCCESS;
	uint8_t mpu = I2C_STAT_SUCCESS;
	uint8_t slave1 = I2C_STAT_SUCCESS;
	uint8_t slave2 = I2C_STAT_SUCCESS;
	uint8_t motors = 0;
};

status stat;

//lcd////////////////////////////////////////////////////////////////
int16_t drawCentreX = 100;
int16_t drawCentreY = 100;
int16_t drawLength = 10;

unsigned long lLcdTime;
uint8_t lcdReady = 0;

//light sensors//////////////////////////////////////////////////////
int16_t lightReading1, lightReading2;

//temp///////////////////////////////////////////////////////////////
uint8_t bIndex;

/////////////////////////////////////////////////////////////////////

void nothing(){}

void setup()
{
#if(!DEBUG_SERIAL)
	dSerial.disable();
#endif

	initPrgmTime = millis();
	pinMode(LED, OUTPUT);		
	digitalWrite(LED, LOW);
	delay(25);	
	digitalWrite(LED, HIGH);
	delay(25);
	digitalWrite(LED, LOW);

	dSerial.begin(DEBUGSERIAL_BAUD);
	initI2C();
	//some i2c devices require a few hundred ms delays before they work
	delay(900);
	//now try and see if connected to all i2c devices
	stat.slave1 = checkConnection(SLAVE1_ADDRESS);
	stat.slave2 = checkConnection(SLAVE2_ADDRESS);
	stat.cmps = checkConnection(CMPS_ADDRESS);
	stat.usFront = checkConnection(US_FRONT_ADDRESS);
	stat.usRight = checkConnection(US_RIGHT_ADDRESS);
	stat.mpu = checkConnection(MPU_ADDRESS);
	
	/*if (stat.slave2 == I2C_STAT_SUCCESS){
		lcdErase();
		lcdWrite(5, 5, "S1 Stat: " + String(stat.slave1));
		lcdWrite(5, 20, "S2 Stat: " + String(stat.slave2));
		lcdWrite(5, 35, "CMPS10 Stat: " + String(stat.cmps));
		lcdWrite(5, 50, "US_FRONT Stat: " + String(stat.usFront));
		lcdWrite(5, 65, "MPU9150 Stat: " + String(stat.mpu));
	}*/
#if(MPU_ENABLED)
	if (!initMPU()){
		stat.mpu = 2;
	}
#endif
	
	if (!cmps.initialise()){
		//cmps failed to initialise
		stat.cmps = 2;
	}
	else{
		//success! Get cmps version
		cmpsVersion = cmps.getVersion();
	}
	if (stat.usFront == I2C_STAT_SUCCESS){
		usFront.getSoft(usFrontVersion);
		usFront.setRange(US_RANGE);
		usFront.startRange();
	}
	if (stat.usRight == I2C_STAT_SUCCESS){
		usRight.getSoft(usRightVersion);
		usRight.setRange(US_RANGE);
		usRight.startRange();
	}
}

void mainLoop(){
	//sensors////////////////////////////////////////////////////////////

	//read slave1	
	stat.slave1 = I2CGet(SLAVE1_ADDRESS, COMMAND_ANGLE_FLOAT, 4, IRAngle);
	stat.slave1 = I2CGet(SLAVE1_ADDRESS, COMMAND_ANGLE_ADV_FLOAT, 4, IRAngleAdv);
	stat.slave1 = I2CGet(SLAVE1_ADDRESS, COMMAND_STRENGTH, 1, IRStrength);	
	//stat.slave1 = I2CGet(SLAVE1_ADDRESS, COMMAND_RESULTS, 20 * 1, IRResults);
	//read slave
	//stat.slave2 = I2CGetHL(SLAVE2_ADDRESS, COMMAND_LSENSOR1, lightReading1);
	//stat.slave2 = I2CGetHL(SLAVE2_ADDRESS, COMMAND_LSENSOR2, lightReading2);
	//stat.slave2 = I2CGetHL(SLAVE2_ADDRESS, COMMAND_GOAL_ANGLE, goalAngle);

	//read cmps to get corrected bearing (getBearingR() gets corrected bearing from offset)
	stat.cmps = cmps.getBearingR(cmpsBearing);

	//read ultrasonics
	usFront.autoGetStartIfCan(usFrontRange);
	usRight.autoGetStartIfCan(usRightRange);
	//read mpu
#if(MPU_ENABLED)
	mpuSuccess = mpu.read();
	if (mpuSuccess){
		mpuBearing = mpu.m_fusedEulerPose[VEC3_Z] * RAD_TO_DEGREE;
		mpuLSuccessTime = now;
		stat.mpu = 0;
	}
	else{
#if(RECONNECT_WHEN_LOST)
		//mpu error. Try and connect to mpu
		if (!initMPU()){
			//can't reconnect. Add to number of attempts
			mpuReconnectAttempts++;
		}
#endif
	}
#endif
	//logic//////////////////////////////////////////////////////////////
	if (IRStrength > 130){
		//robot.move(IRAngleAdv, 50, 0);
		//movePIDForward(IRAngleAdv, MAXSPEED, 0);	
		moveOrbit(IRAngleAdv, MAXSPEED, 0);
	}
	else{
		movePIDForward(0, 0, 0);
	}
	/////////////////////////////////////////////////////////////////////

	//check for all errors
	if (!chkErr()){
		//there's an error!
		waitTime = 50;	//fast blinking
	}
	else{
		waitTime = 500;	//medium blinking
	}
	//lcd debug via slave2
	if (now - initPrgmTime > 500 && lcdReady == 0){
		lcdReady = 1;
	}
}

/*
loop() function called in main() - default for Arduino
We don't really want to edit much here. If there is need to add something to the main loop,
add it to mainLoop().
*/
void loop()
{		
	//reset watchdog. If this isn't done, the watchdog will reset the teensy after 50ms
#if(WATCHDOG_ENABLED)
	watchDog.begin(reset, WATCHDOG_INTERVAL);
#endif
	timings();
	//i2c status
	//stat.i2cLine = Wire.getError();
	
	mainLoop();
#if(DEBUG_SERIAL)
	serialDebug();
#endif
#if(WATCHDOG_ENABLED)
	watchDog.end();
#endif
}
	

void serialDebug(){
	dSerial.append("Freq:" + String(pgmFreq));
	dSerial.append(" IRAngle:" + String(IRAngle));
	dSerial.append(" IRAngleAdv:" + String(IRAngleAdv));
	dSerial.append(" IRStrength:" + String(IRStrength));
	dSerial.append(" IMU Bearing:" + String(mpuBearing));
	dSerial.append(" CMPS Bearing:" + String(cmpsBearing));
	dSerial.append(" GoalAngle:" + String(goalAngle));
	dSerial.append(" L1:" + String(lightReading1));
	dSerial.append(" L2:" + String(lightReading2));
	dSerial.append(" usFront:" + String(usFrontRange));
	dSerial.append(" usRight:" + String(usRightRange));
	dSerial.append(" |");
	for (int i = 0; i < TSOP_COUNT; i++){
		dSerial.append(String(IRResults[i]) + " ");
	}
	serialStatus();
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
	lastTimeMicros = nowMicros;
	nowMicros = micros();
	//get program frequency
	pgmFreq = (nowMicros - lastTimeMicros);
	pgmFreq = 1000000 / pgmFreq;

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
	if (lcdReady == 1){
		//first instance of lcd printing
		//lcdErase();
		//lcdWrite(5, 30, "CMPS10 V.\nUS_FRONT V.\nBearing\nIRAngleAdv\nL1\nL2\nusFront\ngoalAngle\npgmFreq\n");
		lcdReady++;
	}
	if (lcdReady > 1 && now - lLcdTime > 150){
		lLcdTime = now;
		//lcdErase();
		//lcdDrawRect(160, 0, 239, 319);
		//printing one large string is much faster than line by line.
		/*lcdWrite(160, 30, String(cmpsVersion) + "\n"
			+ String(usFrontVersion) + "\n"
			+ String(cmpsBearing) + "\n"
			+ String(IRAngleAdv) + "\n"
			+ String(lightReading1) + "\n"
			+ String(lightReading2) + "\n"
			+ String(usFrontRange) + "\n"
			+ String(goalAngle) + "\n"
			+ String(pgmFreq));*/
	}
}

/////////////////////////////////////////////////////////////////////
//i2c functions//////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

//initiate i2c
void initI2C(){
	Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE);	
	//Wire.begin();
}
#if(MPU_ENABLED)
bool initMPU(){
	mpu.selectDevice(DEVICE_TO_USE);	
	mpu.useAccelCal(true);
	mpu.useMagCal(true);
	return mpu.init(MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_ONLY, MAG_UPDATE_RATE, MPU_LPF_RATE);   // start the MPU
}
#endif

uint8_t checkConnection(uint16_t address){
	Wire.beginTransmission(address);
	return Wire.endTransmission();
}

uint8_t lcdWrite(int16_t x, int16_t y, String str){
	Wire.beginTransmission(SLAVE2_ADDRESS);
	Wire.write(COMMAND_LCD_PRINT);
	Wire.write(highByte(x));
	Wire.write(lowByte(x));
	Wire.write(highByte(y));
	Wire.write(lowByte(y));
	Wire.write(str.c_str());
	Wire.endTransmission();
	Wire.requestFrom(SLAVE2_ADDRESS, 1);
	while (Wire.available() == 0);
	return Wire.read();
}

uint8_t lcdErase(){
	Wire.beginTransmission(SLAVE2_ADDRESS);
	Wire.write(COMMAND_LCD_ERASE);
	Wire.endTransmission();
	Wire.requestFrom(SLAVE2_ADDRESS, 1);
	while (Wire.available() == 0);
	return Wire.read();
}

uint8_t lcdDrawLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2){
	Wire.beginTransmission(SLAVE2_ADDRESS);
	Wire.write(COMMAND_LCD_LINE);
	Wire.write(highByte(x1));
	Wire.write(lowByte(x1));
	Wire.write(highByte(y1));
	Wire.write(lowByte(y1));
	Wire.write(highByte(x2));
	Wire.write(lowByte(x2));
	Wire.write(highByte(y2));
	Wire.write(lowByte(y2));
	Wire.endTransmission();
	Wire.requestFrom(SLAVE2_ADDRESS, 1);
	while (Wire.available() == 0);
	return Wire.read();
}

uint8_t lcdDrawRect(int16_t x1, int16_t y1, int16_t x2, int16_t y2){
	Wire.beginTransmission(SLAVE2_ADDRESS);
	Wire.write(COMMAND_LCD_RECT);
	Wire.write(highByte(x1));
	Wire.write(lowByte(x1));
	Wire.write(highByte(y1));
	Wire.write(lowByte(y1));
	Wire.write(highByte(x2));
	Wire.write(lowByte(x2));
	Wire.write(highByte(y2));
	Wire.write(lowByte(y2));
	Wire.endTransmission();
	Wire.requestFrom(SLAVE2_ADDRESS, 1);
	while (Wire.available() == 0);
	return Wire.read();
}

/////////////////////////////////////////////////////////////////////
//pid and movement control///////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

int16_t PDCalc(float bearing, float offset){
	uint16_t dt = millis() - lastPIDTime;
	error = -(offset - bearing);
	proportional = error;
	derivative = (error - lerror)/dt;
	lerror = error;
	return (kp*proportional + kd*derivative);
}

void movePIDForward(float dir, uint8_t speed, float offset){
	correction = PDCalc(cmpsBearing, offset);
	robot.move(dir, speed, correction);	
}

void moveOrbit(float dir, uint8_t speed, float offset){
	if (dir < 90 || dir > 270){
		if (dir < 180){
			movePIDForward(dir * kOrbit, speed, offset);
		}
		else{
			movePIDForward((dir - 360) * kOrbit + 360, speed, offset);
		}
	}
	else{
		if (dir < 180){
			movePIDForward(dir + 45 * kOrbit, speed, offset);
		}
		else{
			movePIDForward(dir - 45 * kOrbit, speed, offset);
		}
	}
}

/////////////////////////////////////////////////////////////////////
//teensy crash/hang handling/////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

#if(WATCHDOG_ENABLED)
//reset function for teensy
void reset(){
	watchDog.begin(reset_2, WATCHDOG_INTERVAL);	
#ifdef DEBUG_SERIAL
	dSerial.print("Resetting");
#endif
	dSerial.println(" ...");
	delay(100);
	WRITE_RESTART(0x5FA0004);
}

void reset_2(){
#ifdef DEBUG_SERIAL
	dSerial.println("Resetting ...");
#endif
	delay(100);
	WRITE_RESTART(0x5FA0004);
}

//resets the watchDog interrupt
void resetWatchDog(){	
	watchDog.end();
	watchDog.begin(reset, WATCHDOG_INTERVAL);
}
#endif