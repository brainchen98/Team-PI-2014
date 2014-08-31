/*------------------------------------------------------------------------

Master code for TEAM PI.
Created by Brian Chen 03/08/2014
Last Modified by Brian Chen 03/08/2014 9:27:35 AM
... Forever modified by Brian Chen.

Changelog:
	0.10 - Initial version. Basic i2c functionality between slave1 and
			master for TSOP1140s
	0.20 - Added support for MPU9150 using the MPU9150Lib from gitHub
	0.30 - Added partial sensor status support for debugging. LED 
			blinks fast when the teensy detects an error
	0.40 - Added watchdog timer/interrupt. See:
			http://en.wikipedia.org/wiki/Watchdog_timer
			requiring the teensy to reset the interrupt every 50ms 
			(WATCHDOG_INTERVAL). Uses the IntervalTimer class/object built
			into the Teensy core libraries.
	0.50 - Added chasing ball component
	0.60 - Added loop frequency calculations
	0.70 - Implemented orbit. Removed mpu9150 due to frequent crashing and 
			inaccurate data.
	0.80 - Implemented solenoid kicking. Decreased i2c rate to prevent
			crashing.


Beta 0.80 (C) TEAM PI 2014

To compile this program for Teensy 3.0 in VS or Atmel Studio with Visual 
Micro, add the following to the DEFINES PROJECT property:
	F_CPU=48000000;USB_SERIAL;LAYOUT_US_ENGLISH

------------------------------------------------------------------------*/

#include <i2c_t3.h>
//#include <Wire.h>
#include <EEPROM.h>
//#include <EEPROMAnything.h>
#include <i2cAnything.h>
#include <cmps10.h>
#include <SRF08.h>

#include <pwmMotor.h>
#include <omnidrive.h>
#include <debugSerial.h> 

#include <orbit.h>

/*************************************************************************
***** #defines section ***************************************************
*************************************************************************/

/**** general ***********************************************************/
#define MAXSPEED						255
#define KICK_ENABLED					true

#define HIGHVOLT_PIN					11	//15v reference pin.

/**** teensy crash handling *********************************************/
#define WATCHDOG_ENABLED				false
//reset function
#define RESTART_ADDR					0xE000ED0C
#define READ_RESTART()					(*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val)				((*(volatile uint32_t *)RESTART_ADDR) = (val))

/**** debug *************************************************************/
#define LED 13

#define LCD_DEBUG						false

#define DEBUG_SERIAL					false
#define DEBUGSERIAL_BAUD				115200

#define BT_TX							0
#define BT_RX							1
#define BT_SERIAL						Serial1

/**** i2c ***************************************************************/
#define I2C_RATE						I2C_RATE_100	//400khz i2c rate
#define SLAVE1_ADDRESS					0x31			// slave1 address
#define SLAVE2_ADDRESS					0x32			// slave2 address

// i2c errors
#define I2C_STAT_SUCCESS				0
#define I2C_STAT_ERROR_DATA_LONG		1
#define I2C_STAT_ERROR_RECV_ADDR_NACK	2
#define I2C_STAT_ERROR_RECV_DATA_NACK	3
#define I2C_STAT_ERROR_UNKNOWN			4

// i2c commands

// slave1
#define COMMAND_ANGLE_FLOAT				0
#define COMMAND_ANGLE_ADV_FLOAT			1
#define COMMAND_STRENGTH				2
#define COMMAND_RESULTS					3
#define COMMAND_TSOP_PINS				4
#define COMMAND_BINDEX					5

// slave2
#define COMMAND_LCD_PRINT				0
#define COMMAND_LCD_ERASE				1
#define COMMAND_LCD_LINE				2
#define COMMAND_LCD_RECT				3
#define COMMAND_LSENSOR1				10
#define COMMAND_LSENSOR2				11
#define COMMAND_GOAL_ANGLE				20
#define COMMAND_GOAL_X					21
		
/**** solenoid **********************************************************/
#define KICK_SIG						12 		// solenoid signal pin. High releases the relay.
#define KICK_ANA						A0		// solenoid reference pin. Originally max of 5v. Divided to 2.5v.
#define KICK_ANA_REF					2.58 * 1023 / 3.3	// 2.4v/3.3v*1023

/**** TSOP **************************************************************/
#define TSOP_COUNT						20

/**** cmps **************************************************************/
#define CMPS_ADDRESS					0x60
#define CMPS_UPDATE_RATE				75 		// cmps update rate in hertz

/**** ultrasonics *******************************************************/
#define US_RANGE						200 	// range of ultrasonics in cm set un setup code. Improves ultrasonic refresh rate.

#define US_FRONT_ADDRESS				0x70
#define US_RIGHT_ADDRESS				0x71
#define US_LEFT_ADDRESS					0x72

/**** light sensors *****************************************************/

/**** motors ************************************************************/
#define MOTOR_PWM_FREQ					16000	// pwm frequency

// motor pins according to Eagle schematic
#define MOTORA_PWM						20
#define MOTORB_PWM						21
#define MOTORC_PWM						22
#define MOTORD_PWM						23

#define MOTORA_BRK						6
#define MOTORB_BRK						17
#define MOTORC_BRK						16
#define MOTORD_BRK						15

#define MOTORA_DIR						2
#define MOTORB_DIR						3
#define MOTORC_DIR						4
#define MOTORD_DIR						5

/**** PID ***************************************************************/
#define PID_UPDATE_RATE					75

/**** Location **********************************************************/
#define RIGHT 							true
#define LEFT 							false
#define TOP 							true
#define BOTTOM 							false

/*************************************************************************
***** end #defines section ***********************************************
*************************************************************************/

#if(WATCHDOG_ENABLED)
IntervalTimer watchDog;	//timer for watchdog to get out of crash
unsigned long WATCHDOG_INTERVAL = 1000000;	//watchdog interval in ms
#endif

unsigned long nowMillis = 0;
unsigned long nowMicros = 0, lastLoopTime = 0;
uint16_t pgmFreq = 0;

unsigned long lBlinkTime = 0;
unsigned long initPrgmTime;
uint16_t waitTime = 1000;
bool on = true;

debugSerial dSerial;

/*************************************************************************
***** TSOPS **************************************************************
*************************************************************************/

float IRAngle = 0;
float IRAngleAdv = 0;
uint8_t IRStrength = 0;
uint8_t IRResults[TSOP_COUNT];

/*************************************************************************
***** ultrasonics ********************************************************
*************************************************************************/

SRF08 usFront(US_FRONT_ADDRESS);	//front ultrasonic
SRF08 usRight(US_RIGHT_ADDRESS);	//right ultrasonic
SRF08 usLeft(US_LEFT_ADDRESS);	//left ultrasonic

int16_t usFrontRange = 255, usRightRange = 255, usLeftRange = 255;
uint8_t usFrontVersion, usRightVersion, usLeftVersion;

/*************************************************************************
***** cmps ***************************************************************
*************************************************************************/

CMPS10 cmps(CMPS_ADDRESS);
float cmpsBearing;
uint16_t cmpsVersion = 255;

unsigned long lastCMPSTime = 0;

/*************************************************************************
***** Movement ***********************************************************
*************************************************************************/

PMOTOR motorA(MOTORA_PWM, MOTORA_DIR, MOTORA_BRK, true, MOTOR_PWM_FREQ);
PMOTOR motorB(MOTORB_PWM, MOTORB_DIR, MOTORB_BRK, true, MOTOR_PWM_FREQ);
PMOTOR motorC(MOTORC_PWM, MOTORC_DIR, MOTORC_BRK, true, MOTOR_PWM_FREQ);

OMNIDRIVE robot(motorA, motorB, motorC);

float targetBearing = 0; 	// simply where the angle the robot wants to face
float targetDirection = 0;
float ldir = 0;				// last actual direction
uint8_t targetSpeed = MAXSPEED;
uint8_t lSpeed = 0;

unsigned long lastTargetBearingUpdate = 0;

/*************************************************************************
***** PID Control*********************************************************
*************************************************************************/

// pid control for rotation (well PD actually)
unsigned long lastPIDTime = 0;
int16_t rotational_correction; // correction applied to each individual motor

/*
	PD Constants history
	0.6	12	Good but a bit slow. No overshoot
	0.6 3	Extremely good performance at 100 speed. However, at 255, 
				compensation too slow. No overshoot
	0.8 3	A little overshoot. At 255 speed, compensation is still to slow.
	0.8 4	Still a little overshoot.
	0.8	5	Robot spins in circles.
	0.9 3	Still spins in circles but less vigorously.
	0.9 6	Overshoot when error is extremely high. Otherwise no overshoot 
				generally.
	0.9 8	Overshoot when error is extremely high. Otherwise no overshoot 
				generally.
	0.7 10	A little overshoot. Does not spin in circles.
	0.7 9
	0.62	120
	0.62	100
	0.70	130
*/

//float kp = 0.7; //proportional constant. As kp increases, response time decreases
//float kd = 125; //derivative constant. As kd increases, overshoot decreases
// float kp = 0.45;
// float kd = 40;
float kp = 0;
float kd = 0;

float error = 0, lInput = 0;
float proportional = 0, derivative = 0, lDerivative = 0;

/*************************************************************************
***** location ***********************************************************
*************************************************************************/

/*movement limitations. The robot is only allowed to move between these angles.
Values are changed real time depending on the location of the robot.*/
float allowableRangeMin = 0, allowableRangeMax = 360;

/*************************************************************************
***** goals **************************************************************
*************************************************************************/
int16_t goalAngle;

/*************************************************************************
***** solenoid ***********************************************************
*************************************************************************/

bool isKicking = false;
bool kickReady = false;
uint16_t ana = 0;
bool ballInPos = false;

/*************************************************************************
***** errors reporting****************************************************
*************************************************************************/

struct status {
	uint8_t i2cLine = I2C_STAT_SUCCESS;
	uint8_t cmps = I2C_STAT_SUCCESS;
	uint8_t usFront = I2C_STAT_SUCCESS;
	uint8_t usRight = I2C_STAT_SUCCESS;
	uint8_t usLeft = I2C_STAT_SUCCESS;
	uint8_t slave1 = I2C_STAT_SUCCESS;
	uint8_t slave2 = I2C_STAT_SUCCESS;
	uint8_t motors = 0;
};

status stat;

/*************************************************************************
***** lcd ****************************************************************

*************************************************************************/
unsigned long lLcdTime;
uint8_t lcdReady = 0;

/*************************************************************************
***** light sensors ******************************************************
*************************************************************************/

int16_t lightReading1, lightReading2;

/*************************************************************************
***** temp ***************************************************************
*************************************************************************/

unsigned long lineNumber = 0;		// line number for crash debugging
uint8_t previousCrash = 255;

// //
// void mainLoop();
// //
// bool chkErr();
// void chkStatus();
// inline bool blinkLED();
// void timings();
// float PDCalc(float input, float offset);
// inline float diff(float angle1, float angle2);
// inline void movePIDForward(float dir, uint8_t speed, float offset);
// inline float getHalfOrbit(float dir);
// inline float getOrbit_CCW(float dir);
// inline float getOrbit_CW(float dir);
// float getOrbit(float dir);
// inline void kick();
// inline void endKick();
// inline void getKickState();
// inline void initI2C();
// uint8_t checkConnection(uint16_t address);
// uint8_t lcdWrite(int16_t x, int16_t y, String str);
// uint8_t lcdErase();
// uint8_t lcdDrawLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2);
// uint8_t lcdDrawRect(int16_t x1, int16_t y1, int16_t x2, int16_t y2);
// void reset();
// void reset_2();
// void resetWatchDog();
// inline void bearingTo360(float &bearing);
// inline void bearingTo180(float &bearing);
// void serialDebug();
// void serialStatus();
/*************************************************************************
***** Start of main code**************************************************
*************************************************************************/

void setup()
{
	Serial1.begin(115200);
	Serial.begin(9600);
#if(!DEBUG_SERIAL)
	dSerial.disable();
#endif
	initPrgmTime = millis();
	pinMode(LED, OUTPUT);			// set led pin to output
	pinMode(HIGHVOLT_PIN, INPUT);	// set the 15v-> 3.3v pin to input
	pinMode(KICK_ANA, INPUT);		// set kick_ana pin to input
	analogReadAveraging(4);			// get more accurate analog reads with averaging
	pinMode(KICK_SIG, OUTPUT);		// set kick_sig to output

	// startup blink. Easily distinguishable.
	digitalWrite(LED, LOW);
	delay(25);
	digitalWrite(LED, HIGH);
	delay(25);
	digitalWrite(LED, LOW);

	dSerial.begin(DEBUGSERIAL_BAUD);

	initI2C();

	delay(1000);						// delay to allow cmps10 to initialise
	
	digitalWrite(LED, HIGH);
	// now try and see if connected to all i2c devices
	chkStatus();

#if(LCD_DEBUG)
	if (stat.slave2 == I2C_STAT_SUCCESS){
		lcdErase();
		lcdWrite(5, 5, "S1 Stat: " + String(stat.slave1));
		lcdWrite(5, 20, "S2 Stat: " + String(stat.slave2));
		lcdWrite(5, 35, "CMPS10 Stat: " + String(stat.cmps));
		lcdWrite(5, 50, "US_FRONT Stat: " + String(stat.usFront));
	}
#endif

	// initialise ultrasonics
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
	if (stat.usLeft == I2C_STAT_SUCCESS){
		usLeft.getSoft(usLeftVersion);
		usLeft.setRange(US_RANGE);
		usLeft.startRange();
	}

	digitalWrite(LED, LOW);
	
	// calibrate cmps10 offset with cmps.initialise()
	if (stat.cmps == I2C_STAT_SUCCESS){
		if (!cmps.initialise()){
			// cmps failed to initialise
			stat.cmps = 2;
		}
		else{
			// success! Get cmps version
			cmpsVersion = cmps.getVersion();			
		}
	}
	// wait for 15v to turn on
	// while (digitalRead(HIGHVOLT_PIN) == LOW){ 
	// 	nowMillis = millis();
	// 	//special blink signal
	// 	if (blinkLED()){
	// 		if (waitTime != 100){
	// 			waitTime = 100;
	// 		}
	// 		else{
	// 			waitTime = 800;
	// 		}
	// 	}
	// 	dSerial.println("waiting for 15v");
	// }
}

void mainLoop(){
	lineNumber = __LINE__;
	/*************************************************************************
	***** sensors ************************************************************
	*************************************************************************/
	chkStatus();	// check i2c status

	lineNumber = __LINE__;
	// read slave1	
	if (stat.slave1 == I2C_STAT_SUCCESS){
		lineNumber = __LINE__;
		stat.slave1 = I2CGet(SLAVE1_ADDRESS, COMMAND_ANGLE_FLOAT, 4, IRAngle);
		lineNumber = __LINE__;
		stat.slave1 = I2CGet(SLAVE1_ADDRESS, COMMAND_ANGLE_ADV_FLOAT, 4, IRAngleAdv);
		lineNumber = __LINE__;
		stat.slave1 = I2CGet(SLAVE1_ADDRESS, COMMAND_STRENGTH, 1, IRStrength);

		// stat.slave1 = I2CGet(SLAVE1_ADDRESS, COMMAND_RESULTS, 20 * 1, IRResults);
	}
	lineNumber = __LINE__;
	// read slave2
	if (stat.slave2 == I2C_STAT_SUCCESS){
		stat.slave2 = I2CGetHL(SLAVE2_ADDRESS, COMMAND_LSENSOR1, lightReading1);
		stat.slave2 = I2CGetHL(SLAVE2_ADDRESS, COMMAND_LSENSOR2, lightReading2);
		stat.slave2 = I2CGetHL(SLAVE2_ADDRESS, COMMAND_GOAL_ANGLE, goalAngle);
	}
	lineNumber = __LINE__;
	// read ultrasonics
	if (stat.usFront == I2C_STAT_SUCCESS){
		usFront.autoGetStartIfCan(usFrontRange);
	}
	if (stat.usRight == I2C_STAT_SUCCESS){
		usRight.autoGetStartIfCan(usRightRange);
	}
	lineNumber = __LINE__;

	/*************************************************************************
	***** logic** ************************************************************
	*************************************************************************/
	
	lineNumber = __LINE__;
	ballInPos = false;
	bool inAngle = ((IRAngleAdv > 345 && IRAngleAdv <= 360)) || (IRAngleAdv < 15 && IRAngleAdv >= 0);

	if ((IRStrength > 145) && inAngle){
		ballInPos = true;
	}
	if (IRStrength > 110){
		if(IRAngleAdv > 0){
			// ball on right. Must do a right orbit
			targetDirection = getOrbit(IRAngleAdv);
		}
		else{
			// ball on left. Must do a left orbit
			targetDirection = getOrbit(IRAngleAdv);
		}
		// targetDirection = IRAngleAdv;
		targetSpeed = MAXSPEED;
	}
	else if (IRStrength > 40){
		targetDirection = IRAngleAdv;
		targetSpeed = MAXSPEED;
	}
	else{
		// ball not found
		targetDirection = 0;
		targetSpeed = 0;
	}
	// now move the robot
	movePIDForward(targetDirection, targetSpeed, targetBearing);

	/*************************************************************************
	***** kicking ************************************************************
	*************************************************************************/

	getKickState();		// check kick state
	if (isKicking){
		digitalWrite(KICK_SIG, HIGH);
	}
	else{
		digitalWrite(KICK_SIG, LOW);
	}
	if (kickReady && !isKicking && ballInPos){
		kick();
	}
	else if (ana < 10 && isKicking){
		endKick();
	}

	lineNumber = __LINE__;

	/*************************************************************************/

	// check for all errors
	if (!chkErr()){
		waitTime = 50;	// fast blinking
	}
	else{
		waitTime = 500;	// medium blinking
	}
	// lcd debug via slave2
	if (nowMillis - initPrgmTime > 500 && lcdReady == 0){
		lcdReady = 1;
	}
	lineNumber = __LINE__;
}

/* basic loop structure. Like the "bones" of the program */
void loop()
{
	
#if(WATCHDOG_ENABLED)
	/* reset watchdog. If this isn't done, the watchdog will reset the 
	teensy after WATCHDOG_INTERVAL us */
	watchDog.begin(reset, WATCHDOG_INTERVAL);
#endif
	timings();
	mainLoop();
#if(DEBUG_SERIAL)
	serialDebug();
#endif
#if(WATCHDOG_ENABLED)
	watchDog.end();
#endif
}

bool chkErr(){
	if (stat.i2cLine != 0){ return false; }
	if (stat.cmps != 0){ return false; }
	if (stat.slave1 != 0){ return false; }
	if (stat.slave2 != 0){ return false; }
	if (stat.motors != 0){ return false; }
	return true;
}

void chkStatus(){
	stat.i2cLine = Wire.status();
	stat.slave1 = checkConnection(SLAVE1_ADDRESS);
	stat.slave2 = checkConnection(SLAVE2_ADDRESS);
	stat.cmps = checkConnection(CMPS_ADDRESS);
	stat.usFront = checkConnection(US_FRONT_ADDRESS);
	stat.usRight = checkConnection(US_RIGHT_ADDRESS);
	stat.usLeft = checkConnection(US_LEFT_ADDRESS);
}

inline bool blinkLED(){
	if (nowMillis - lBlinkTime >= waitTime){
		if (on){
			// turn off
			digitalWrite(LED, LOW);
			on = false;
		}
		else{
			// turn on
			digitalWrite(LED, HIGH);
			on = true;
		}
		lBlinkTime = nowMillis;
		return true;
	}
	return false;
}

void timings(){
	// get time	
	nowMicros = micros();
	nowMillis = nowMicros / 1000;

	pgmFreq = 1000000 / (nowMicros - lastLoopTime);	//get program frequency
	lastLoopTime = nowMicros;

	blinkLED();
	
	if (lcdReady == 1){
		// first instance of lcd printing
		// lcdErase();
		// lcdWrite(5, 30, "CMPS10 V.\nUS_FRONT V.\nBearing\nIRAngleAdv\nL1\nL2\nusFront\ngoalAngle\npgmFreq\n");
		lcdReady++;
	}
	if (lcdReady > 1 && nowMillis - lLcdTime > 150){
		lLcdTime = nowMillis;
		// lcdErase();
		// lcdDrawRect(160, 0, 239, 319);
		// printing one large string is much faster than line by line.
		// lcdWrite(160, 30, String(cmpsVersion) + "\n"
		// 	+ String(usFrontVersion) + "\n"
		// 	+ String(cmpsBearing) + "\n"
		// 	+ String(IRAngleAdv) + "\n"
		// 	+ String(lightReading1) + "\n"
		// 	+ String(lightReading2) + "\n"
		// 	+ String(usFrontRange) + "\n"
		// 	+ String(goalAngle) + "\n"
		// 	+ String(pgmFreq));
	}
	if (nowMillis - lastCMPSTime > 1000 / CMPS_UPDATE_RATE){
		// read cmps to get corrected bearing (getBearingR() gets corrected bearing from offset)
		stat.cmps = cmps.getBearingR(cmpsBearing);
	}
	
	/*if (nowMillis - lastTargetBearingUpdate > 100){
	targetBearing = cmpsBearing + goalAngle * 1.2;
	while (targetBearing < 0){ targetBearing += 360; }
	while (targetBearing >= 360){ targetBearing -= 360; }
	}*/
}

/*************************************************************************
***** PD and movement control ********************************************
*************************************************************************/

float PDCalc(float input, float offset){
	
	//unsigned long dt = micros() - lastPIDTime;
	//float output;
	//int cycleSpeed = 5;
	//float instDerivative = 0;
	//if (dt > 1000000 / PID_UPDATE_RATE){	//only compute pid at desired rate
	//	error = -(offset - input);
	//	//make sure error is an element of (-180,180]
	//	if (error <= -180){ error += 360; }
	//	if (error >= 180){ error -= 180; }
	//	proportional = error;
	//	instDerivative = (input - lInput) * 1000 / dt; //derivative = d/dx(error) when offset = 0. However, derivative = d/dx(input) regardless of setpoint
	//	derivative = instDerivative / 2 + lDerivative / 2;
	//	lDerivative = instDerivative;
	//	lInput = input;
	//	lastPIDTime = micros();

	//}
	//if (error > 90) {
	//	output = 80;
	//}
	//else if (error < -90) {
	//	output = -80;
	//}
	//else {
	//	output = (proportional - derivative * cycleSpeed)*0.6;

	//}
	///*
	//if ((error > 90 || error < -90) && derivative < 10){
	//output = (kp*proportional);
	//}
	//else{
	//output = (kp*proportional + kd*derivative);
	//}
	//*/
	///*BT_SERIAL.print("Output:" + String(output));	BT_SERIAL.print(",");	BT_SERIAL.print(String(derivative * kd));	BT_SERIAL.print(",");	BT_SERIAL.println(String(output));*/
	//if (output > 80){ output = 80; }
	//if (output < -80){ output = -80; }
	//if ((output < 10) && (output > -10)){ output = 0; }


	//return output;
	
	unsigned long dt = micros() - lastPIDTime;
	float output;
	if (dt > 1000000 / PID_UPDATE_RATE){	// only compute pid at desired rate
		error = -(offset - input);
		// make sure error ϵ (-180,180]
		if (error <= -180){ error += 360; }
		if (error >= 180){ error -= 180; }
		proportional = error;
		// derivative = d/dx(error) when offset = 0. However, derivative = d/dx(input) regardless of setpoint
		derivative = (input - lInput) * 1000 / dt; 
		// moving average filter for derivative.
		derivative = derivative / 2 + lDerivative / 2;
		lDerivative = derivative;
		lInput = input;
		lastPIDTime = micros();
		
	}
	if ((error > 90 || error < -90) && derivative < 10){
		output = (kp*proportional + kd*derivative);
	}
	else{
		output = (kp*proportional);
	}
	if (output > 255){ output = 255; }
	if (output < -255){ output = -255; }
	return output;
	
}

// get the smallest difference between two bearings.
inline float diff(float angle1, float angle2){
	if (angle1 - angle2 > 180){
		return (angle1 - angle2 - 360);
	}
	else if (angle1 - angle2 < -180){
		return (angle1 - angle2 + 360);
	}
	return (angle1 - angle2);
}

inline void movePIDForward(float dir, uint8_t speed, float offset){
	float difference = diff(dir, ldir);

	
	// speed control to prevent slipping
	if (abs(difference) >= 120){
		speed = 0;
	}
	else{
		speed = speed*(120 - abs(difference))/120;
	}

	// acceleration/deceleration

	/*if (speed - lSpeed > 3){
		speed = lSpeed + 3;
	}
	else if (speed - lSpeed < -3){
		speed = lSpeed - 3;
	*/}

	// if (speed != 0){
	// 	if (difference > 1){
	// 		dir = ldir + 1;
	// 	}
	// 	else if (difference < 1){
	// 		dir = ldir - 1;
	// 	}
	// }
	

	Serial.print(diff(dir, ldir));
	Serial.print("dir proper: " + String(dir) + "\t");
	if (dir < 0){
		dir += 360;
	}
	if (dir > 360){
		dir -= 360;
	}
	Serial.println("dir after: " + String(dir));
	ldir = dir;
	lSpeed = speed;
	rotational_correction = PDCalc(cmpsBearing, offset);
	robot.move(dir, speed, rotational_correction);
}

// Foundation of other orbit commands. Expected dir to be between 0 and 180.
inline float getHalfOrbit(float dir){
	if (dir >= 0 && dir <= 10){
		//leave direction as it is
	}
	else if (dir > 10 && dir <= 40){
		dir += 20;
	}
	else if (dir > 40 && dir < 90){
		dir += 55;
	}
	else if (dir > 90 && dir <= 180){
		dir += 80;
	}
	return dir;
}

inline float getOrbit_CCW(float dir){
	bearingTo180(dir);
	if (dir >= 0){
		dir = getHalfOrbit(dir);
	}
	bearingTo180(dir);
	return dir;
}

inline float getOrbit_CW(float dir){
	bearingTo180(dir);
	if (dir < 0){
		dir = -getHalfOrbit(-dir);
	}
	bearingTo180(dir);
	return dir;
}

float getOrbit(float dir){
	bearingTo180(dir);
	if (dir >= -10 && dir <= 10){
		//leave direction as it is
	}
	else if (dir > 10 && dir <= 20){
		dir += 10;
	}
	else if (dir > 20 && dir <= 40){
		dir += 25;
	}
	else if (dir > 40 && dir < 90){
		dir += 55;
	}
	else if (dir > 90 && dir <= 180){
		dir += 80;
	}
	else if (dir < -10 && dir >= -40){
		dir -= 20;
	}
	else if (dir < -40 && dir > -90){
		dir -= 55;
	}
	else if (dir <= -90 && dir > -180){
		dir -= 80;
	}
	bearingTo360(dir);
	//Serial.println("dir: " + String(dir));
	return dir;
}

/*************************************************************************
***** solenoid ***********************************************************
*************************************************************************/

inline void kick(){
	digitalWrite(KICK_SIG, HIGH);
	isKicking = true;
}

inline void endKick(){
	digitalWrite(KICK_SIG, LOW);
	isKicking = false;
}

inline void getKickState(){
	ana = analogRead(KICK_ANA);
	if (ana > KICK_ANA_REF){
		// ready to kick
		kickReady = true;
	}
	else{
		kickReady = false;
	}
}

/*************************************************************************
***** i2c ****************************************************************
*************************************************************************/

//initiate i2c
inline void initI2C(){
	Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE);
	//Wire.begin();
}

uint8_t checkConnection(uint16_t address){
	Wire.beginTransmission(address);
	return Wire.endTransmission();
}

/*************************************************************************
***** i2c lcd ************************************************************
*************************************************************************/

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



/*************************************************************************
***** teensy crash handling***********************************************
*************************************************************************/

#if(WATCHDOG_ENABLED)
//reset function for teensy
void reset(){
	watchDog.begin(reset_2, WATCHDOG_INTERVAL);
#ifdef DEBUG_SERIAL
	dSerial.print("Resetting");
	dSerial.print("Crash at line " + String(lineNumber));
	dSerial.println(" ...");
#endif
	EEPROM.write(1, (uint8_t)1);
	delay(100);
	WRITE_RESTART(0x5FA0004);
}

void reset_2(){
#ifdef DEBUG_SERIAL
	dSerial.println("Resetting 2 ...");
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

/*************************************************************************
***** angular functions **************************************************
*************************************************************************/

inline void bearingTo360(float &bearing){
	while (bearing < 0){
		bearing += 360;
	}
	while (bearing >= 360){
		bearing -= 360;
	}
}

inline void bearingTo180(float &bearing){
	while (bearing <= -180){
		bearing += 360;
	}
	while (bearing > 180){
		bearing -= 360;
	}
}

/*************************************************************************
***** serial debugging *** ***********************************************
*************************************************************************/

void serialDebug(){
	dSerial.append("Freq:" + String(pgmFreq));
	dSerial.append("\tIRAngle:" + String(IRAngle));
	dSerial.append("\tIRAngleAdv:" + String(IRAngleAdv));
	dSerial.append("\tIRStrength:" + String(IRStrength));
	dSerial.append("\tCMPS Bearing:" + String(cmpsBearing));
	dSerial.append("\tTargetBearing:" + String(targetBearing));
	dSerial.append("\tTargetDirection:" + String(targetDirection));
	dSerial.append("\tTargetSpeed:" + String(targetSpeed));
	dSerial.append("\tGoalAngle:" + String(goalAngle));
	dSerial.append("\tL1:" + String(lightReading1));
	dSerial.append("\tL2:" + String(lightReading2));
	dSerial.append("\tusFront:" + String(usFrontRange));
	dSerial.append("\tusRight:" + String(usRightRange));
	dSerial.append(" |");
	for (uint8_t i = 0; i < TSOP_COUNT; i++){
		dSerial.append(String(IRResults[i]) + " ");
	}
	serialStatus();
	dSerial.writeBuffer();
}

void serialStatus(){
	dSerial.append("Status: ");
	dSerial.append("I2C:"); dSerial.append(stat.i2cLine);
	dSerial.append("\tCMPS:"); dSerial.append(stat.cmps);
	dSerial.append("\tS1:"); dSerial.append(stat.slave1);
	dSerial.append("\tS2:"); dSerial.append(stat.slave2);
	dSerial.append("\tMotors:"); dSerial.append(stat.motors);
}