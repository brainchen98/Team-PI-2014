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
0.70 - Implemented orbit. Removed mpu9150 due to frequent crashing and inaccurate data.

Beta 0.50 (C) TEAM PI 2014

To compile this program for Teensy 3.0 in VS or Atmel Studio with Visual Micro, add the
following to the DEFINES PROJECT property
F_CPU=48000000;USB_SERIAL;LAYOUT_US_ENGLISH
*/

#include <i2c_t3.h>
//#include <Wire.h>
#include <EEPROM.h>
#include <EEPROMAnything.h>
#include <i2cAnything.h>
#include <cmps10.h>
#include <SRF08.h>

#include <pwmMotor.h>
#include <omnidrive.h>
#include <debugSerial.h>

//general////////////////////////////////////////////////////////////
#define MAXSPEED 255

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

//pid////////////////////////////////////////////////////////////////
#define PID_UPDATE_RATE 25
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////


unsigned long nowMillis = 0;
unsigned long nowMicros = 0, lastLoopTime = 0;
uint16_t pgmFreq = 0;

unsigned long lBlinkTime = 0;
unsigned long initPrgmTime;
uint16_t waitTime = 1000;
bool on = true;


//omnidrive//////////////////////////////////////////////////////////
PMOTOR motorA(MOTORA_PWM, MOTORA_DIR, MOTORA_BRK, true, MOTOR_PWM_FREQ);
PMOTOR motorB(MOTORB_PWM, MOTORB_DIR, MOTORB_BRK, true, MOTOR_PWM_FREQ);
PMOTOR motorC(MOTORC_PWM, MOTORC_DIR, MOTORC_BRK, true, MOTOR_PWM_FREQ);

OMNIDRIVE robot(motorA, motorB, motorC);


void setup()
{
	Serial.begin(9600);
	pinMode(LED, OUTPUT);
}

/*
loop() function called in main() - default for Arduino
We don't really want to edit much here. If there is need to add something to the main loop,
add it to mainLoop().
*/
void loop()
{
	//now move the robot
	unsigned long start = millis();
	while (millis() - start < 2000){
		robot.move(90, MAXSPEED, 0);
	}
	start = millis();
	while (millis() - start < 2000){
		robot.move(90, MAXSPEED, 0);
	}
	timings();
}

void timings(){
	//get times
	nowMillis = millis();
	nowMicros = micros();

	pgmFreq = 1000000 / (nowMicros - lastLoopTime);	//get program frequency
	lastLoopTime = nowMicros;

	if (nowMillis - lBlinkTime >= waitTime){
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
		lBlinkTime = nowMillis;
	}
}