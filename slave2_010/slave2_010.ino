/*
	Slave2 code for TEAM PI. Slave2 has an i2c address of 0x32.
	Created by Brian Chen 07/06/2014
	Last Modified by Brian Chen 07/06/2014 7:28pm
	... Forever modified by Brian Chen.

	Beta 0.10 (C) TEAM PI 2014

	To compile this program for Teensy 3.0 in VS or Atmel Studio with Visual Micro, add the
	following to the DEFINES PROJECT property
	F_CPU=48000000;USB_SERIAL;LAYOUT_US_ENGLISH
*/

#include <SPI.h>  
#include <Pixy.h>
#include <SRF08.h>
#include <i2c_t3.h>
#include <i2cAnything.h>
#include <debugSerial.h>
#include <SMARTGPU2.h>

#define LED 					13
#define LED_ENABLED 			false

#define DEBUG_SERIAL 			true
#define DEBUGSERIAL_BAUD 		115200

//slavei2c///////////////////////////////////////////////////////////
#define I2C_ADDRESS 			0x32
#define I2C_RATE 				I2C_RATE_100

//lcd////////////////////////////////////////////////////////////////
#define LCD_ENABLED 			false
#define LCD_FREQ 				10 //frequency of lcd refreshes
#define LCD_WAIT_TIME 			1000/LCD_FREQ     
#define LCD_BAUDRATE 			115200

//light sensors//////////////////////////////////////////////////////
#define LSENSOR_PIN1 			A9 //right led
#define LSENSOR_PIN2 			A8 //left led

//solenoid///////////////////////////////////////////////////////////
#define SOLENOID_ENABLED 		false
#define SOLENOID_SIG 			20
#define SOLENOID_ANA 			A7 //ANA pin gives off a voltage of 0.32*VIN when fully charged
#define SOLENOID_ANA_REFERENCE 	0.32*16

//camera/////////////////////////////////////////////////////////////
#define PIXY_ENABLED 			true
#define PIXY_ELEVATION 			11.4	//height of pixy above ground

#define BLUE_GOAL				1
#define YELLOW_GOAL				2

//commands///////////////////////////////////////////////////////////
#define COMMAND_LCD_PRINT		0
#define COMMAND_LCD_ERASE		1
#define COMMAND_LCD_LINE		2
#define COMMAND_LCD_RECT		3
#define COMMAND_LSENSOR1		10
#define COMMAND_LSENSOR2		11
#define COMMAND_GOAL_ANGLE		20
#define COMMAND_GOAL_X			21
#define COMMAND_KICK			30

/////////////////////////////////////////////////////////////////////

SMARTGPU2 lcd;

bool on = true;	//led on

unsigned long lUnlockTime, lLCDTime, lBlinkTime = 0, lRequestTime, lEraseTime, now;
unsigned long nowMicros = 0, lastTimeMicros = 0;
uint16_t pgmFreq = 0;

uint16_t ledWaitTime = 500;	//led waitTime
uint16_t requestTimeMax = 500;	//max time before i2c not being received

//debug stuff////////////////////////////////////////////////////////
debugSerial dSerial;

//i2c slave stuff////////////////////////////////////////////////////
char command[200]; //accept commands of up to 200 bytes (for long strings)
uint16_t strLength;
uint8_t b, bIndex;
int16_t x_1, y_1, x_2, y_2; 

//light sensors//////////////////////////////////////////////////////
int16_t lightReading1, lightReading2;

//camera/////////////////////////////////////////////////////////////
Pixy pixy;

uint8_t pixyCount = 0;
uint16_t blocks = 0;
uint16_t goalX = 0, goalY = 0;
uint16_t goalWidth = 0, goalHeight = 0;
int16_t goalAngle = 1000;	//0 degrees is forward
uint16_t goalDistance = 0;	//goal distance in mm
uint8_t goalColour = BLUE_GOAL;

unsigned long lDetectGoal = 0;
bool goalDetected = false;

//solenoid///////////////////////////////////////////////////////////
bool isKicking = false;
unsigned long lKickTime = 0;

/////////////////////////////////////////////////////////////////////


void setup(){
	initI2C();

	pinMode(LSENSOR_PIN1, INPUT);
	pinMode(LSENSOR_PIN2, INPUT);

	dSerial.begin(DEBUGSERIAL_BAUD);
#if(!DEBUG_SERIAL)
	dSerial.disable();
#endif

#if(LED_ENABLED)
	pinMode(LED, OUTPUT);
	digitalWrite(LED, HIGH);
#endif	

#if(LCD_ENABLED)
	if (lcdInit()){
		dSerial.println("SMARTGPU2 initiation successful");
	}
	lLCDTime = micros();
#endif	

#if(PIXY_ENABLED)
	pixy.init();
#endif

	lUnlockTime = micros();
}

void loop(){
	//timings
	timings();
	//light sensors
	lightReading1 = analogRead(LSENSOR_PIN1);
	lightReading2 = analogRead(LSENSOR_PIN2);

	//camera
#if(PIXY_ENABLED)
	blocks = pixy.getBlocks();
	getGoalDimensions();
	//pixyCount++;
#endif
	//solenoid
#if(SOLENOID_ENABLED)
	if (!isKicking){
		digitalWrite(SOLENOID_SIG, LOW); //ensure kick pin isn't high
	}
	else if (analogRead(SOLENOID_ANA) == 0){
		endKick();
	}
#endif

	//serial debug to pc
#if(DEBUG_SERIAL)	
	serialDebug();
#endif   
}

void serialDebug(){
	dSerial.println("goalAngle" + String(blocks));
}

void timings(){
	now = millis();
	lastTimeMicros = nowMicros;
	nowMicros = micros();
	pgmFreq = (nowMicros - lastTimeMicros);
	pgmFreq = 1000000 / pgmFreq; 
#if(LED_ENABLED)
	if (now - lBlinkTime >= ledWaitTime){
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
	if (now - lRequestTime >= requestTimeMax){
		ledWaitTime = 50;
	}
#endif
}

/////////////////////////////////////////////////////////////////////
//i2c functions//////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

//initiate i2c
void initI2C(){
	Wire.begin(I2C_SLAVE, I2C_ADDRESS, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE); // join i2c bus (address optional for master)        // join i2c bus with address 
	//Wire.begin(I2C_ADDRESS);
	Wire.onReceive(receiveEvent);   // join i2c event handlers
	Wire.onRequest(requestEvent);
}

//response to Wire.write();
void receiveEvent(size_t len){	
	strLength = (len - 4);
	if (Wire.available()){
		//if (len != 1){ return; }
		//memset(command, 0, sizeof(command));
		Wire.readBytes(command, len);
	}
	else{
		return;
	}	
}

//response to Wire.requestFrom();
void requestEvent(){	
	//dSerial.println("b");
	switch (command[0])
	{
	case COMMAND_LCD_PRINT:
		commandLCDPrint();
		break;
	case COMMAND_LCD_ERASE:
		commandLCDErase();
		break;
	case COMMAND_LCD_LINE:
		commandLCDLine();
		break;
	case COMMAND_LCD_RECT:
		commandLCDRect();
		break;
	case COMMAND_LSENSOR1:
		commandLSensor1();
		break;
	case COMMAND_LSENSOR2:
		commandLSensor2();
		break;
	case COMMAND_GOAL_ANGLE:
		commandGoalAngle();
		break;
	case COMMAND_GOAL_X:
		commandGoalX();
		break;
	case COMMAND_KICK:
		break;
	default:
		//unknown command
		break;
	}
	ledWaitTime = 500;
	lRequestTime = now;	
}

/////////////////////////////////////////////////////////////////////
//command functions//////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

void commandLCDPrint(){
	x_1 = (command[1] << 8) + command[2];
	y_1 = (command[3] << 8) + command[4];
	//check if string has arrived
	if (1 == 1){	
		char str[strLength];
		for (int i = 0; i < strLength; i++){
			str[i] = command[i + 5];
		}
		lcdString(x_1, y_1, MAX_X_PORTRAIT, MAX_Y_PORTRAIT, str);
		Wire.send(1);
	}
	else{
		Wire.send(0);
	}
	memset(command, 0, sizeof(command));
}

void commandLCDErase(){
	lcd.erase();
	Wire.send(1);
	memset(command, 0, sizeof(command));
}

void commandLCDLine(){
	x_1 = (command[1] << 8) + command[2];
	y_1 = (command[3] << 8) + command[4];
	x_2 = (command[5] << 8) + command[6];
	y_2 = (command[7] << 8) + command[8];
	if (true){
		lcd.drawLine(x_1, y_1, x_2, y_2, BLUE);
		Wire.send(1);		
	}
	else{
		Wire.send(0);
	}
	memset(command, 0, sizeof(command));
}

void commandLCDRect(){
	x_1 = (command[1] << 8) + command[2];
	y_1 = (command[3] << 8) + command[4];
	x_2 = (command[5] << 8) + command[6];
	y_2 = (command[7] << 8) + command[8];
	if (true){
		lcd.drawRectangle(x_1, y_1, x_2, y_2, BLACK, FILL);
		Wire.send(1);
	}
	else{
		Wire.send(0);
	}
}

void commandLSensor1(){
	Wire.send(highByte(lightReading1));
	Wire.send(lowByte(lightReading1));
}

void commandLSensor2(){
	Wire.send(highByte(lightReading2));
	Wire.send(lowByte(lightReading2));
}

void commandGoalAngle(){
	Wire.send(highByte(goalAngle));
	Wire.send(lowByte(goalAngle));
}

void commandGoalX(){
	Wire.send(highByte(goalX));
	Wire.send(lowByte(goalX));
}

void commandKick(){
	if (tryKick()){
		Wire.send(1); //kicked
	}
	else{
		Wire.send(0); //hasn't kicked
	}
}

/////////////////////////////////////////////////////////////////////
//lcd functions//////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

bool lcdInit(){
	bool success = false;
	lcd.init();
	if (lcd.start() == OK && lcd.baudChange(LCD_BAUDRATE) == OK){
		success = true;
	}
	lcd.orientation(PORTRAIT_TOP);
	lcd.setEraseBackColour(BLACK);
	lcd.setTextColour(WHITE);
	lcdSplashScreen();
	return success;
}

void lcdSplashScreen(){
	lcd.erase();
	lcd.setTextSize(FONT5);
	lcd.string(5, 5, MAX_X_PORTRAIT, MAX_Y_PORTRAIT, "Team PI", 0);
	lcd.string(5, 200, MAX_X_PORTRAIT, MAX_Y_PORTRAIT, "Team PI", 0);
	lcd.setTextSize(FONT2);
}

SMARTGPUREPLY eraseLcd(){
	if (now - lEraseTime > 1000/LCD_FREQ){
		lEraseTime = now;
		return lcd.erase();		
	}
	return OK;
}

SMARTGPUREPLY lcdString(AXIS a, AXIS b, AXIS c, AXIS d, String str){
	return lcd.string(a, b, c, d, str, 0);
}
/////////////////////////////////////////////////////////////////////
//camera functions///////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

void getGoalDimensions(){
	if (blocks){
		Serial.println("got some blocks");
		if ((pixy.blocks[0].width * pixy.blocks[0].height > 1000) && pixy.blocks[0].y < 125){
			//goal detected
			goalDetected = true;
			goalX = pixy.blocks[0].x;
			goalY = pixy.blocks[0].y;
			goalWidth = pixy.blocks[0].width;
			goalHeight = pixy.blocks[0].height;
			goalAngle = (goalX - 320/2) * 75 / 320;
			lDetectGoal = millis();
		}
		else if(millis() - lDetectGoal > 800){	//if goal isn't detect for 1500ms
			goalDetected = false;
			goalX = 0;
			goalY = 0;
			goalWidth = 0;
			goalHeight = 0;
			goalAngle = 1000;
		}
	}	
	else if(millis() - lDetectGoal > 800){	//if goal isn't detect for 1500ms
		goalDetected = false;
		goalX = 0;
		goalY = 0;
		goalWidth = 0;
		goalHeight = 0;
		goalAngle = 1000;	//goalAngle of 1000 is none
	}
}

/////////////////////////////////////////////////////////////////////
//solenoid functions/////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

bool tryKick(){	
#if(SOLENOID_ENABLED)
	if (analogRead(SOLENOID_ANA) > SOLENOID_ANA_REFERENCE){
		//kick is ready
		digitalWrite(SOLENOID_SIG, HIGH);
	}
#endif
	return false;
}

void endKick(){
#if(SOLENOID_ENABLED)
	digitalWrite(SOLENOID_SIG, LOW);
	isKicking = false;
#endif
}