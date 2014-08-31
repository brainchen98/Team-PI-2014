/*
	Slave1 code for TEAM PI. Slave1 has an i2c address of 0x31. It continuously reads tsop data.
	Created by Brian Chen 25/12/2013
	Last Modified by Brian Chen 04/04/2014 11:54pm
	... Forever modified by Brian Chen.

	Beta 0.60 (C) TEAM PI 2014

	To compile this program for Teensy 3.0 in VS or Atmel Studio with Visual Micro, add the
	following to the DEFINES PROJECT property
	F_CPU=48000000;USB_SERIAL;LAYOUT_US_ENGLISH
*/

#include <i2c_t3.h>
#include <TSOP.h>
#include <i2cAnything.h>
#include <debugSerial.h>

#define LED 13

#define DEBUG_SERIAL true
#define DEBUGSERIAL_BAUD 9600
#define DEBUG_LCD false

//slavei2c///////////////////////////////////////////////////////////
#define I2C_ADDRESS 0x31
#define I2C_RATE I2C_RATE_2400

//commands///////////////////////////////////////////////////////////
#define COMMAND_ANGLE_FLOAT 0
#define COMMAND_ANGLE_ADV_FLOAT 1
#define COMMAND_STRENGTH 2
#define COMMAND_RESULTS 3
#define COMMAND_TSOP_PINS 4
#define COMMAND_BINDEX 5

//tsop///////////////////////////////////////////////////////////////
#define TSOP_COUNT 20   //number of tsops
#define UNLOCK_PIN 0    //pin that powers tsops
#define TSOP_WAIT_TIME 2500		//time between unlocking tsops in milliseconds
/////////////////////////////////////////////////////////////////////

unsigned long nowMillis = 0;
unsigned long nowMicros = 0, lastLoopTime = 0;
uint16_t pgmFreq = 0;

//tsop stuff/////////////////////////////////////////////////////////

//the pin numbers of the tsops. Based on Eagle designs of robots
extern const uint8_t tsops[] ={
	1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 
    11, 12, 14, 15, 16, 17, 20, 21, 22, 23 
};

//define tsop
TSOP tsop(tsops, UNLOCK_PIN);

uint8_t results[TSOP_COUNT];    //array to store results
float angle, angleAdv;    //variable to store angle result from tsops
uint8_t strength;

unsigned long lUnlockTime, lLCDTime, lBlinkTime = 0, lRequestTime;
bool on = true;	//led on
uint16_t ledWaitTime = 500;	//led waitTime
uint16_t requestTimeMax = 500;	//max time before i2c not being received

//debug stuff////////////////////////////////////////////////////////
debugSerial dSerial;

//i2c stuff//////////////////////////////////////////////////////////
uint8_t command;
uint8_t b, bIndex;
uint8_t filter_index = 0;
float angleAdvArray[16];
uint8_t strengthArray[16];
uint8_t conseq_removals = 0;

void setup(){
	pinMode(LED, OUTPUT);
	digitalWrite(LED, HIGH);
	initI2C();    
	dSerial.begin(DEBUGSERIAL_BAUD);
	#if(DEBUG_LCD)
		if (lcdInit()){
			dSerial.println("SMARTGPU2 initiation successful");
		}
	#endif
	lUnlockTime = micros(); 
	#if(DEBUG_LCD)
		lLCDTime = micros();  
	#endif    
}
//say hi
void loop(){
	//timings
	float temp;
    timings();    
    //main stuff
	tsop.getAll(results);
	results[0] *= 1.15;
	tsop.getBest(results, bIndex, b);
	//tsop.getAnglePairing(results, angleAdv);
	tsop.getStrength(results, strength);

	tsop.getAngle(results, temp);	
    angle = temp;
	//filter
	tsop.getAngleAdv(results, temp);
	angleAdv = temp;
	/*for (uint8_t i = 0; i < sizeof(angleAdvArray)/sizeof(angleAdvArray[0]) - 1; i++){
		angleAdvArray[i] = angleAdvArray[i + 1];
	}
	for (uint8_t i = 0; i < sizeof(strengthArray) / sizeof(strengthArray[0]) - 1; i++){
		strengthArray[i] = strengthArray[i + 1];
	}
	if (strength < MIN_IGNORE_THRESHOLD && conseq_removals < 4){
		angleAdvArray[15] = angleAdvArray[15 - 1];
		conseq_removals++;
	}
	else{
		conseq_removals = 0;
		tsop.getAngleAdv(results, angleAdvArray[15]);
		strengthArray[15] = strength;
	}*/

	/*if (filter_index == 16){
		float temp = 0;
		temp = getMidAngle(getMidAngle(getMidAngle(getMidAngle(angleAdvArray[0], angleAdvArray[1]), getMidAngle(angleAdvArray[2], angleAdvArray[3])),
			getMidAngle(getMidAngle(angleAdvArray[4], angleAdvArray[5]), getMidAngle(angleAdvArray[6], angleAdvArray[7]))),
			getMidAngle(getMidAngle(getMidAngle(angleAdvArray[8], angleAdvArray[9]), getMidAngle(angleAdvArray[10], angleAdvArray[11])),
			getMidAngle(getMidAngle(angleAdvArray[12], angleAdvArray[13]), getMidAngle(angleAdvArray[14], angleAdvArray[15]))));
		angleAdv = temp;
		temp = 0;
		for (uint8_t i = 0; i < 16; i++){
			temp += strengthArray[i];
		}
		strength = (uint8_t)(temp / 16);
	}
	else{
		filter_index++;
	}*/
	
    //serial debug to pc 
	#if(DEBUG_SERIAL)
		serialDebug();   
	#endif   
	
}

void serialDebug(){  
	dSerial.append("Freq:\t" + String(pgmFreq));
    dSerial.append("\tnangleAdv:\t" + String(angleAdv));
	dSerial.append("\tangle:\t" + String(angle));
	dSerial.append("\tStrength:\t" + String(strength));
	dSerial.append("\tBest:\t" + String(bIndex) + "\t");
    
	//Serial.print(tsop.index);		Serial.print('\t');		Serial.println(b_value);	
	for (int i = 0; i < TSOP_COUNT; i++){
		dSerial.append(String(results[i]) + "\t");
	}
	dSerial.writeBuffer();
}

void timings(){
    nowMillis = millis();
	nowMicros = micros();

	pgmFreq = 1000000 / (nowMicros - lastLoopTime);	//get program frequency
	lastLoopTime = nowMicros;

	if (nowMillis - lUnlockTime >= TSOP_WAIT_TIME){
        lUnlockTime = nowMillis;
		tsop.unlock(); 
	}
	if (nowMillis - lBlinkTime >= ledWaitTime){
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
	if(nowMillis - lRequestTime >= requestTimeMax){
		ledWaitTime = 50;
	}
	#if(DEBUG_LCD)
		if (now - lLCDTime >= LCD_WAIT_TIME){
			//lcd stuff
			lLCDTime = now;        
				String buffer = "";
				for(int i = 0; i < TSOP_COUNT; i++){
					buffer += String(results[i]) + "\t";
				}
				buffer += "\n\nAngle: " + String(angle);
				lcd.string(5, 50, MAX_X_PORTRAIT, MAX_Y_PORTRAIT, buffer, 0);                
		}
	#endif
}

/////////////////////////////////////////////////////////////////////
//i2c functions//////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

//initiate i2c
void initI2C(){
	Wire.begin(I2C_SLAVE, I2C_ADDRESS, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE); // join i2c bus (address optional for master)        // join i2c bus with address 
	Wire.onReceive(receiveEvent);   // join i2c event handlers
	Wire.onRequest(requestEvent);
}

//response to Wire.write();
void receiveEvent(size_t len){	
	if (Wire.available()){
		//if (len != 1){ return; }
		command = Wire.read();
	}
	else{
		return;
	}	
}

//response to Wire.requestFrom();
void requestEvent(){
	switch (command)
	{
		case COMMAND_ANGLE_FLOAT: 
			commandAngleFloat(); 
			break;
		case COMMAND_ANGLE_ADV_FLOAT:
			commandAngleAdvFloat();
			break;
		case COMMAND_STRENGTH:
			commandStrength();
			break;
		case COMMAND_RESULTS:
			commandResults();
			break;
		case COMMAND_TSOP_PINS:
			commandTSOPPins();
			break;
		case COMMAND_BINDEX:
			commandBIndex();
			break;
		default:
			//unknown command
			break;
	}
	ledWaitTime = 500;
	lRequestTime = nowMillis;
}

/////////////////////////////////////////////////////////////////////
//command functions//////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

//send angle as a 4 byte float
void commandAngleFloat(){	
	I2C_write(angle);	//write output. I2C_write can write anything
}

void commandAngleAdvFloat(){
	I2C_write(angleAdv);
}

void commandStrength(){
	uint8_t output;
	I2C_write(strength);
}

//send results array
void commandResults(){
	//results is already in byte/uint8_t array
	Wire.write(results, TSOP_COUNT);
}

//send tsop pins array
void commandTSOPPins(){
	//tsop pins already in byte/uint8_t array
	Wire.write(tsop.tsop_pins, TSOP_COUNT);	
}

void commandBIndex(){
	Wire.send(bIndex);
}

inline float getMidAngle(float a, float b){
	float index;
	if (abs(a - b) > 180){
		index = ((a + b) / 2 + 180);
		if (index >= 360){
			index -= 360;
		}
		if (index < 0){
			index += 360;
		}
	}
	else{
		index = (a + b) / 2;
	}
	return index;
}