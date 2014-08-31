/*
	Slave1 code for TEAM PI. Slave1 has an i2c address of 0x31. It continuously reads tsop data.
	Created by Brian Chen 25/12/2013
	Last Modified by Brian Chen 04/04/2014 11:54pm

	Beta 0.60 (C) TEAM PI 2014

	F_CPU=48000000;USB_SERIAL;LAYOUT_US_ENGLISH
*/

#include <i2c_t3.h>
#include <TSOP.h>
#include <i2cAnything.h>
#include <debugSerial.h>
#include <SMARTGPU2.h>

#define LED 13

#define DEBUG_SERIAL true
#define DEBUGSERIAL_BAUD 9600
#define DEBUG_LCD false

//slavei2c///////////////////////////////////////////////////////////
#define I2C_ADDRESS 0x31
#define I2C_RATE I2C_RATE_400

//lcd////////////////////////////////////////////////////////////////
#define LCD_FREQ 30 //frequency of lcd refreshes
#define LCD_WAIT_TIME 1000000/LCD_FREQ     

//commands///////////////////////////////////////////////////////////
#define COMMAND_ANGLE_FLOAT 0
#define COMMAND_ANGLE_ADV_FLOAT 1
#define COMMAND_STRENGTH 2
#define COMMAND_RESULTS 3
#define COMMAND_TSOP_PINS 4

//tsop///////////////////////////////////////////////////////////////
#define TSOP_COUNT 20   //number of tsops
#define UNLOCK_PIN 0    //pin that powers tsops
#define TSOP_WAIT_TIME 2500		//time between unlocking tsops in milliseconds
/////////////////////////////////////////////////////////////////////

SMARTGPU2 lcd;

//tsop stuff/////////////////////////////////////////////////////////

//the pin numbers of the tsops. Based on Eagle designs of robots
extern const uint8_t tsops[] ={
	1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 
    11, 12, 14, 15, 16, 17, 20, 21, 22, 23 
};

//define tsop
TSOP tsop(tsops, UNLOCK_PIN);

uint8_t results[TSOP_COUNT];    //array to store results
float angle;    //variable to store angle result from tsops
uint8_t strength;

unsigned long lUnlockTime, lLCDTime, now;

//debug stuff////////////////////////////////////////////////////////
debugSerial dSerial;

//i2c stuff//////////////////////////////////////////////////////////
uint8_t command;


uint8_t b, bIndex;

void setup(){
	initI2C();
    pinMode(LED, OUTPUT);    
	dSerial.begin(DEBUGSERIAL_BAUD);
	#if(DEBUG_LCD)
		if (lcdInit()){
			dSerial.println("SMARTGPU2 initiation successful");
		}
	#endif
	lUnlockTime = micros();  
    lLCDTime = micros();  
    digitalWrite(LED, HIGH);
}

void loop(){
	digitalWrite(LED, HIGH);
	//timings
    timings();    
    //main stuff
	tsop.getAll(results);
	tsop.getBest(results, bIndex, b);
    tsop.getAngleAdv(results, angle);
	tsop.getStrength(results, strength);
    //serial debug to pc 
	#if(DEBUG_SERIAL)
		serialDebug();   
	#endif   
}

void serialDebug(){    
	
    dSerial.append("\nAngle:" + String(angle));
	dSerial.append("\tStrength:" + String(strength));
	dSerial.append("\tBest:" + String(bIndex));
    dSerial.writeBuffer();
	//Serial.print(tsop.index);		Serial.print('\t');		Serial.println(b_value);	
	for (int i = 0; i < TSOP_COUNT; i++){
		dSerial.append(String(results[i]) + " ");
	}
}

void timings(){
    now = micros();
	if (now - lUnlockTime >= TSOP_WAIT_TIME){
        lUnlockTime = now;
		tsop.unlock();        
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
		default:
			//unknown command
			break;
	}
}

/////////////////////////////////////////////////////////////////////
//command functions//////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

//send angle as a 4 byte float
void commandAngleFloat(){	
	float output;
	tsop.getAngle(results, output);	//get estimated angle
	I2C_write(output);	//write output. I2C_write can write anything
}

void commandAngleAdvFloat(){
	float output;
	tsop.getAngleAdv(results, output);
	I2C_write(output);
}

void commandStrength(){
	uint8_t output;
	tsop.getStrength(results, output);
	I2C_write(output);
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

/////////////////////////////////////////////////////////////////////
//lcd functions//////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

bool lcdInit(){
	bool success = false;
	lcd.init();
	if (lcd.start() == OK && lcd.baudChange(2000000) == OK){
		success = true;
	}
	lcd.orientation(PORTRAIT_LOW);
	lcd.setEraseBackColour(BLACK);
	lcd.setTextColour(WHITE);
	lcd.erase();
	lcd.setTextSize(FONT3);
	lcd.string(5, 5, MAX_X_PORTRAIT, MAX_Y_PORTRAIT, "Team PI", 0);
	lcd.setTextSize(FONT0);
	return success;
}