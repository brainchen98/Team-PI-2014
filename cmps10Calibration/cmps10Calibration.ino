/*
CMPS10 Calibration code for Team PI

To compile this program for Teensy 3.0 in VS or Atmel Studio with Visual Micro, add the
following to the DEFINES PROJECT property
F_CPU=48000000;USB_SERIAL;LAYOUT_US_ENGLISH
*/

#include <SRF08.h>
#include <i2c_t3.h>
#include <i2cAnything.h>
#include <cmps10.h>
#include <debugSerial.h>

//debug//////////////////////////////////////////////////////////////
#define LED 13

#define DEBUG_SERIAL true
#define DEBUGSERIAL_BAUD 9600

#define BT_TX 0
#define BT_RX 1
#define BT_SERIAL Serial1

#define LCD_ENABLED false

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

//slave2
#define COMMAND_LCD_PRINT 0
#define COMMAND_LCD_ERASE 1
#define COMMAND_LCD_LINE 2
#define COMMAND_LSENSOR1 10
#define COMMAND_LSENSOR2 11
#define COMMAND_US_FRONT 12

//cmps///////////////////////////////////////////////////////////////
#define CMPS_ADDRESS					0x60
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
CMPS10 cmps(CMPS_ADDRESS);

//calibration
uint16_t pause = 8000;
bool done = false;

float bearing = 0;
byte version;
/////////////////////////////////////////////////////////////////////

void waitForInput(){
	while (Serial.available() == 0) {}
	while (Serial.available()){
		Serial.read();
	}
}

void setup()
{
	Serial.begin(9600);
	pinMode(LED, OUTPUT);
	digitalWrite(LED, LOW);
	delay(25);
	digitalWrite(LED, HIGH);
	delay(25);
	digitalWrite(LED, LOW);
	
	initI2C();
	delay(500);
	lcdWrite(5, 50, "CMPS10 Calib");
	delay(1000);	//short delay to wait for other i2c devices to start	
	lcdErase();
}


void loop()
{
	version = cmps.getVersion();
	lcdWrite(5, 160, String(version));
	Serial.println(version);

	lcdWrite(5, 5, "Start");
	Serial.println("Start");
	Wire.beginTransmission(CMPS_ADDRESS);
	Wire.write(22);
	Wire.write(0xF0);
	Wire.endTransmission();

	digitalWrite(LED, HIGH);
	lcdWrite(5, 20, "North");
	Serial.println("North");
	waitForInput();
	Wire.beginTransmission(CMPS_ADDRESS);
	Wire.write(22);
	Wire.write(0xF5);
	Wire.endTransmission();

	digitalWrite(LED, LOW);
	lcdWrite(5, 35, "East");
	Serial.println("East");
	waitForInput();
	Wire.beginTransmission(CMPS_ADDRESS);
	Wire.write(22);
	Wire.write(0xF5);
	Wire.endTransmission();

	digitalWrite(LED, HIGH);
	lcdWrite(5, 50, "South");
	Serial.println("South");
	waitForInput();
	Wire.beginTransmission(CMPS_ADDRESS);
	Wire.write(22);
	Wire.write(0xF5);
	Wire.endTransmission();

	digitalWrite(LED, LOW);
	lcdWrite(5, 65, "West");
	Serial.println("West");
	waitForInput();
	Wire.beginTransmission(CMPS_ADDRESS);
	Wire.write(22);
	Wire.write(0xF5);
	Wire.endTransmission();

	digitalWrite(LED, HIGH);
	done = true;
	lcdErase();
	lcdWrite(5, 5, "Done");
	Serial.println("Done!");

	//cmps.initialise();
	while (true){
		cmps.getBearing(bearing);
		lcdErase();
		lcdWrite(5, 5, String(bearing));
		Serial.println(bearing);
		delay(20);
	}
}
/////////////////////////////////////////////////////////////////////
//i2c functions//////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

//initiate i2c
void initI2C(){
	//Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE);	
	Wire.begin();
}


uint8_t lcdWrite(int16_t x, int16_t y, String str){
#if(LCD_ENABLED)
	Wire.beginTransmission(SLAVE2_ADDRESS);
	Wire.write(COMMAND_LCD_PRINT);
	Wire.write(highByte(x));
	Wire.write(lowByte(x));
	Wire.write(highByte(y));
	Wire.write(lowByte(y));
	for (int i = 0; i < sizeof(str); i++){
		Wire.write(str[i]);
	}
	Wire.endTransmission();
	Wire.requestFrom(SLAVE2_ADDRESS, 1);
	while (Wire.available() == 0);
	return Wire.read();
#endif
}

uint8_t lcdErase(){
#if(LCD_ENABLED)
	Wire.beginTransmission(SLAVE2_ADDRESS);
	Wire.write(COMMAND_LCD_ERASE);
	Wire.endTransmission();
	Wire.requestFrom(SLAVE2_ADDRESS, 1);
	while (Wire.available() == 0);
	return Wire.read();
#endif
}

uint8_t lcdDrawLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2){
#if(LCD_ENABLED)
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
#endif
}
