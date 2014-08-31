// --------------------------------------
// i2c_scanner
//
// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not know.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    http://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
// 
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//

#include <Wire.h>

int led = 13;

#define SRF_ADDRESS         0x68                                   // Address of the SRF08
#define CMD                 (byte)0x00                             // Command byte, values of 0 being sent with write have to be masked as a byte to stop them being misinterpreted as NULL this is a bug with arduino 1.0
#define LIGHTBYTE           0x01                                   // Byte to read light sensor
#define RANGEBYTE           0x02                     

byte highByte = 0x00;                             // Stores high byte from ranging
byte lowByte = 0x00;                              // Stored low byte from ranging

void setup()
{
	Wire.begin();

	Serial.begin(9600);
	Serial.println("\nI2C Scanner");
	pinMode(led, OUTPUT);
}


void loop()
{
	digitalWrite(led, HIGH);
	int range = getRange();
	delay(100);           // wait 5 seconds for next scan
	digitalWrite(led, LOW);   // turn the LED on (HIGH is the voltage level)
	delay(100);
	Serial.println(range);
}


int getRange(){                                   // This function gets a ranging from the SRF08

	int range = 0;

	Wire.beginTransmission(SRF_ADDRESS);             // Start communticating with SRF08
	Wire.write(CMD);                                 // Send Command Byte
	Wire.write(0x51);                                // Send 0x51 to start a ranging
	Wire.endTransmission();

	//delay(100);                                      // Wait for ranging to be complete

	Wire.beginTransmission(SRF_ADDRESS);             // start communicating with SRFmodule
	Wire.write(RANGEBYTE);                           // Call the register for start of ranging data
	Wire.endTransmission();

	Wire.requestFrom(SRF_ADDRESS, 2);                // Request 2 bytes from SRF module
	while (Wire.available() < 2);                     // Wait for data to arrive
	highByte = Wire.read();                          // Get high byte
	lowByte = Wire.read();                           // Get low byte

	range = (highByte << 8) + lowByte;               // Put them together

	return(range);                                   // Returns Range
}