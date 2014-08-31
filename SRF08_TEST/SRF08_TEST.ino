/*
Generic example for the SRF modules 02, 08, 10 and 235.
Only the SRF08 uses the light saensor so when any other
range finder is used with this code the light reading will
be a constant value.
*/

#include <Wire.h>

#define SRF_ADDRESS         0x68                                   // Address of the SRF08
#define CMD                 (byte)0x00                             // Command byte, values of 0 being sent with write have to be masked as a byte to stop them being misinterpreted as NULL this is a bug with arduino 1.0
#define LIGHTBYTE           0x01                                   // Byte to read light sensor
#define RANGEBYTE           0x02                     

byte highByte = 0x00;                             // Stores high byte from ranging
byte lowByte = 0x00;                              // Stored low byte from ranging

int softRev = 100;

void setup(){
	Serial.begin(9600);
	Wire.begin();
	delay(100);                                     // Waits to make sure everything is powered up before sending or receiving data
	pinMode(13, OUTPUT);
	digitalWrite(13, LOW);
	softRev = getSoft();                        // Call
	
}

void loop(){
	int rangeData = 10;
	rangeData = getRange();                     // Calls a function to get range gaces after data 
	Serial.println(softRev);
	delay(100);                                      // Wait before looping
}

int getRange(){                                   // This function gets a ranging from the SRF08

	int range = 0;

	Wire.beginTransmission(SRF_ADDRESS);             // Start communticating with SRF08
	Wire.write(CMD);                                 // Send Command Byte
	Wire.write(0x51);                                // Send 0x51 to start a ranging
	Wire.endTransmission();

	delay(100);                                      // Wait for ranging to be complete

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

int getLight(){                                    // Function to get light reading

	Wire.beginTransmission(SRF_ADDRESS);
	Wire.write(LIGHTBYTE);                           // Call register to get light reading
	Wire.endTransmission();

	Wire.requestFrom(SRF_ADDRESS, 1);                // Request 1 byte
	while (Wire.available() < 0);                     // While byte available
	int lightRead = Wire.read();                     // Get light reading

	return(lightRead);                               // Returns lightRead

}

int getSoft(){                                     // Function to get software revision

	Wire.beginTransmission(SRF_ADDRESS);             // Begin communication with the SRF module
	Wire.write(CMD);                                 // Sends the command bit, when this bit is read it returns the software revision
	Wire.endTransmission();

	Wire.requestFrom(SRF_ADDRESS, 1);                // Request 1 byte
	while (Wire.available() < 0);                     // While byte available
	int software = Wire.read();                      // Get byte

	return(software);

}
