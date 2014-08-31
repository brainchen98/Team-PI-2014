#include <Wire.h>

void setup()
{
	Wire.begin();        // join i2c bus (address optional for master)
	pinMode(13, OUTPUT);
}

void loop()
{
	Wire.requestFrom(2, 6);    // request 6 bytes from slave device #2
	

	while (Wire.available())    // slave may send less than requested
	{
		digitalWrite(13, HIGH);
		char c = Wire.read(); // receive a byte as character
	}
}