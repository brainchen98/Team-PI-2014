#include <Wire.h>

int i = 0;

void setup()
{
	Serial.begin(9600);
	Wire.begin(2);                // join i2c bus with address #2
	Wire.onRequest(requestEvent); // register event
	pinMode(13, OUTPUT);
}

void loop()
{

}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent()
{
	i++;
	Wire.write("hello "); // respond with message of 6 bytes
	Serial.print(i); Serial.print("\t");
	Serial.println("requested. Replied 'hello '");
}
