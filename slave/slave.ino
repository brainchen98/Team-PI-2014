#include <Wire.h> 

#define LED 13

byte i = 0;

void setup()
{
	Wire.begin(0x88);                // join i2c bus with address #2 
	Wire.onRequest(requestEvent); // register event 
	Serial.begin(9600);           // start serial for output 
	pinMode(LED, OUTPUT);
}

void loop()
{
	delay(100);
}

// function that executes whenever data is received from master 
// this function is registered as an event, see setup() 
void requestEvent()
{
	digitalWrite(LED, HIGH);
	Wire.write(i);
	if (i > 100){
		i = 0;
	}
	i++;
	digitalWrite(LED, LOW);
}


