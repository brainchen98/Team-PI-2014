/*
F_CPU=48000000;USB_SERIAL;LAYOUT_US_ENGLISH
800ms after slave startup to connect
*/

#include <RN42.h>

RN42 bluetooth;

void setup()
{
	pinMode(13, HIGH);
	delay(700);
	initBluetoothMaster();		
	digitalWrite(13, HIGH);
	delay(50);
}

void loop()
{
	port.println("55");
	
	digitalWrite(13, LOW);
}


void initBluetoothMaster(){
	bluetooth.init(115200);
	bluetooth.commandMode();
	delay(50);
	bluetooth.setMode(MASTER);
	delay(50);
	bluetooth.connect("0006664E42D4");
	delay(100);
	bluetooth.normalMode();
}