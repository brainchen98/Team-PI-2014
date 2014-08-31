#include <RN42.h>

RN42 bluetooth;

void setup()
{
	//Serial.begin(9600);
	initBluetoothSlave();
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);
	delay(100);
}

void loop()
{
	String msg = "hi";
	while (port.available()){		
		msg += (char)port.read();
		digitalWrite(13, HIGH);
		delay(10);
		//msg.append((char)port.read());
	}
	//Serial.println(msg);
	digitalWrite(13, LOW);
}

void initBluetoothSlave(){
	bluetooth.init(115200);
	bluetooth.commandMode();
	delay(50);
	bluetooth.setMode(SLAVE);
	delay(50);
	bluetooth.normalMode();
}