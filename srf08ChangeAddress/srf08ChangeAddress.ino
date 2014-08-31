/*
	Official Team Pi SRF08 Ultrasonic Changer.
	Make sure there is only one SRF08 device connected when changing the address.

	For more info, see http://www.robot-electronics.co.uk/htm/srf08tech.shtml
*/

#include <i2c_t3.h>
#include <SRF08.h>
//original srf08 address. Default address is 0x70 = 0xE0
#define ORIGINAL_ADDRESS 0x70

/*
new address of srf08. Acceptable addresses are
				Decimal	Hex
				224		0xE0
				226		0xE2
				228		0xE4
				230		0xE6
				232		0xE8
				234		0xEA
				236		0xEC
				238		0xEE
				240		0xF0
				242		0xF2
				244		0xF4
				246		0xF6
				248		0xF8
				250		0xFA
				252		0xFC
				254		0xFE
*/
#define NEW_ADDRESS 0xE2

SRF08 srf(ORIGINAL_ADDRESS);

uint8_t soft = 0;
void setup()
{
	Wire.begin();
	Serial.begin(9600);
	pinMode(13, OUTPUT);
	
	delay(50);
	digitalWrite(13, HIGH);

	Serial.println("Getting software version");
	srf.getSoft(soft);
	Serial.println("Soft rev" + String(soft));

	Serial.print("Changing SRF08 address ");
	Serial.print(ORIGINAL_ADDRESS, HEX);
	Serial.print(" to ");
	Serial.println(NEW_ADDRESS, HEX);
	srf.setAddress(NEW_ADDRESS);

	Serial.println("done!");	
	digitalWrite(13, HIGH);
}

void loop(){
	digitalWrite(13, LOW);
	delay(500);
	digitalWrite(13, HIGH);
	delay(500);
}
