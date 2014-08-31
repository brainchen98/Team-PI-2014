#include <SMARTGPU2.h>     //include the SMARTGPU2 library!

SMARTGPU2 lcd;             //create our object called LCD

char message[] = "Hello World";

void setup() { //initial setup
	//Those two functions must always be called for SMARTGPU2 support
	Serial.begin(9600);
	delay(250);
	Serial.println("Begin");
	lcd.init();  //configure the serial and pinout of arduino board for SMARTGPU2 support
	Serial.println("INited");
	lcd.start(); //initialize the SMARTGPU2 processor
	pinMode(13, OUTPUT);
	
}

void loop() { //main loop
	digitalWrite(13, HIGH);
	lcd.erase();                                         //erase screen
	lcd.putPixel(300, 200, CYAN);                          //draw a pixel
	lcd.drawLine(50, 50, 150, 200, WHITE);                   //draw a line
	lcd.drawRectangle(10, 10, 200, 180, RED, UNFILL);         //draw a rectangle
	lcd.drawCircle(160, 120, 50, GREEN, UNFILL);             //draw a circle
	lcd.drawTriangle(15, 15, 200, 210, 180, 70, BLUE, UNFILL);  //draw a triangle
	delay(100);
	digitalWrite(13, LOW);
	delay(100);
}