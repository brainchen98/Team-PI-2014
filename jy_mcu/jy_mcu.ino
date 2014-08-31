void setup()
{
	// Open serial communications and wait for port to open:
	Serial.begin(9600);
	Serial.println("Goodnight moon!");

	Serial1.begin(9600);
	while (true){
		Serial1.println("Hello, world?");
	}
}

void loop() // run over and over
{
	if (Serial1.available())
		Serial.write(Serial1.read());
	if (Serial.available())
		Serial1.write(Serial.read());
}
