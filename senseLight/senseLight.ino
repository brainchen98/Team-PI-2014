void setup()
{
	Serial.begin(9600);
  /* add setup code here */
	pinMode(A9, INPUT);
}

void loop()
{
	int input = 0;
	for (int i = 0; i < 100; i++){
		input += analogRead(A9);
	}
	input /= 100;
	Serial.println(input);
  /* add main program code here */

}
