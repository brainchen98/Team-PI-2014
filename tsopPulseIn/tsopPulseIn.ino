#define SAMPLES 127

unsigned long duration = 0;
uint32_t total = 0;
int _index = 0;
uint16_t samples[SAMPLES] = { 0 };
uint16_t average;

extern const uint8_t tsops[] = {
	1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
	11, 12, 14, 15, 16, 17, 20, 21, 22, 23
};

void setup()
{
	Serial.begin(9600);
  /* add setup code here */
	pinMode(11, INPUT);
}

void loop()
{
	unsigned long duration = pulseIn(11, HIGH, 20000);
	duration = digitalRead(11);
	if (duration != 0){
		Serial.println(duration);
	}
	if (duration)
	{
		total -= samples[_index]; 
		samples[_index] = duration;
		total += samples[_index]; 
		_index = (_index + 1) % SAMPLES;
		average = total / SAMPLES;
		Serial.println(average);
	}
	// clear cache if nothing detected
	else { for (byte i = 0; i<SAMPLES; i++){ samples[i] = 0; } total = 0; }
}
