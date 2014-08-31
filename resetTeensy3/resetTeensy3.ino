#define RESTART_ADDR 0xE000ED0C
#define READ_RESTART() (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))

void setup()
{
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);
	delay(100);
	digitalWrite(13, LOW);
	delay(100);
	//reset teensy
	WRITE_RESTART(0x5FA0004);
}

void loop()
{
	//should never reach here
}
