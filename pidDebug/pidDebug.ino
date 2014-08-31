#include <pid.h>

#define outMax 255
#define outMin -255

int16_t input, output, offset = 0;

float kp = 2, ki = 0, kd = 0;

PID pid(input, output, offset, kp, ki, kd, outMax, outMin, 0);

float **f;

union fbytes { float f; char chars[sizeof(float)]; };

void setup()
{
    Serial.begin(9600);
    f[0] = &kp;
    *f[0] = 3;
    Serial.println((int)kp);
}

void loop()
{
    uint8_t available = Serial.available();
    if(available > 0){
        char buff[available];
        Serial.readBytes(buff, available);
        Serial.println(buff);
        if(buff[0] == (uint8_t)'f'){
            //edit var
            uint8_t varIndex = (uint8_t)buff[1];
            fbytes newFloat;
            newFloat.chars[0] = buff[2];
            newFloat.chars[1] = buff[3];
            newFloat.chars[2] = buff[4];
            newFloat.chars[3] = buff[5];
            *f[varIndex] = newFloat.f;
        }
    }
    input = 50;
    pid.compute();
    Serial.print(output);
    Serial.print(" ");
    Serial.print(pid.error);
    Serial.print(" ");
    Serial.print(pid.integral);
    Serial.print(" ");
    Serial.print(pid.derivative);
    Serial.println();
    delay(0);
}
