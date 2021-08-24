#include "OpenServo.h"

OpenServo::OpenServo(void)
{

}

void OpenServo::ServoSignalRising(void)
{

}

OpenServo::init(int _pin)
{
    pin = _pin;
    pinMode(pin, INPUT); 

    attachInterrupt(2, ServoSignalRising, RISING);
}