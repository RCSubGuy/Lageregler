#include "OpenServo.h"
#include <avr/io.h>
#include <avr/interrupt.h>


// Outside of class
OpenServo *pointerToClass; // declare a pointer to testLib class

static void RisingInterruptHandler(void) { // define global handler
  pointerToClass->ServoSignalRising(); // calls class member handler
  //Serial.println("Pre Rising");
}

static void FallingInterruptHandler(void) { // define global handler
  pointerToClass->ServoSignalFalling(); // calls class member handler
  //Serial.println("Pre Falling");
}

OpenServo::OpenServo(void)
{

}

void OpenServo::ServoSignalRising(void)
{
    ServoSignalStartTime = micros();
    attachInterrupt(digitalPinToInterrupt(pin), FallingInterruptHandler, FALLING);
    //Serial.println("Rising");
}

void OpenServo::ServoSignalFalling(void)
{
    ServoSignalTime = micros() - ServoSignalStartTime;
    attachInterrupt(digitalPinToInterrupt(pin), RisingInterruptHandler, RISING);
    //Serial.print(Channelname);
    //Serial.println(ServoSignalTime);
}

OpenServo::init(int _pin, String _name)
{
    Channelname = _name;
    pin = _pin;
    pinMode(pin, INPUT); 
    pointerToClass = this;
    attachInterrupt(digitalPinToInterrupt(pin), RisingInterruptHandler, RISING);
}

int OpenServo::getValue(void)
{
    return ServoSignalTime;
}

