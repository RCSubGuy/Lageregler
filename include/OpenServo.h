#include <Arduino.h>



class OpenServo
{
public:
    OpenServo(void);
    void ServoSignalRising(void);
    void ServoSignalFalling(void);
    
    init(int _pin, String _name);
    int getValue(void);
private:
    int pin;
    String Channelname;
    volatile int ServoSignalTime;
    volatile int ServoSignalStartTime;
    void (*localPointerToCallback)(const int);  // https://www.onetransistor.eu/2019/05/arduino-class-interrupts-and-callbacks.html
};


