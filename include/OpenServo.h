#include <Arduino.h>

class OpenServo
{
public:
    OpenServo(void);
    void ServoSignalRising(void);
    init(int _pin);
private:
    int pin;
    
    
};


