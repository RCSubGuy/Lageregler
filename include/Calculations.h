#include <Arduino.h>

class Tendencies
{
public:
    Tendencies();

    int compareInteger(int _Value);
    int returnIntegerHistoryPosition(void);
    void setIntegerHistoryPosition(int _position);
    int returnIntegerHistoryValue(int _position);

    int compareFloat(float _Value);
    int returnFloatHistoryPosition(void);
    void setFloatHistoryPosition(int _position);
    float returnFloatHistoryValue(int _position);

    
private:
    int lastIntegerValue;
    int IntegerValueCounter;
    int IntegerValueHistory[256];

    float lastFloatValue;
    int FloatValueCounter;
    float FloatValueHistory[256];
};