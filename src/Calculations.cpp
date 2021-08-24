#include "Calculations.h"

Tendencies::Tendencies()
{
    IntegerValueCounter = 0;
    FloatValueCounter = 0;
}

int Tendencies::compareInteger(int _Value)
{
    int tendencie = 0;

    if(lastIntegerValue < _Value){
        tendencie = -1;
    }

    if(lastIntegerValue > _Value){
        tendencie = 1;
    }

    lastIntegerValue = _Value;
    IntegerValueHistory[IntegerValueCounter] = _Value;
    IntegerValueCounter++;
    if(IntegerValueCounter == 256) IntegerValueCounter = 0;
    return tendencie;
}

int Tendencies::returnIntegerHistoryPosition(void)
{
    return IntegerValueCounter;
}

void Tendencies::setIntegerHistoryPosition(int _position)
{
    if(_position == 256) _position = 0;
    IntegerValueCounter = _position;
}

int Tendencies::returnIntegerHistoryValue(int _position)
{
    if(_position == 256) _position = 0;
    return IntegerValueHistory[_position];
}

int Tendencies::compareFloat(float _Value)
{
    int tendencie = 0;

    if(lastFloatValue < _Value){
        tendencie = -1;
    }

    if(lastFloatValue > _Value){
        tendencie = 1;
    }

    lastFloatValue = _Value;
    FloatValueHistory[FloatValueCounter] = _Value;
    FloatValueCounter++;
    if(FloatValueCounter == 256) FloatValueCounter = 0;

    return tendencie;
}

int Tendencies::returnFloatHistoryPosition(void)
{
    return FloatValueCounter;
}

void Tendencies::setFloatHistoryPosition(int _position)
{
    if(_position == 256) _position = 0;
    FloatValueCounter = _position;
}

float Tendencies::returnFloatHistoryValue(int _position)
{
    if(_position == 256) _position = 0;
    return FloatValueHistory[_position];
}