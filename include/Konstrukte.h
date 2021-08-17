struct AccelerometerValueContainer {
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t roll;
    int16_t pitch;
};

struct ServoValueContainer {
  float Servo1Value;
  float Servo2Value;
  float Servo3Value;
  float Servo4Value;
};

struct CompassValueContainer {
  int Azimuth;
  byte Bearing;
  int X;    // Lage
  int Y;    // Krängung
  int Z;
};

struct RoutineRuntimeContainer {
  int GetCompassValuesRuntime;
  int GetAccelerometerValuesRuntime;
  int SetServoValuesRuntime;
  int GetServoValuesRuntime;
  int GetValuesCallbackRuntime;
  int CalculateReactionRuntime;
};

struct PositionValueContainer {
  float bearing;
  
};

#define FILTERSIZE 50





int i = 0 ;
int __X[FILTERSIZE];
int __Y[FILTERSIZE];
int __Z[FILTERSIZE];

struct CompassValueContainer CompassValuesAverageFilter(CompassValueContainer _CompassValues)
{
  int tempX = 0;
  int tempY = 0;
  int tempZ = 0;

  __X[i] = _CompassValues.X;
  __Y[i] = _CompassValues.Y;
  __Z[i] = _CompassValues.Z;
    
  i++;
  if(i == FILTERSIZE) i = 0;

for(int waste=0;waste<FILTERSIZE; waste++)
{
  tempX = __X[waste] + tempX;
  tempY = __Y[waste] + tempY;
  tempZ = __Z[waste] + tempZ;
}

_CompassValues.X = tempX / FILTERSIZE;
_CompassValues.Y = tempY / FILTERSIZE;
_CompassValues.Z = tempZ / FILTERSIZE;

return _CompassValues;
}