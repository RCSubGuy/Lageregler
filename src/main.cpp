#include <Arduino.h>
#include <ADC.h>
#include <ADC_util.h>
#include <Servo.h>
#include "PCA9685.h"
#include <SPI.h>
#include <QMC5883LCompass.h>
#include <ADXL345.h>
#include <Thread.h>
#include <PID_v1.h>
#include <EEPROM.h>

#include "main.h"
#include "Konstrukte.h"

#define MYDEBUG
#define MYDEBUG_TIMECYCLE 200
#define MYDEBUG_VALUE_PLOTTER

QMC5883LCompass compass;
PCA9685 pwmController(B010101);           // Library using Wire1 @400kHz, and B101010 (A5-A0) i2c address
ADXL345 adxl; //variable adxl is an instance of the ADXL345 library

// Testing our second servo has found that -90° sits at 128, 0° at 324, and +90° at 526.
// Since 324 isn't precisely in the middle, a cubic spline will be used to smoothly
// interpolate PWM values, which will account for said discrepancy. Additionally, since
// 324 is closer to 128 than 526, there is slightly less resolution in the -90° to 0°
// range while slightly more in the 0° to +90° range.

PCA9685_ServoEval ServoOutput1(128,324,526);
PCA9685_ServoEval ServoOutput2(128,324,526);
PCA9685_ServoEval ServoOutput3(128,324,526);
PCA9685_ServoEval ServoOutput4(128,324,526);

Thread DebugThread = Thread();
Thread GetValuesThread = Thread();

ADC *adc = new ADC(); // adc object;

struct ServoValueContainer ServoInputValues;
struct ServoValueContainer ServoOutputValues;
struct CompassValueContainer CompassValues;
struct AccelerometerValueContainer AccelerometerValues; 
struct RoutineRuntimeContainer RoutineRuntimeValues;

float DepthValue;
int CompassCorrectionValueX = 760;
int CompassCorrectionValueY = -808;
int CompassCorrectionValueZ = -945;

volatile int ServoSignal1StartTime;
volatile int ServoSignal2StartTime;
volatile int ServoSignal3StartTime;
volatile int ServoSignal4StartTime;

//Define Variables we'll be connecting to
double LagereglerSetpoint, LagereglerInput, LagereglerOutput;
double TiefenraglerSetpoint, TiefenraglerInput, TiefenraglerOutput;

//Specify the links and initial tuning parameters
double LagereglerKp=2, LagereglerKi=5, LagereglerKd=1;
double TiefenreglerKp=2, TiefenreglerKi=5, TiefenreglerKd=1;

double EEPROMLagereglerKp = 0;
double EEPROMLagereglerKi = 16;
double EEPROMLagereglerKd = 32;
double EEPROMTiefenreglerKp = 48;
double EEPROMTiefenreglerKi = 64;
double EEPROMTiefenreglerKd = 80;

PID LagereglerPID(&LagereglerInput, &LagereglerOutput, &LagereglerSetpoint, LagereglerKp, LagereglerKi, LagereglerKd, DIRECT);
PID TiefenreglerPID(&LagereglerInput, &TiefenraglerOutput, &TiefenraglerSetpoint, TiefenreglerKp, TiefenreglerKi, TiefenreglerKd, DIRECT);

void ServoSignal1Rising(void) {
  attachInterrupt(2, ServoSignal1Falling, FALLING);
  ServoSignal1StartTime = micros();
}

void ServoSignal2Rising(void) {
  attachInterrupt(3, ServoSignal2Falling, FALLING);
  ServoSignal2StartTime = micros();
}

void ServoSignal3Rising(void) {
  attachInterrupt(4, ServoSignal3Falling, FALLING);
  ServoSignal3StartTime = micros();
}

void ServoSignal4Rising(void) {
  attachInterrupt(5, ServoSignal4Falling, FALLING);
  ServoSignal4StartTime = micros();
}

void ServoSignal1Falling(void) {
  float Servo1Value = micros() - ServoSignal1StartTime;
  Servo1Value = (Servo1Value - 1550) / 5;
  ServoInputValues.Servo1Value = Servo1Value;
}

void ServoSignal2Falling(void) {
  float Servo2Value = micros() - ServoSignal2StartTime;
  Servo2Value = (Servo2Value - 1550) / 5;
  ServoInputValues.Servo2Value = Servo2Value;
}

void ServoSignal3Falling(void) {
  float Servo3Value = micros() - ServoSignal3StartTime;
  Servo3Value = (Servo3Value - 1550) / 5;
  ServoInputValues.Servo3Value = Servo3Value;
}

void ServoSignal4Falling(void) {
  float Servo4Value = micros() - ServoSignal4StartTime;
  Servo4Value = (Servo4Value - 1550) / 5;
  ServoInputValues.Servo4Value = Servo4Value;
}

void GetServoValues(void)
{
  int start_time = micros();
    
  attachInterrupt(2, ServoSignal1Rising, RISING);
  attachInterrupt(3, ServoSignal2Rising, RISING);
  attachInterrupt(4, ServoSignal3Rising, RISING);
  attachInterrupt(5, ServoSignal4Rising, RISING);

 // _ServoInputValues.Servo1Value = pulseIn(2, HIGH);
 // _ServoInputValues.Servo2Value = pulseIn(3, HIGH);
 // _ServoInputValues.Servo3Value = pulseIn(4, HIGH);
 // _ServoInputValues.Servo4Value = pulseIn(5, HIGH);

  RoutineRuntimeValues.GetServoValuesRuntime = micros() - start_time;   
}

void SetServoValues(struct ServoValueContainer _ServoOutputValues)
{
  int start_time = micros();

  pwmController.setChannelPWM(0, ServoOutput1.pwmForAngle(_ServoOutputValues.Servo1Value));
  pwmController.setChannelPWM(1, ServoOutput2.pwmForAngle(_ServoOutputValues.Servo2Value));
  pwmController.setChannelPWM(2, ServoOutput3.pwmForAngle(_ServoOutputValues.Servo3Value));
  pwmController.setChannelPWM(3, ServoOutput4.pwmForAngle(_ServoOutputValues.Servo4Value));
  RoutineRuntimeValues.SetServoValuesRuntime = micros() - start_time;
}

struct AccelerometerValueContainer GetGyroValues(void)
{
  struct AccelerometerValueContainer _AccelerometerValues;
  int x,y,z;  
	adxl.readXYZ(&x, &y, &z); //read the accelerometer values and store them in variables  x,y,z
  _AccelerometerValues.x = x;
  _AccelerometerValues.y = y;
  _AccelerometerValues.z = z;

/*
// Calculate Roll and Pitch (rotation around X-axis, rotation around Y-axis)
  roll = atan(Y_out / sqrt(pow(X_out, 2) + pow(Z_out, 2))) * 180 / PI;
  pitch = atan(-1 * X_out / sqrt(pow(Y_out, 2) + pow(Z_out, 2))) * 180 / PI;
  // Low-pass filter
  rollF = 0.94 * rollF + 0.06 * roll;
  pitchF = 0.94 * pitchF + 0.06 * pitch;
*/  

  _AccelerometerValues.pitch = atan(y / sqrt(pow(x, 2) + pow(z, 2))) * 180 / PI;
  _AccelerometerValues.roll = atan(-1 * x / sqrt(pow(y, 2) + pow(z, 2))) * 180 / PI;

  return _AccelerometerValues;
}

struct CompassValueContainer GetCompassValues(void)
{
  int start_time = micros();
  struct CompassValueContainer _CompassValues;
  
  compass.read();
  _CompassValues.X = compass.getX() - CompassCorrectionValueX;
  _CompassValues.Y = compass.getY() - CompassCorrectionValueY;
  _CompassValues.Z = compass.getZ() - CompassCorrectionValueZ;

  _CompassValues = CompassValuesAverageFilter(_CompassValues);

  RoutineRuntimeValues.GetCompassValuesRuntime = micros() - start_time;
  return _CompassValues;
}

void GetSerialValues(void)
{
  if(Serial.available())
  {
    Serial.println("Serial Data Detected");
    String SerialCommand;
    SerialCommand = Serial.readStringUntil('X');
    Serial.println(SerialCommand);
    
    if(SerialCommand == "SetLagereglerKp")
    {
      Serial.println("Awaiting Value");
      SerialCommand = Serial.readString();
      double _KpValue = SerialCommand.toFloat();
      Serial.println(_KpValue);
      EEPROM.put(EEPROMLagereglerKp, _KpValue);
      copyPIDValues();
    } 

    if(SerialCommand == "GetLagereglerKp")
    {
      double _KpValue;
      EEPROM.get(EEPROMLagereglerKp, _KpValue);
      Serial.println(_KpValue);
    }  

    if(SerialCommand == "SetLagereglerKi")
    {
      Serial.println("Awaiting Value");
      SerialCommand = Serial.readString();
      double _KiValue = SerialCommand.toFloat();
      Serial.println(_KiValue);
      EEPROM.put(EEPROMLagereglerKi, _KiValue);
      copyPIDValues();
    } 

    if(SerialCommand == "GetLagereglerKi")
    {
      double _KiValue;
      EEPROM.get(EEPROMLagereglerKi, _KiValue);
      Serial.println(_KiValue);
    }   

    if(SerialCommand == "SetLagereglerKd")
    {
      Serial.println("Awaiting Value");
      SerialCommand = Serial.readString();
      double _KdValue = SerialCommand.toFloat();
      Serial.println(_KdValue);
      EEPROM.put(EEPROMLagereglerKd, _KdValue);
      copyPIDValues();
    } 

    if(SerialCommand == "GetLagereglerKd")
    {
      double _KdValue;
      EEPROM.get(EEPROMLagereglerKd, _KdValue);
      Serial.println(_KdValue);
    }    

    if(SerialCommand == "SetTiefenreglerKp")
    {
      Serial.println("Awaiting Value");
      SerialCommand = Serial.readString();
      double _KpValue = SerialCommand.toFloat();
      Serial.println(_KpValue);
      EEPROM.put(EEPROMTiefenreglerKp, _KpValue);
      copyPIDValues();
    } 

    if(SerialCommand == "GetTiefenreglerKp")
    {
      double _KpValue;
      EEPROM.get(EEPROMTiefenreglerKp, _KpValue);
      Serial.println(_KpValue);
    }    

    if(SerialCommand == "SetTiefenreglerKi")
    {
      Serial.println("Awaiting Value");
      SerialCommand = Serial.readString();
      double _KiValue = SerialCommand.toFloat();
      Serial.println(_KiValue);
      EEPROM.put(EEPROMTiefenreglerKd, _KiValue);
      copyPIDValues();
    } 

    if(SerialCommand == "GetTiefenreglerKi")
    {
      double _KiValue;
      EEPROM.get(EEPROMTiefenreglerKd, _KiValue);
      Serial.println(_KiValue);
    }    

    if(SerialCommand == "SetTiefenreglerKd")
    {
      Serial.println("Awaiting Value");
      SerialCommand = Serial.readString();
      double _KdValue = SerialCommand.toFloat();
      Serial.println(_KdValue);
      EEPROM.put(EEPROMTiefenreglerKd, _KdValue);
      copyPIDValues();
    } 

    if(SerialCommand == "GetTiefenreglerKd")
    {
      double _KdValue;
      EEPROM.get(EEPROMTiefenreglerKd, _KdValue);
      Serial.println(_KdValue);
    } 
    
    delay(1000);
  }
}

/****************************************************************************************/
/*                                                                                      */
/*                      Hauptberechnung der Lageregelung                                */
/*                                                                                      */
/****************************************************************************************/

struct ServoValueContainer CalculateReaction(struct ServoValueContainer _ServoInputValues,  struct CompassValueContainer _CompassValues)
{
  int start_time = micros();

  if(_ServoInputValues.Servo1Value < 2)
  {
    if(_ServoInputValues.Servo1Value > -2)
    {
      float temp;
      temp = (float)AccelerometerValues.pitch * 10;

      if(temp > 90) temp = 90;
      if(temp < -90) temp = -90;
      _ServoInputValues.Servo1Value = temp + _ServoInputValues.Servo1Value;
    }
  }
  RoutineRuntimeValues.CalculateReactionRuntime = micros() - start_time;
  return _ServoInputValues;
}

void GetValuesCallback(void)
{
  int start_time = micros();
  GetServoValues();
  //CompassValues = GetCompassValues();
  AccelerometerValues = GetGyroValues();
  float value = adc->adc0->analogRead(A0);
  DepthValue = value; 
  RoutineRuntimeValues.GetValuesCallbackRuntime = micros() - start_time;
}

void DebugValueCallbackInitializer(void)
{
  Serial.println("Servo Eingang 1; Servo Eingang 2; Servo Eingang 3; Servo Eingang 4; Servo Ausgang 1; Servo Ausgang 2; Servo Ausgang 3; Servo Ausgang 4; Compass Azimuth; Compass Bearing; Compass X; Compass Y; Compass Z; Drucksensor");
}

void DebugValueCallback(void)
{

GetSerialValues();

 // Some Change
#ifdef MYDEBUG_VALUE_PLOTTER
  Serial.print(ServoInputValues.Servo1Value);
  Serial.print(";");
  Serial.print(ServoInputValues.Servo2Value);
  Serial.print(";");
  Serial.print(ServoInputValues.Servo3Value);
  Serial.print(";");
  Serial.print(ServoInputValues.Servo4Value);
  Serial.print(";");

  Serial.print(ServoOutputValues.Servo1Value);
  Serial.print(";");
  Serial.print(ServoOutputValues.Servo2Value);
  Serial.print(";");
  Serial.print(ServoOutputValues.Servo3Value);
  Serial.print(";");
  Serial.print(ServoOutputValues.Servo4Value);
  Serial.print(";");


  Serial.print(AccelerometerValues.roll);
  Serial.print(";");
  Serial.print(AccelerometerValues.pitch);
  Serial.print(";");
  Serial.print(AccelerometerValues.x);
  Serial.print(";");
  Serial.print(AccelerometerValues.y);
  Serial.print(";");
  Serial.print(AccelerometerValues.z);
  Serial.print(";");

  Serial.print(DepthValue);
  Serial.print(";");

  Serial.print("leer");
  Serial.println(";");
#endif
}

void copyPIDValues(void)
{
LagereglerKp = EEPROMLagereglerKp;
LagereglerKi = EEPROMLagereglerKi;
LagereglerKd = EEPROMLagereglerKd;

TiefenreglerKp = EEPROMTiefenreglerKp;
TiefenreglerKi = EEPROMTiefenreglerKi;
TiefenreglerKd = EEPROMTiefenreglerKd;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600); 

  DebugThread.onRun(DebugValueCallback);
	DebugThread.setInterval(MYDEBUG_TIMECYCLE);

  GetValuesThread.onRun(GetValuesCallback);
  GetValuesThread.setInterval(21);

  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);

  Wire.begin();
  pwmController.resetDevices();       // Resets all PCA9685 devices on i2c line
  pwmController.init();               // Initializes module using default totem-pole driver mode, and default disabled phase balancer
  pwmController.setPWMFreqServo();    // 50Hz provides standard 20ms servo phase length
  //pwmController.printModuleInfo();

  compass.init();

  adxl.powerOn();

  //set activity/ inactivity thresholds (0-255)
  adxl.setActivityThreshold(75); //62.5mg per increment
  adxl.setInactivityThreshold(75); //62.5mg per increment
  adxl.setTimeInactivity(10); // how many seconds of no activity is inactive?
 
  //look of activity movement on this axes - 1 == on; 0 == off 
  adxl.setActivityX(1);
  adxl.setActivityY(1);
  adxl.setActivityZ(1);
 
  //look of inactivity movement on this axes - 1 == on; 0 == off
  adxl.setInactivityX(1);
  adxl.setInactivityY(1);
  adxl.setInactivityZ(1);
 
  //look of tap movement on this axes - 1 == on; 0 == off
  adxl.setTapDetectionOnX(0);
  adxl.setTapDetectionOnY(0);
  adxl.setTapDetectionOnZ(1);
 
  //set values for what is a tap, and what is a double tap (0-255)
  adxl.setTapThreshold(50); //62.5mg per increment
  adxl.setTapDuration(15); //625us per increment
  adxl.setDoubleTapLatency(80); //1.25ms per increment
  adxl.setDoubleTapWindow(200); //1.25ms per increment
 
  //set values for what is considered freefall (0-255)
  adxl.setFreeFallThreshold(7); //(5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(45); //(20 - 70) recommended - 5ms per increment
 
  //setting all interrupts to take place on int pin 1
  //I had issues with int pin 2, was unable to reset it
  adxl.setInterruptMapping( ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,    ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,     ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT1_PIN );
 
  //register interrupt actions - 1 == on; 0 == off  
  adxl.setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
  adxl.setInterrupt( ADXL345_INT_ACTIVITY_BIT,   1);
  adxl.setInterrupt( ADXL345_INT_INACTIVITY_BIT, 1);

  adc->adc0->setAveraging(16);
  adc->adc0->setResolution(16);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED); 

  copyPIDValues();

  LagereglerInput = 0;
  LagereglerSetpoint = 0;
  LagereglerPID.SetMode(AUTOMATIC);

#ifdef MYDEBUG
  DebugValueCallbackInitializer();
#endif  


}

void loop() {
  
  ServoOutputValues = CalculateReaction(ServoInputValues, CompassValues);
  SetServoValues(ServoOutputValues);

// checks if thread should run
  if(GetValuesThread.shouldRun())
      GetValuesThread.run();

#ifdef MYDEBUG
	if(DebugThread.shouldRun())
		  DebugThread.run();
#endif      
}