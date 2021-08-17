void ServoSignal1Falling(void);
void ServoSignal1Rising(void);

void ServoSignal2Falling(void);
void ServoSignal2Rising(void);

void ServoSignal3Falling(void);
void ServoSignal3Rising(void);

void ServoSignal4Falling(void);
void ServoSignal4Rising(void);

void copyPIDValues(void);
void DebugValueCallback(void);
void DebugValueCallbackInitializer(void);
void GetValuesCallback(void);
struct ServoValueContainer CalculateReaction(struct ServoValueContainer _ServoInputValues,  struct CompassValueContainer _CompassValues);
void GetSerialValues(void);
struct CompassValueContainer GetCompassValues(void);
struct AccelerometerValueContainer GetGyroValues(void);
void SetServoValues(struct ServoValueContainer _ServoOutputValues);
void GetServoValues(void);