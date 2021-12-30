/*
 Name:    PROSem1Project.ino
 Created: 12/28/2021 1:23:16 PM
 Author:  Group B378
*/

#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>

#define DEBUG_MODE true
#define SERIAL_FREQUENCY 9600
#define MOTOR_SPEED 100
#define NUM_SENSORS 5
#define ROTATE_LEFT 'L'
#define ROTATE_RIGHT 'R'

#define LEFT_SENSOR 0
#define CENTER_LEFT_SENSOR 1
#define CENTER_SENSOR 2
#define CENTER_RIGHT_SENSOR 3
#define RIGHT_SENSOR 4

Zumo32U4Motors Motors;
Zumo32U4LineSensors LineSensors;
Zumo32U4LCD LCD;
Zumo32U4ButtonA ButtonA;
Zumo32U4IMU IMU;
Zumo32U4ProximitySensors ProximitySensors;

void LCDDebug(String text, bool newLine = true) {
    if (DEBUG_MODE) {
        LCD.clear();
        newLine ? LCD.println(text) : LCD.print(text);
    }
}

bool LCDCountdown(int count) {
    LCDDebug((String)count);
    for (int index = count; index > -1; index--) {
        delay(1000);
        LCDDebug((String)index);
    }
    return true;
}

void DebugLog(String text, bool error = false, bool newLine = true) {
    if (DEBUG_MODE) {
        String tag = error ? "[ERROR]: " : "[DEBUG]: ";
        Serial.println(tag + text);
    }
}

uint16_t SensorValues[NUM_SENSORS];

struct LineSensorStates
{
    bool Left, LeftCenter, Center, RightCenter, Right;

    void SetAllValues(bool state) {
        Left = LeftCenter = Center = RightCenter = Right = state;
    }

    void LogStates() {
        DebugLog("L: " + (String)Left + ", LC: " + (String)LeftCenter + ", C: "
            + (String)Center + ", RC: " + (String)RightCenter + "R: " + (String)Right);
    }
};

LineSensorStates LineSensorStates_ = { false, false, false, false, false };
int outerThreshold = NULL;
int innerThreshold = NULL;
int centerThreshold = NULL;
bool UseEmitters = true;

bool Operating = false;
bool FirstLineFound = false;
bool CanDetectLineFound = false;
bool CanDetected = false;
bool SmallCanDetected = false;
bool LargeCanDetected = false;
bool ReturningHome = false;
bool CanDisposed = false;
bool ReturnLineFound = false;
bool ReturnedHome = false;

int DirectionalAngle = NULL;
int16_t GyroscopeOffset;
uint16_t LastGyroscopeUpdate;
int16_t TurnRate;
uint32_t TurnAngle;
int TurnAngleDegrees;
int TurnAngleDegreesInverted;

uint16_t BrightnessLevels[4] = { 1, 2, 3, 4 };

void SetMotorSpeed(int speed1, int rightSpeed = -1) {
    if (rightSpeed == -1) {
        Motors.setSpeeds(speed1, speed1);
    }
    else {
        Motors.setSpeeds(speed1, rightSpeed);
    }
}

void Turn(char dir, float angleDeg) {
    DebugLog("Turning: " + (String)angleDeg + " degrees to the " + (String)dir + " direction.");
    bool facingDirection = false;
    GyroscopeReset();
    switch (dir)
    {
    case 'L':
        while (!facingDirection)
        {
            GyroscopeUpdate();
            SetMotorSpeed(-MOTOR_SPEED * 1.2, MOTOR_SPEED);

            if (DirectionalAngle >= angleDeg && (angleDeg) >= DirectionalAngle)
                facingDirection = true;
        }
        StopMotors();
        DebugLog("Rotation done");
        break;

    case 'R':
        while (!facingDirection)
        {
            GyroscopeUpdate();
            //TurnAngleDegreesInverted = -DirectionalAngle; //Because degrees are negative to the right, we have to flip the value to some positive integer
            SetMotorSpeed(MOTOR_SPEED, -MOTOR_SPEED * 1.2);

            if (DirectionalAngle >= angleDeg && (angleDeg) >= DirectionalAngle)
                facingDirection = true;
        }
        StopMotors();
        DebugLog("Rotation done");
        break;

    default:
        DebugLog("Unhandled rotational direction: " + dir, true);
        break;
    }
}

void HandleLargeCan() {
    CanDisposed = false;
    Turn(ROTATE_RIGHT, 270);
    SetMotorSpeed(MOTOR_SPEED * 0.75);
    delay(3600);
    Turn('L', 90);
    delay(250);
    SetMotorSpeed(MOTOR_SPEED);
    delay(1200);
    Turn('L', 90);
    StopMotors();
    delay(1000);
    Turn('R', 345);
    delay(1000);
    SetMotorSpeed(MOTOR_SPEED);
    while (!CanDisposed) {
        DebugLog("Disposing can..");
        GetLineSensorData();
        if (LineSensorStates_.LeftCenter && LineSensorStates_.RightCenter) {
            DebugLog("Can has been disposed.");
            CanDisposed = true;
        }
    }
    delay(150);
    StopMotors();
    delay(100);
    Turn('L', 80);
    delay(200);
    SetMotorSpeed(MOTOR_SPEED * 0.75);
    delay(200);
    while (!ReturnLineFound) {
        GetLineSensorData();
        if (LineSensorStates_.Right && !LineSensorStates_.Left) {            
            ReturnLineFound = true;           
        }
    }
    delay(1300);
    StopMotors();
    delay(200);
    Turn('L', 90);
    delay(200);
    SetMotorSpeed(MOTOR_SPEED);
    while (!ReturnedHome) {
        GetLineSensorData();
        if (LineSensorStates_.LeftCenter && LineSensorStates_.RightCenter) {
            ReturnedHome = true;           
        }
    }
    delay(800);
    StopMotors();
    delay(200);
    Turn('L', 90);
    delay(200);
    DetectCan();
}

void HandleSmallCan() {
    CanDisposed = false;
    ReturnedHome = false;

    SetMotorSpeed(MOTOR_SPEED);
    while (!CanDisposed) {
        DebugLog("Disposing can..");
        GetLineSensorData();
        if (LineSensorStates_.LeftCenter && LineSensorStates_.RightCenter) {
            DebugLog("Can has been disposed.");
            CanDisposed = true;
        }
    }
    delay(150);
    StopMotors();
    delay(1000);
    SetMotorSpeed(-MOTOR_SPEED);
    //while (!ReturnedHome) {
    //    GetLineSensorData();
    //    if (LineSensorStates_.LeftCenter && LineSensorStates_.RightCenter) {
    //        DebugLog("Can has been disposed.");
    //        ReturnedHome = true;
    //    }
    //}
    delay(2200);
    StopMotors();
    delay(5000);
    DetectCan();
}

void DetectCan() 
{
    CanDetected = false;
    LargeCanDetected = false;
    SmallCanDetected = false;
    ReturnedHome = false;

    DebugLog("About to start logging.");
    while (!CanDetected)
    {
        DebugLog("Looking for can");
        ProximitySensors.read();
        int center_left_sensor = ProximitySensors.countsFrontWithLeftLeds();
        int center_right_sensor = ProximitySensors.countsFrontWithRightLeds();

        if (center_left_sensor >= 4 && center_right_sensor >= 4) {
            DebugLog("Found large can");
            HandleLargeCan();
            LargeCanDetected = false;
            CanDetected = true;
        }
        else if (center_left_sensor <= 3 && center_right_sensor <= 3)
        {
            DebugLog("Found small can");
            HandleSmallCan();
            SmallCanDetected = false;
            CanDetected = true;
        }

        delay(250);
    }
}

void ProximitySensorSetup() {
    ProximitySensors.initThreeSensors();
    ProximitySensors.setBrightnessLevels(BrightnessLevels, 4);
}

void GyroscopeSetup() {
    Wire.begin();
    IMU.init();
    IMU.enableDefault();
    IMU.configureForTurnSensing();

    LCDDebug("IMU Cal..");
    ledYellow(true);
    delay(250);

    int32_t totalReadings = 0;
    int sampleIterations = 1024;
    for (uint16_t iteration = 0; iteration < sampleIterations; iteration++) 
    {
        if (!IMU.gyroDataReady()) {
            return;
        }

        IMU.readGyro();
        totalReadings += IMU.g.z;
    }
    GyroscopeOffset = totalReadings / sampleIterations;
    GyroscopeReset();
    delay(250);

    ledYellow(false);
    LCD.clear();
}

void GyroscopeReset() {
    DebugLog("Gyroscope reset");
    LastGyroscopeUpdate = micros();
    TurnAngle = 0;
}

void GyroscopeUpdate() {
    if (!IMU.gyroDataReady()) {
        return;
    }

    IMU.readGyro();
    TurnRate = IMU.g.z - GyroscopeOffset;
    uint16_t micros_ = micros();
    uint16_t deltaTime = micros_ - LastGyroscopeUpdate;
    LastGyroscopeUpdate = micros_;

    int32_t d = (int32_t)TurnRate * deltaTime;
    TurnAngle += (int64_t)d * 14680064 / 17578125;

    // Converts angle interval from [-180;180] to [0;360]
    TurnAngleDegrees = ((((int32_t)TurnAngle >> 16) * 360) >> 16);
    if (TurnAngleDegrees <= 0 && TurnAngleDegrees >= -180) {
        DirectionalAngle = (360 + TurnAngleDegrees);
    }
    if (TurnAngleDegrees >= 0 && TurnAngleDegrees <= 180) {
        DirectionalAngle = TurnAngleDegrees;
    }
}

void GetLineSensorData() {
    LineSensors.read(SensorValues, UseEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);
    LineSensorStates_.SetAllValues(false);
    // LineSensorStates_.LogStates();
    
    if (SensorValues[LEFT_SENSOR] < outerThreshold) {
        LineSensorStates_.Left = true;
    }
    if (SensorValues[CENTER_LEFT_SENSOR] < innerThreshold) {
        LineSensorStates_.LeftCenter = true;
    }
    if (SensorValues[CENTER_SENSOR] < centerThreshold) {
        LineSensorStates_.Center = true;
    }
    if (SensorValues[CENTER_RIGHT_SENSOR] < innerThreshold) {
        LineSensorStates_.RightCenter = true;
    }
    if (SensorValues[RIGHT_SENSOR] < outerThreshold) {
        LineSensorStates_.Right = true;
    }
}

void CalibrateLineSensors() {
    LCDDebug("Press2Cal");
    delay(250);
    ButtonA.waitForPress();
    LCDDebug("Calibr...");

    ledYellow(true);
    GetLineSensorData();
    outerThreshold = (SensorValues[LEFT_SENSOR] + SensorValues[RIGHT_SENSOR] / 2) + 20;
    innerThreshold = (SensorValues[CENTER_LEFT_SENSOR] + SensorValues[CENTER_RIGHT_SENSOR] / 2) + 20;
    centerThreshold = SensorValues[CENTER_SENSOR] + 20;
    delay(250);

    ledYellow(false);
    LCD.clear();
}

bool ConfigureComponents() {
  LCD.init();
  LineSensors.initFiveSensors();
  CalibrateLineSensors();
  ProximitySensorSetup();
  GyroscopeSetup();

  DebugLog("Finished configuring..");
  return true;
}

void StopMotors() {
    Motors.setSpeeds(0, 0);
    DebugLog("Stopped motors");
}

void setup() { 
  Serial.begin(SERIAL_FREQUENCY);
  if (ConfigureComponents()) {
    DebugLog("Ready to go.");
    LCDDebug("Ready?");
    delay(250);
    ButtonA.waitForPress();
    bool countdownComplete = LCDCountdown(3);
    if (countdownComplete) {
        LCD.clear();
        SetMotorSpeed(-MOTOR_SPEED);
        delay(1000);
        SetMotorSpeed(MOTOR_SPEED);
        Operating = true;
    }
  }
}

// the loop function runs over and over again until power down or reset
void loop() 
{
    if (Operating) 
    {        
        while (!FirstLineFound)
        {
            DebugLog((String)TurnAngleDegrees);
            //DebugLog("Searching for the first line");
            GetLineSensorData();
            if (LineSensorStates_.LeftCenter && LineSensorStates_.RightCenter) {
                DebugLog("First line found!");
                FirstLineFound = true;
            }
            delay(250);
        }

        DebugLog((String)TurnAngleDegrees);
        SetMotorSpeed(0);
        delay(250);
        Turn(ROTATE_RIGHT, 270);
        delay(500);
        SetMotorSpeed(MOTOR_SPEED * 0.5);

        while (!CanDetectLineFound) {
            DebugLog((String)TurnAngleDegrees);
            // GyroscopeReset();
            // DebugLog("Looking for line 2");
            GetLineSensorData();
            if (LineSensorStates_.LeftCenter && LineSensorStates_.RightCenter) {                
                CanDetectLineFound = true;                
            }
            delay(100);
        }
        DebugLog((String)TurnAngleDegrees);
        StopMotors();
        DebugLog("Time for a break lel xd");        
        delay(5000);

        DetectCan();
        Operating = false;
    }
}
