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

void DebugLog(String text, bool newLine = true) {
    if (DEBUG_MODE) {
        Serial.println("[DEBUG]: " + text);
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

int DirectionalAngle = NULL;
int16_t GyrscopeOffset;
uint16_t LastGyroscopeUpdate;

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
    GyrscopeOffset = totalReadings / sampleIterations;
    GyroscopeReset();
    delay(250);

    ledYellow(false);
    LCD.clear();
}

void GyroscopeReset() {
    LastGyroscopeUpdate = micros();
    DirectionalAngle = NULL;
}

void GetLineSensorData() {
    LineSensors.read(SensorValues, UseEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);
    LineSensorStates_.SetAllValues(false);
    LineSensorStates_.LogStates();
    
    if (SensorValues[0] < outerThreshold) {
        LineSensorStates_.Left = true;
    }
    if (SensorValues[1] < innerThreshold) {
        LineSensorStates_.LeftCenter = true;
    }
    if (SensorValues[2] < centerThreshold) {
        LineSensorStates_.Center = true;
    }
    if (SensorValues[3] < innerThreshold) {
        LineSensorStates_.RightCenter = true;
    }
    if (SensorValues[4] < outerThreshold) {
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
  GyroscopeSetup();

  DebugLog("Finished configuring..");
  return true;
}

void Turn(char dir, float angleDeg) {
    DebugLog("Turning: " + (String)angleDeg + " degrees to the " + (String)dir + " direction.");
}

void SetMotorSpeed(int speed1, int rightSpeed = -1) {
  if (rightSpeed == -1) {
    Motors.setSpeeds(speed1, speed1);
  }
  else {
    Motors.setSpeeds(speed1, rightSpeed);
  }
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
            DebugLog("Searching...");
            GetLineSensorData();
            if (LineSensorStates_.Left && LineSensorStates_.Right) {
                DebugLog("First line found!");
                FirstLineFound = true;
            }
            delay(250);
        }

        SetMotorSpeed(0);
        Turn(ROTATE_RIGHT, 90);
        while (!CanDetectLineFound) {
            DebugLog("Looking for can detection line...");

            delay(1250);
        }
    }

    DebugLog("Found line!");
    delay(500);
}
