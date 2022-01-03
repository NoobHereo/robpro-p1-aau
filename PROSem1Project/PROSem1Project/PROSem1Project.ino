/*
 Name:    PROSem1Project.ino
 Created: 12/28/2021 1:23:16 PM
 Author:  Group B378

 ---- Links ----
 (1) zumo32u4 library by Pololu: https://github.com/pololu/zumo-32u4-arduino-library/blob/master/examples/MazeSolver/TurnSensor.h
 -----------------
*/

#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>

#define DEBUG_MODE true // Must be true in order to log stuff
#define SERIAL_FREQUENCY 9600 // Frequency for the serial monitor
#define MOTOR_SPEED 100 // Default motor speed
#define NUM_SENSORS 5 // The number of line sensors on the board
#define ROTATE_LEFT 'L' // Char value for left turn
#define ROTATE_RIGHT 'R' // Char value for right turn

// All the line sensors identified numerically
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

/// <summary>
/// Debug function for writing text on the LCD if DEBUG MODE is true.
/// </summary>
/// <param name="text"></param>
/// <param name="newLine"></param>
void LCDDebug(String text, bool newLine = true) {
    if (DEBUG_MODE) {
        LCD.clear();
        newLine ? LCD.println(text) : LCD.print(text);
    }
}

/// <summary>
/// Numeric countdown animation on the LCD
/// </summary>
/// <param name="count"></param>
/// <returns></returns>
bool LCDCountdown(int count) {
    LCDDebug((String)count);
    for (int index = count; index > -1; index--) {
        delay(1000);
        LCDDebug((String)index);
    }
    return true;
}

/// <summary>
/// Debug function for logging to the serial monitor if DEBUG MODE is true
/// </summary>
/// <param name="text"></param>
/// <param name="error"></param>
/// <param name="newLine"></param>
void DebugLog(String text, bool error = false, bool newLine = true) {
    if (DEBUG_MODE) {
        String tag = error ? "[ERROR]: " : "[DEBUG]: ";
        Serial.println(tag + text);
    }
}

uint16_t SensorValues[NUM_SENSORS]; // Line sensor values stored in a array

/// <summary>
/// A collection of Line Sensor states for each sensor that are counted by NUM_SENSORS.
/// Sensor values are stored in a uint16_t array 'SensorValues'
/// </summary>
struct LineSensorStates
{
    bool Left, LeftCenter, Center, RightCenter, Right; // Boolean states for each sensor

    /// <summary>
    /// Set's all the sensor states to a common value. This is usually only ever
    /// called to reset the sensor states before a reading.
    /// </summary>
    /// <param name="state"></param>
    void SetAllValues(bool state) {
        Left = LeftCenter = Center = RightCenter = Right = state;
    }

    /// <summary>
    /// Debug function for logging current sensor states as long as DEBUG MODE is true.
    /// </summary>
    void LogStates() {
        DebugLog("L: " + (String)Left + ", LC: " + (String)LeftCenter + ", C: "
            + (String)Center + ", RC: " + (String)RightCenter + "R: " + (String)Right);
    }
};

LineSensorStates LineSensorStates_ = { false, false, false, false, false }; // Reference to the struct
int outerThreshold = NULL; // Threshold for outer sensors, those being: Left & Right
int innerThreshold = NULL; // Threshold for inner sensors, those being: LeftCenter & RightCenter
int centerThreshold = NULL; // Threshold for center sensor, being: Center
bool UseEmitters = true; // Boolean indication wether emission is used on the line sensors or not

bool Operating = false;
bool FirstLineFound = false;
bool CanDetectLineFound = false;
bool CanDetected = false;
bool ReturningHome = false;
bool CanDisposed = false;
bool ReturnLineFound = false;
bool ReturnedHome = false;

int DirectionalAngle = NULL;

// This is the average reading obtained from the gyro's Z axis
// during calibration.
int16_t GyroscopeOffset;

// This variable helps us keep track of how much time has passed
// between readings of the gyro.
uint16_t LastGyroscopeUpdate = NULL;

// turnRate is the current angular rate of the gyro, in units of
// 0.07 degrees per second.
int16_t TurnRate;

uint32_t TurnAngle;
int TurnAngleDegrees;

uint16_t BrightnessLevels[4] = { 1, 2, 3, 4 }; // Brightness levels used as distance measure for identifying can types

/// <summary>
/// Sets the motor based based on the arguments given. If only 1 value is given for speed1
/// that speed will be set on both motors. If two arguments are given as values for both speed1 and rightSpeed
/// each argument value will be applied to each motor left & right. Since turning requires a given speed for one motor and none 
/// for the other 0 cannot be a default value here, thus the value of -1 as it is as close to 0 we can get and the most unlikely speed
/// to ever use. NULL is not an option since it expands to 0 in C.
/// </summary>
/// <param name="speed1"></param>
/// <param name="rightSpeed"></param>
void SetMotorSpeed(int speed1, int rightSpeed = -1) {
    if (rightSpeed == -1) { // If no argument is given to rightSpeed and it remains unchanged from -1
        Motors.setSpeeds(speed1, speed1);
    }
    else {
        Motors.setSpeeds(speed1, rightSpeed);
    }
}

/// <summary>
/// Given a direction 'dir' of either right ('R') or left ('L') this function will turn the robot
/// to a desired angle 'angleDeg' that is expected to be in degrees and not radians.
/// Before each turn the gyroscopes angle will be reset in order to move to a new angle from what it did last.
/// a boolean ensures that the directional angle has either been reached or not, as long as it has not
/// the robot will be stuck in a while loop constructed within the switch that handles the direction.
/// As long as it's turning it will update the gyroscope and turn the robot. This function
/// stops all motor drive by default when the desired angle has been met.
/// The robot will handle any turn direction that is not handled by logging an error if in debug mode.
/// </summary>
/// <param name="dir"></param>
/// <param name="angleDeg"></param>
void Turn(char dir, float angleDeg) {
    DebugLog("Turning: " + (String)angleDeg + " degrees to the " + (String)dir + " direction.");
    bool facingDirection = false;
    GyroscopeReset(); // The global variable that accounts for current rotation functions as a count that needs to be reset before each turn.
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

/// <summary>
/// Called from DetectCan() when a large can is recognized. This function executes
/// a series of hardcoded movement maneuvers for disposing large cans over the correct edge
/// of the platform that is static, meaning it is always expected to be the same drive surface.
/// </summary>
void HandleLargeCan() {
    Turn(ROTATE_RIGHT, 270); // Rotates to look right rather than at the large can
    SetMotorSpeed(MOTOR_SPEED * 0.75); // Starts driving to the right with a slight deviance to the straight line ahead
    delay(3600); // Waits 3.6 seconds before stopping the motors
    Turn('L', 90); // Turns 90 degrees left
    delay(250);
    SetMotorSpeed(MOTOR_SPEED); // Starts driving forward with the robot's left side facing the can
    delay(1200); // Waits 1.2 seconds before stopping motors
    Turn('L', 90); // Turns 90 degrees left again to look directly at the can
    StopMotors(); // Although already stopped this is to account for some weird behavior that would not stop the motors in Turn()
    delay(1000);
    Turn('R', 345); // Accounts for an error where the robot was not really at the angle the gyroscope logged. This makes the robot look directly at the can and ensure collision with the can.
    delay(1000);
    SetMotorSpeed(MOTOR_SPEED); // Drives torwards the can
    while (!CanDisposed) { // Drives into the can until a white line is eventually detected.
        DebugLog("Disposing can..");
        GetLineSensorData();
        if (LineSensorStates_.LeftCenter && LineSensorStates_.RightCenter) {
            DebugLog("Can has been disposed.");
            CanDisposed = true;
        }
    }
    delay(150);
    StopMotors(); // Stops on the white line to not drive of the platform.
    delay(100);
    Turn('L', 80); // Turns 80 degrees to the left in order to drive back to the startline.
    delay(200);
    SetMotorSpeed(MOTOR_SPEED * 0.75);
    delay(200);
    while (!ReturnLineFound) { // Finds the last white line on the side edge of the platform.
        GetLineSensorData();
        if (LineSensorStates_.Right && !LineSensorStates_.Left) {            
            ReturnLineFound = true;           
        }
    }
    delay(1300);
    StopMotors(); // Stops the motors after 1.3 seconds allowing the robot to drive a little further across the line.
    delay(200);
    Turn('L', 90); // Turns 90 degrees left to face the starting line
    delay(200);
    SetMotorSpeed(MOTOR_SPEED); // Drives home
    while (!ReturnedHome) {
        GetLineSensorData();
        if (LineSensorStates_.LeftCenter && LineSensorStates_.RightCenter) {
            ReturnedHome = true;           
        }
    }
    delay(800);
    StopMotors(); // Stops the motors when home
    delay(200);
    Turn('L', 90); // Turns 90 degrees left to look in the direction of any potential cans
    delay(200);
    DetectCan(); // Returns to scanning for cans.
}

/// <summary>
/// Called from DetectCan() when a small can is recognized. This function executes
/// a series of hardcoded movement maneuvers for disposing small cans over the correct edge
/// of the platform that is static, meaning it is always expected to be the same drive surface.
/// </summary>
void HandleSmallCan() {
    SetMotorSpeed(MOTOR_SPEED); // Drives straight ahead to push the can of the edge
    while (!CanDisposed) { // Pushes the can until a white line is detected which indicates the can has been disposed
        DebugLog("Disposing can..");
        GetLineSensorData();
        if (LineSensorStates_.LeftCenter && LineSensorStates_.RightCenter) {
            DebugLog("Can has been disposed.");
            CanDisposed = true;
        }
    }
    delay(150);
    StopMotors(); // Stops the motors on disposal of the can and waits one second.
    delay(1000);
    SetMotorSpeed(-MOTOR_SPEED); // Drives with a negative speed on both motors to return home.
    delay(2200);
    StopMotors(); // Stops the motors after 2.2 seconds which is approximately the time it takes to return
    delay(5000); // The time of return to 'home' is time based as the white line was too thin and reflective to work well with the line sensors
    DetectCan();
}

/// <summary>
/// Main function for handling can detection based on distance measures with infrared proximity sensors.
/// NOTE: This is not really size measuring and is not the correct way to do it, however in this case there is a restriction
/// on hardware that allows for this method of detection since the proximity sensors can only measure the amount of infrared light rays that 
/// is returned compared to how many that was emitted on each emission from each infrared diode that is the sensor.
/// </summary>
void DetectCan() 
{
    // Resets all the values used in each of the can type handlers
    CanDetected = false;
    ReturnedHome = false;

    DebugLog("About to start logging.");
    while (!CanDetected) // While a can is not detected in general despite size/type
    {
        DebugLog("Looking for can");

        // Measuring process of the proximity sensors mentioned in the summary
        ProximitySensors.read();
        int center_left_sensor = ProximitySensors.countsFrontWithLeftLeds();
        int center_right_sensor = ProximitySensors.countsFrontWithRightLeds();

        // If the distance is greater or equal to 4 units = large can
        if (center_left_sensor >= 4 && center_right_sensor >= 4) {
            DebugLog("Found large can");
            HandleLargeCan();
            CanDetected = true;
        }
        // If the distance is less or equal to 3 units = small can
        else if (center_left_sensor <= 3 && center_right_sensor <= 3)
        {
            DebugLog("Found small can");
            HandleSmallCan();
            CanDetected = true;
        }

        delay(250);
    }
}

/// <summary>
/// Setup for the proximity sensors that simply initialises them and set's the brightness levels.
/// </summary>
void ProximitySensorSetup() {
    ProximitySensors.initThreeSensors();
    ProximitySensors.setBrightnessLevels(BrightnessLevels, 4);
}

/// <summary>
/// Setup function for the IMU with the intention of configuring the gyroscope which will be used.
/// </summary>
void GyroscopeSetup() {
    Wire.begin();
    IMU.init();
    IMU.enableDefault();
    IMU.configureForTurnSensing();

    LCDDebug("IMU Cal..");
    ledYellow(true); // Indicates setup/calibration process if LCD is not used.
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
    GyroscopeReset(); // Resets the gyroscope data to ensure a clean start
    delay(250);

    ledYellow(false); // Indicates end of setup/calibration process
    LCD.clear();
}

/// <summary>
/// Resets the gyroscope read data and sets the TurnAngle to 0 that is used to orientate
/// the robot while turning and navigating around the platform.
/// </summary>
void GyroscopeReset() {
    DebugLog("Gyroscope reset");
    LastGyroscopeUpdate = micros();
    TurnAngle = 0;
}

/// <summary>
/// Updates gyroscope data and is continously called from Turn().
/// This function updates TurnAngle and converts it to degrees as long as the gyroscope
/// data is indicated ready by a built in function for the IMU.
/// </summary>
void GyroscopeUpdate() {
    if (!IMU.gyroDataReady()) {
        return; // Exits the function if data is not ready from the IMU
    }

    // Read the measurements from the gyroscope
    IMU.readGyro();
    TurnRate = IMU.g.z - GyroscopeOffset;

    // Figure out how much time has passed since the last update (deltaTime)
    uint16_t micros_ = micros();
    uint16_t deltaTime = micros_ - LastGyroscopeUpdate;
    LastGyroscopeUpdate = micros_;

    // Multiply deltaTime by TurnRate in order to get an estimation of how
    // much the robot has turned since the last update.
    // (angular change = angular velocity * time)
    int32_t d = (int32_t)TurnRate * deltaTime;

    // The units of d are gyro digits times microseconds.  We need
    // to convert those to the units of turnAngle, where 2^29 units
    // represents 45 degrees.  The conversion from gyro digits to
    // degrees per second (dps) is determined by the sensitivity of
    // the gyro: 0.07 degrees per second per digit.
    //
    // (0.07 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
    // = 14680064/17578125 unit/(digit*us)
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

/// <summary>
/// Resets and reads all the sensor data, setting boolean values to true or false
/// for each sensor based on their 'SensorState' see the struct LineSensorStates() in the
/// variable declerations at the top of this file for the struct.
/// </summary>
void GetLineSensorData() {
    LineSensors.read(SensorValues, UseEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);
    LineSensorStates_.SetAllValues(false); // Resets all sensor states by setting them to false
    // LineSensorStates_.LogStates(); // Logs all the states for debugging, currently disabled.
    
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

/// <summary>
/// Calibrates line sensors by defining a thresshold that indicates wether a
/// difference in contrast has been observed or not on the surface area the sensors read from.
/// A margin of error to this reading process is given as '+ 20'
/// </summary>
void CalibrateLineSensors() {
    LCDDebug("Press2Cal");
    delay(250);
    ButtonA.waitForPress();
    LCDDebug("Calibr...");

    ledYellow(true); // Indicates calibration/setup process is ongoing if LCD is not used.
    GetLineSensorData();
    int errorMargin = 20;
    outerThreshold = (SensorValues[LEFT_SENSOR] + SensorValues[RIGHT_SENSOR] / 2) + errorMargin;
    innerThreshold = (SensorValues[CENTER_LEFT_SENSOR] + SensorValues[CENTER_RIGHT_SENSOR] / 2) + errorMargin;
    centerThreshold = SensorValues[CENTER_SENSOR] + errorMargin;
    delay(250);

    ledYellow(false); // Indicates end of calibration/setup process
    LCD.clear();
}

/// <summary>
/// Handles configuration of all components and is called in setup() before anything else
/// to ensure everything is setup and calibrated before the initial drive.
/// Returns a boolean value as soon as the entire function has been iterated through.
/// </summary>
/// <returns></returns>
bool ConfigureComponents() {
  LCD.init();
  LineSensors.initFiveSensors();
  CalibrateLineSensors();
  ProximitySensorSetup();
  GyroscopeSetup();

  DebugLog("Finished configuring..");
  return true;
}

/// <summary>
/// Stops the motors by setting both motor speeds to 0.
/// </summary>
void StopMotors() {
    Motors.setSpeeds(0, 0);
    DebugLog("Stopped motors");
}

// the setup function runs once when you press reset or power the board
void setup() { 
  Serial.begin(SERIAL_FREQUENCY);
  if (ConfigureComponents()) { // Will only iterate through this if statement when ConfigureComponents is done and returns true
    DebugLog("Ready to go.");
    LCDDebug("Ready?");
    delay(250);
    ButtonA.waitForPress();
    bool countdownComplete = LCDCountdown(3); // Makes a visual countdown on the LCD
    if (countdownComplete) { // When countdown is done the robot will move back and drive forward to detect the white line it starts on.
        LCD.clear();
        SetMotorSpeed(-MOTOR_SPEED);
        delay(1000);
        SetMotorSpeed(MOTOR_SPEED);
        Operating = true; // This bool is used in loop() to start looking for cans.
    }
  }
}

// the loop function runs over and over again until power down or reset
void loop() 
{
    if (Operating) // Set to true in the end of the setup function
    {        
        while (!FirstLineFound)
        {
            DebugLog("Searching for the first line");
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
        Turn(ROTATE_RIGHT, 270); // Turns right from the starting line it detected above to look in the direction of the cans
        delay(500);
        SetMotorSpeed(MOTOR_SPEED * 0.5); // Slowly drives forward to trigger the platform's rollboard sensor

        while (!CanDetectLineFound) {
            DebugLog((String)TurnAngleDegrees);
            GetLineSensorData();
            if (LineSensorStates_.LeftCenter && LineSensorStates_.RightCenter) {
                DebugLog("Found line");
                CanDetectLineFound = true;                
            }
            delay(100);
        }
        // DebugLog((String)TurnAngleDegrees); // Debugging for logging the current angle on the LCD
        StopMotors();       
        delay(5000);

        DetectCan(); // Starts looking for cans
        Operating = false; // Stops operating here in order to do a short demonstration for 2 cans
        // this boolean can however be commented out or set to true at the end of any of the can handlers
        // to make a endless cycle of can sorting, this would be ideal for a final product where a 
        // endless stream of cans would roll in on the rolling field of the platform.
    }
}
