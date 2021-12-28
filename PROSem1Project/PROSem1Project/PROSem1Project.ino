/*
 Name:		PROSem1Project.ino
 Created:	12/28/2021 1:23:16 PM
 Author:	Group B378
*/

#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>

#define NUM_SENSORS 5 //Number of activated sensors
#define SERIAL_FREQUENCY 9600
#define DEBUG_MODE false
#define MOTOR_SPEED 100

Zumo32U4Motors Motors;
Zumo32U4LCD LCD;
Zumo32U4Encoders Encoders;
Zumo32U4ButtonA ButtonA;
Zumo32U4IMU IMU;

//Booleans that help the robot know how far in the process it is
bool GoalAngleReached, LineFound, SensorStateCheck;

bool TurnComplete = false; 
bool CanDetected = false; 
bool SmallCanDetected = false; 
bool LargeCanDetected = false; 
bool ReturnedHome = false;

Zumo32U4ProximitySensors ProximitySensors;
Zumo32U4LineSensors LineSensors;
uint16_t SensorValues[NUM_SENSORS]; //Some array that contains the raw read values from the sensors between 0-2000
bool UseEmitters = true;

/// <summary>
/// Datatype that stores the boolean values for the sensorStates
/// </summary>
struct LineSensorsWhite
{
    bool Left;
    bool LeftCenter;
    bool Center;
    bool RightCenter;
    bool Right;
};


//Storage values for linesensor thresholds and for distance measurement
int DrivenDistance;

// threshold1: White threshold, white return values lower than this
// threshold2: white treshold for center 2 sensors
// threshold3: White threshold for center sensor
int threshold1, threshold2, threshold3;

int turnAngleDegrees, flippedturnAngleDegrees;
uint32_t turnAngle = 0;
int16_t turnRate, gyroOffset;
uint16_t gyroLastUpdate = 0;

LineSensorsWhite SensorStates = { false,false,false,false,false };
uint16_t BrightnessLevels[4] = { 1,2,3,4 };

// the setup function runs once when you press reset or power the board
void setup() {
    ConfigureComponents();
    delay(250);
}

// the loop function runs over and over again until power down or reset
void loop() {

    GyroscopeUpdate();
    turnAngleDegrees = ((((int32_t)turnAngle >> 16) * 360) >> 16); //Updates turnAngleDegrees

    //Drive Forward and detect line, follow line and stop on sensor
    FindLine();

    //standby and detect can types
    DetectCan();

    delay(20);
    ReturnToHome();
}

/// <summary>
/// Configures all hardware components before initialising the drive and can detection
/// process
/// </summary>
void ConfigureComponents()
{
    Serial.begin(SERIAL_FREQUENCY); // This is for debugging
    ProximitySensors.initThreeSensors(); //Sets up proximity sensors
    ProximitySensors.setBrightnessLevels(BrightnessLevels, 4);  //Extends measurement levels for proximity sensors to increase overall accuracy

    GyroscopeSetup(); // Sets up the IMU gyroscope
    delay(500);
    GyroscopeReset(); // Resets the IMU gyroscope to prepare for the drive
    LCD.clear();

    LineSensors.initFiveSensors(); // Initializes the five line sensors
    CalibrateLineSensors();
    DebugLog("Press A to start");
    ButtonA.waitForPress();
}

/// <summary>
/// Returns the robot to it's start position
/// </summary>
void ReturnToHome()
{
    while (!ReturnedHome)
    {
        if (SmallCanDetected)
        {
            Motors.setSpeeds(-MOTOR_SPEED, -MOTOR_SPEED);
            delay(200);
            HandleSensorData(SensorStates);
            while (!SensorStates.Left && !SensorStates.Right) 
            {
                HandleSensorData(SensorStates);
                Motors.setSpeeds(-MOTOR_SPEED, -MOTOR_SPEED);
                delay(50);
            }
            Motors.setSpeeds(0, 0);
            ReturnedHome = true;
        }
        else if (LargeCanDetected)
        {
            //Back up off of sensor to stop conveyor
            Motors.setSpeeds(-MOTOR_SPEED * 2, -MOTOR_SPEED * 2);
            delay(100);
            Motors.setSpeeds(0, 0);
            delay(200);

            //Turn 90 degrees left
            Turn(90, 'L');

            //Move forward for x cm
            Encoders.getCountsAndResetLeft();

            delay(50);
            MoveForward(MOTOR_SPEED, MOTOR_SPEED * 0.3);

            //turn 90 degrees left
            Turn(90, 'L');

            //forward until line is detected
            HandleSensorData(SensorStates);
            while (!SensorStates.Left && !SensorStates.Right) 
            {
                HandleSensorData(SensorStates);
                Motors.setSpeeds(MOTOR_SPEED * 0.75, MOTOR_SPEED * 0.75);
                if (!SensorStates.Left && SensorStates.Right) 
                {
                    Motors.setSpeeds(MOTOR_SPEED * 0.75, 0);
                    delay(100);
                }
                else if (SensorStates.Left && !SensorStates.Right) 
                {
                    Motors.setSpeeds(0, MOTOR_SPEED * 0.75);
                    delay(100);
                }
            }
            Motors.setSpeeds(MOTOR_SPEED * 0.75, MOTOR_SPEED * 0.75);
            delay(600);
            Motors.setSpeeds(0, 0);
            delay(200);
            Turn(90, 'L');
            delay(200);

            HandleSensorData(SensorStates);
            Motors.setSpeeds(MOTOR_SPEED * 0.75, MOTOR_SPEED * 0.75);
            while (!SensorStates.Left && !SensorStates.Right)
            {
                HandleSensorData(SensorStates);
                Motors.setSpeeds(MOTOR_SPEED * 0.75, MOTOR_SPEED * 0.75);
                if (!SensorStates.Left && SensorStates.Right) 
                {
                    Motors.setSpeeds(MOTOR_SPEED * 0.75, 0);
                    delay(50);
                }
                else if (SensorStates.Left && !SensorStates.Right) 
                {
                    Motors.setSpeeds(0, MOTOR_SPEED * 0.75);
                    delay(50);
                }
            }
            delay(100);
            Motors.setSpeeds(0, 0);
        }
        ReturnedHome = true;
    }
    CanDetected = false;
    LargeCanDetected = false;
    SmallCanDetected = false;
    ReturnedHome = false;
}

/// <summary>
/// Identifies the object in front of the robot as either a large or small can
/// </summary>
void DetectCan()
{
    while (!CanDetected)
    {
        ProximitySensors.read();
        int center_left_sensor = ProximitySensors.countsFrontWithLeftLeds();
        int center_right_sensor = ProximitySensors.countsFrontWithRightLeds();

        if (center_left_sensor >= 4 && center_right_sensor >= 4)
        {
            HandleLargeCan();
            LargeCanDetected = true;
        }
        else if (center_left_sensor <= 3 && center_right_sensor <= 3)
        {
            HandleSmallCan();
            SmallCanDetected = true;
        }
        CanDetected = true;
    }
}

/// <summary>
/// Handles the registration of a large canned object in front of the robot.
/// The robot will proceed to push the object around the belt and forward into a container/bucket.
/// </summary>
void HandleLargeCan()
{
    //Back up off of sensor to stop conveyor
    Motors.setSpeeds(-MOTOR_SPEED * 2, -MOTOR_SPEED * 2);
    delay(100);
    Motors.setSpeeds(0, 0);
    delay(200);

    //Turn 90 degrees right
    Turn(90, 'R');
    Encoders.getCountsAndResetLeft();

    //Move forward for x cm
    MoveForward(100, 20);

    //turn 90 degrees left
    Turn(90, 'L');

    //move forward for x cm 
    MoveForward(MOTOR_SPEED, MOTOR_SPEED * 0.15);

    //Turn 90 degrees left move 
    Turn(90, 'L');

    //forward until line is detected
    HandleSensorData(SensorStates);

    while (!SensorStates.Left && !SensorStates.Right) 
    {
        HandleSensorData(SensorStates);
        Motors.setSpeeds(MOTOR_SPEED, MOTOR_SPEED);
        delay(50);
    }
    Motors.setSpeeds(0, 0);
}

/// <summary>
/// Handles the registration of a small canned object in front of the robot.
/// The robot will proceed to push the object forward into a container/bucket.
/// </summary>
void HandleSmallCan() {
    Motors.setSpeeds(MOTOR_SPEED, MOTOR_SPEED);
    delay(200);
    HandleSensorData(SensorStates);
    while (!SensorStates.Left && !SensorStates.Right) 
    {
        HandleSensorData(SensorStates);
        Motors.setSpeeds(MOTOR_SPEED, MOTOR_SPEED);
        delay(50);
    }
    Motors.setSpeeds(0, 0);
}

/// <summary>
/// Moves the robot in a fowarqd direction with a fixed angle that has been set during the turn.
/// While a given distance is not yet reached based on the driven distance determined by movement increment
/// the robot will drive in a forward direction until the given direction 'dist' has been reached. 
/// After the distance has been reached it will reset the encoder count and driven distance for later use.
/// </summary>
/// <param name="speeds"></param>
/// <param name="dist"></param>
void MoveForward(int speed, int dist) 
{               
    while (DrivenDistance < dist) // While desired distance travelled is greater than the driven distance
    {
        float countsL = Encoders.getCountsLeft(); // Retrieve motorcounts
        double movementIncm = (countsL / 900) * (PI * 4) - 1.5; // Convert motorcounts to CM            
        Motors.setSpeeds(speed, speed);
        DrivenDistance = movementIncm;
    }
    Motors.setSpeeds(0, 0); // Stops the motors
    Encoders.getCountsAndResetLeft();
    DrivenDistance = 0;
}

/// <summary>
/// Turns the robot to a decired angle in one of either directions, L: Left, R: Right.
/// </summary>
/// <param name="angle"></param>
/// <param name="dir"></param>
void Turn(int angle, char dir)
{
    switch (dir)
    {
        case 'L':
            while (!GoalAngleReached)
            {
                GyroscopeUpdate();     
                turnAngleDegrees = ((((int32_t)turnAngle >> 16) * 360) >> 16);
                LCD.gotoXY(0, 0);
                LCD.print((((int32_t)turnAngle >> 16) * 360) >> 16);
                LCD.print(F("   "));

                if (turnAngleDegrees >= angle && (angle + 1) >= turnAngleDegrees)
                {
                    Motors.setSpeeds(0, 0);
                    GoalAngleReached = true;
                }
                else
                {
                    Motors.setSpeeds(-MOTOR_SPEED * 1.2, MOTOR_SPEED);
                }
            }
            break;

        case 'R':
            while (!GoalAngleReached)
            {
                GyroscopeUpdate();
                turnAngleDegrees = ((((int32_t)turnAngle >> 16) * 360) >> 16);
                LCD.gotoXY(0, 0);
                LCD.print((((int32_t)turnAngle >> 16) * 360) >> 16);
                LCD.print(F("   "));
                flippedturnAngleDegrees = -turnAngleDegrees; //Because degrees are negative to the right, we have to flip the value to some positive integer
                if (flippedturnAngleDegrees >= angle && (angle + 1) >= flippedturnAngleDegrees)
                {
                    Motors.setSpeeds(0, 0);
                    GoalAngleReached = true;
                }
                else
                {
                    Motors.setSpeeds(MOTOR_SPEED, -MOTOR_SPEED * 1.2);
                }
            }
            break;

        default:
            DebugLog("Unknown turn direction registered: " + (String)dir);
            break;
    }

    GyroscopeReset();
    GoalAngleReached = false;
}

/// <summary>
/// Drives the robot forward until a white line is detected. This is used for the initial
/// start and preperation stage for the robot.
/// </summary>
void FindLine() 
{
    while (!LineFound) 
    {
        Motors.setSpeeds(MOTOR_SPEED * 0.75, MOTOR_SPEED * 0.75);
        HandleSensorData(SensorStates);
        if (SensorStates.Left && SensorStates.Right) 
        {
            delay(600);
            Motors.setSpeeds(0, 0);
            LineFound = true;
        }
        else if (!SensorStates.Left && SensorStates.Right) 
        {
            Motors.setSpeeds(MOTOR_SPEED, 0);
            delay(10);
        }
        else if (SensorStates.Left && !SensorStates.Right) 
        {
            Motors.setSpeeds(0, MOTOR_SPEED);
            delay(10);
        }
    }

    while (LineFound && !SensorStateCheck) 
    {
        if (!TurnComplete) {
            Turn(90, 'R');
            TurnComplete = true;
        }

        Motors.setSpeeds(MOTOR_SPEED * 0.75, MOTOR_SPEED);
        HandleSensorData(SensorStates);

        if (SensorStates.Left && SensorStates.Right) {
            delay(200);
            Motors.setSpeeds(0, 0);
            SensorStateCheck = true;
        }
        else if (!SensorStates.Left && SensorStates.Right) {
            Motors.setSpeeds(MOTOR_SPEED, 0);
            delay(100);
        }
        else if (SensorStates.Left && !SensorStates.Right) {
            Motors.setSpeeds(0, MOTOR_SPEED);
            delay(100);
        }
    }
    Motors.setSpeeds(0, 0);
    TurnComplete = false;
}

/// <summary>
/// Calibrates the line sensors by reading average thresholds based on the surface
/// the robot scans during calibration.
/// </summary>
void CalibrateLineSensors()
{
    ButtonA.waitForPress();
    LCD.print("Prs A Cal");
    delay(250);
    ButtonA.waitForPress();
    HandleSensorData(SensorStates);
    threshold1 = ((SensorValues[0] + SensorValues[4]) / 2 + 20); //takes the mean value of far left and right sensors and adds some margin to create a threshold
    threshold2 = ((SensorValues[1] + SensorValues[3]) / 2 + 20);
    threshold3 = (SensorValues[2] + 20);
    delay(250);
    Serial.println(threshold1); //prints threshold once at the beginning of the code
    Serial.println(threshold2);
    LCD.clear();
}

/// <summary>
/// Fills struct with values from sensorValues and compares to thresholds
/// Next line reads the sensor values and store them in the array lineSensorValues
/// </summary>
/// <param name="state"></param>
void HandleSensorData(LineSensorsWhite& state) 
{
    LineSensors.read(SensorValues, UseEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF); //Retrieves data from sensors
    state = { false,false,false,false,false }; // state of the sensors is ALWAYS set to negative in the structure, so that the if statements below only change the boolean to true when the conditions are met
    if (SensorValues[0] < threshold1) {
        state.Left = true;
    }
    if (SensorValues[1] < threshold2) {
        state.LeftCenter = true;
    }
    if (SensorValues[2] < threshold3) {
        state.Center = true;
    }
    if (SensorValues[3] < threshold2) {
        state.RightCenter = true;
    }
    if (SensorValues[4] < threshold1) {
        state.Right = true;
    }
}

/// <summary>
/// Setup function for the Zumo32U4's IMU and Gyroscope
/// </summary>
void GyroscopeSetup()
{
    Wire.begin();
    IMU.init();
    IMU.enableDefault();
    IMU.configureForTurnSensing();

    LCD.clear();
    LCD.print(F("Gyro cal"));

    // Turn on the yellow LED in case the LCD is not available.
    ledYellow(1);

    // Delay to give the user time to remove their finger.
    delay(500);

    // Calibrate the gyro.
    int sampleIterations = 1024;
    int32_t total = 0;
    for (uint16_t i = 0; i < sampleIterations; i++)
    {
        // Wait for new data to be available, then read it.
        while (!IMU.gyroDataReady()) {}
        IMU.readGyro();

        // Add the Z axis reading to the total.
        total += IMU.g.z;
    }
    ledYellow(0);
    gyroOffset = total / sampleIterations;

    // Display the angle (in degrees from -180 to 180) until the
    // user presses A.
    LCD.clear();
}

/// <summary>
/// Resets the gyroscope angle and sets the last update to be TODO: Micros
/// </summary>
void GyroscopeReset()
{
    gyroLastUpdate = micros();
    turnAngle = 0;
}

/// <summary>
/// IMU Gyrscope setup function. TODO: Fix all magic numbers here.
/// </summary>
void GyroscopeUpdate()
{
    IMU.readGyro();
    turnRate = IMU.g.z - gyroOffset;
    uint16_t m = micros();
    uint16_t dt = m - gyroLastUpdate;
    gyroLastUpdate = m;

    // Multiply dt by turnRate in order to get an estimation of how
    // much the robot has turned since the last update.
    // (angular change = angular velocity * time)
    int32_t d = (int32_t)turnRate * dt;

    // The units of d are gyro digits times microseconds. We need
    // to convert those to the units of turnAngle, where 2^29 units
    // represents 45 degrees. The conversion from gyro digits to
    // degrees per second (dps) is determined by the sensitivity of
    // the gyro: 0.07 degrees per second per digit.
    //
    // (0.07 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
    // = 14680064/17578125 unit/(digit*us)
    turnAngle += (int64_t)d * 14680064 / 17578125;
}

/// <summary>
/// Debug logging that prints text to the serial monitor if the robot is in DEBUG mode
/// </summary>
/// <param name="text"></param>
/// <param name="newLine"></param>
void DebugLog(String text, bool newLine = true)
{
    if (DEBUG_MODE) {
        Serial.println("[DEBUG]" + text);
    }
}

