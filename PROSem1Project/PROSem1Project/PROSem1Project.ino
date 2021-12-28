/*
 Name:		PROSem1Project.ino
 Created:	12/28/2021 1:23:16 PM
 Author:	Group B378
*/

#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4LCD lcd;
Zumo32U4Encoders encoders;
Zumo32U4ButtonA buttonA;
Zumo32U4IMU imu;

//Booleans that help the robot know how far in the process it is
bool rotatCheck, findLine, findIR, turned, canDetected, smallCanDetected, largeCanDetected, returnedHome, almostHome = false;

//////////////////////// SENSORS ////////////////////////
#define NUM_SENSORS 5 //Number of activated sensors
Zumo32U4ProximitySensors proxSensors;
Zumo32U4LineSensors lineSensors;
uint16_t sensorValues[NUM_SENSORS]; //Some array that contains the raw read values from the sensors between 0-2000
bool useEmitters = true;

struct LineSensorsWhite //Datatype that stores the boolean values for the sensorStates
{
    bool left;
    bool leftCenter;
    bool Center;
    bool rightCenter;
    bool right;
};

////////////////////////////////////////////////////////////

//Storage values for linesensor thresholds and for distance measurement
int drivenDist, threshold, threshold2, threshold3;

// threshold: White threshold, white return values lower than this
// threshold2: white treshold for center 2 sensors
// threshold3: White threshold for center sensor

////////////////Gyro setup/////////////////////
int turnAngleDegrees, flippedturnAngleDegrees;
uint32_t turnAngle = 0;
int turnAngle2 = turnAngle;
int16_t turnRate, gyroOffset;
uint16_t gyroLastUpdate = 0;

LineSensorsWhite sensorState = { false,false,false,false,false };
uint16_t brightnessLevels[4] = { 1,2,3,4 };

// the setup function runs once when you press reset or power the board
void setup() {
    configureComponents();
    delay(250);
}

// the loop function runs over and over again until power down or reset
void loop() {

    turnSensorUpdate();
    turnAngleDegrees = ((((int32_t)turnAngle >> 16) * 360) >> 16); //Updates turnAngleDegrees

      //Drive Forward and detect line, follow line and stop on sensor
    findLineAndSensor();

    //standby and detect can types
    detectCan();

    delay(20);
    returnHome();
}

/////////////////////// FUNCTIONS ///////////////////////

void configureComponents()
{
    Serial.begin(9600);
    proxSensors.initThreeSensors(); //Sets up proximity sensors
    proxSensors.setBrightnessLevels(brightnessLevels, 4);  //Extends measurement levels for proximity sensors to increase overall accuracy

    turnSensorSetup(); //Sets up turning sensors
    delay(500);
    turnSensorReset();
    lcd.clear();

    lineSensors.initFiveSensors(); // Initializes five sensors
    calibrWht();
    Serial.println("Press A to start");
    buttonA.waitForPress();
}

void returnHome()
{
    while (!returnedHome)
    {
        if (smallCanDetected)
        {
            motors.setSpeeds(-100, -100);
            delay(200);
            readSensors(sensorState);
            while (!sensorState.left && !sensorState.right) {
                readSensors(sensorState);
                motors.setSpeeds(-100, -100);
                delay(50);
            }
            motors.setSpeeds(0, 0);
            returnedHome = true;
        }
        else if (largeCanDetected && !almostHome)
        {
            //Back up off of sensor to stop conveyor
            motors.setSpeeds(-200, -200);
            delay(100);
            motors.setSpeeds(0, 0);
            delay(200);

            //Turn 90 degrees left
            turn90L();
            //Move forward for x cm
            encoders.getCountsAndResetLeft();
            delay(50);
            moveForward(100, 30);
            //turn 90 degrees left
            turn90L();

            //forward until line is detected
            readSensors(sensorState);
            while (!sensorState.left && !sensorState.right) {
                readSensors(sensorState);
                motors.setSpeeds(75, 75);
                if (!sensorState.left && sensorState.right) {
                    motors.setSpeeds(75, 0);
                    delay(100);
                }
                else if (sensorState.left && !sensorState.right) {
                    motors.setSpeeds(0, 75);
                    delay(100);
                }
            }
            motors.setSpeeds(75, 75);
            delay(600);
            motors.setSpeeds(0, 0);
            delay(200);
            turn90L();
            delay(200);

            readSensors(sensorState);
            motors.setSpeeds(75, 75);
            while (!sensorState.left && !sensorState.right) {
                readSensors(sensorState);
                motors.setSpeeds(75, 75);
                if (!sensorState.left && sensorState.right) {
                    motors.setSpeeds(75, 0);
                    delay(50);
                }
                else if (sensorState.left && !sensorState.right) {
                    motors.setSpeeds(0, 75);
                    delay(50);
                }
            }
            delay(100);
            motors.setSpeeds(0, 0);
        }
        returnedHome = true;
    }
    canDetected = false;
    largeCanDetected = false;
    smallCanDetected = false;
    returnedHome = false;
    almostHome = false;
}

//Function for detecing can type and initializing can pushing sequences
void detectCan()
{
    while (!canDetected)
    {
        proxSensors.read();
        int center_left_sensor = proxSensors.countsFrontWithLeftLeds();
        int center_right_sensor = proxSensors.countsFrontWithRightLeds();

        if (center_left_sensor == 4 && center_right_sensor == 4)
        {
            largeCan();
            largeCanDetected = true;
        }
        else if (center_left_sensor <= 3 && center_right_sensor <= 3)
        {
            smallCan();
            smallCanDetected = true;
        }
        canDetected = true;
    }
}

//Function for pushing large can into bin. (Goes around the belt and pushes)
void largeCan()
{
    //Back up off of sensor to stop conveyor
    motors.setSpeeds(-200, -200);
    delay(100);
    motors.setSpeeds(0, 0);
    delay(200);

    //Turn 90 degrees right
    turn90R();
    encoders.getCountsAndResetLeft();
    //Move forward for x cm
    moveForward(100, 20);
    //turn 90 degrees left
    turn90L();
    //move forward for x cm 
    moveForward(100, 15);
    //Turn 90 degrees left move 
    turn90L();
    //forward until line is detected
    readSensors(sensorState);
    while (!sensorState.left && !sensorState.right) {
        readSensors(sensorState);
        motors.setSpeeds(100, 100);
        delay(50);
    }
    motors.setSpeeds(0, 0);
}

//Pushing function for small can (Push forward)  
void smallCan() {
    motors.setSpeeds(100, 100);
    delay(200);
    readSensors(sensorState);
    while (!sensorState.left && !sensorState.right) {
        readSensors(sensorState);
        motors.setSpeeds(100, 100);
        delay(50);
    }
    motors.setSpeeds(0, 0);
}

//Reuseable function for driving distances in cm
void moveForward(int speeds, int dist) {                //Functions for moving directions, expecting three values. One for speed and one for time
    while (drivenDist < dist) {              //while desired distance travelled is greater than the actually travelled distance
        float countsL = encoders.getCountsLeft();               //Retrieve motorcounts
        double movementIncm = (countsL / 900) * (PI * 4) - 1.5; //Convert motorcounts to CM            
        motors.setSpeeds(speeds, speeds);                        //Set positive speeds to go forward
        drivenDist = movementIncm;
    }
    motors.setSpeeds(0, 0);                                  //turn off motors
    encoders.getCountsAndResetLeft();                       //Reset encodercount for future use
    drivenDist = 0;
}

//Funtion for making a 90 degree turn to the right
void turn90R()
{
    while (!rotatCheck) {
        turnSensorUpdate();
        turnAngleDegrees = ((((int32_t)turnAngle >> 16) * 360) >> 16);
        lcd.gotoXY(0, 0);
        lcd.print((((int32_t)turnAngle >> 16) * 360) >> 16);
        lcd.print(F("   "));
        flippedturnAngleDegrees = -turnAngleDegrees; //Because degrees are negative to the right, we have to flip the value to some positive integer
        if (flippedturnAngleDegrees >= 90 && 91 >= flippedturnAngleDegrees)
        {
            motors.setSpeeds(0, 0);
            rotatCheck = true;
        }
        else
        {
            motors.setSpeeds(100, -120);
        }
    }
    turnSensorReset();
    rotatCheck = false;
}

//Function for making a 90 degree turn to the left.
void turn90L()
{
    while (!rotatCheck) {
        turnSensorUpdate();
        turnAngleDegrees = ((((int32_t)turnAngle >> 16) * 360) >> 16);
        lcd.gotoXY(0, 0);
        lcd.print((((int32_t)turnAngle >> 16) * 360) >> 16);
        lcd.print(F("   "));

        if (turnAngleDegrees >= 90 && 91 >= turnAngleDegrees)
        {
            motors.setSpeeds(0, 0);
            rotatCheck = true;
        }
        else
        {
            motors.setSpeeds(-120, 100);
        }
    }
    turnSensorReset();
    rotatCheck = false;
}

//Initial function that finds the IR sensor
void findLineAndSensor() {
    while (!findLine) {
        motors.setSpeeds(75, 75);
        readSensors(sensorState);
        if (sensorState.left && sensorState.right) {
            delay(600);
            motors.setSpeeds(0, 0);
            findLine = true;
        }
        else if (!sensorState.left && sensorState.right) {
            motors.setSpeeds(100, 0);
            delay(10);
        }
        else if (sensorState.left && !sensorState.right) {
            motors.setSpeeds(0, 100);
            delay(10);
        }
    }

    while (findLine && !findIR) { //
        if (!turned) {
            turn90R();
            turned = true;
        }

        motors.setSpeeds(75, 100);
        readSensors(sensorState);

        if (sensorState.left && sensorState.right) {
            delay(200);
            motors.setSpeeds(0, 0);
            findIR = true;
        }
        else if (!sensorState.left && sensorState.right) {
            motors.setSpeeds(100, 0);
            delay(100);
        }
        else if (sensorState.left && !sensorState.right) {
            motors.setSpeeds(0, 100);
            delay(100);
        }
    }
    motors.setSpeeds(0, 0);
    turned = false;
}

//Calibration function for white lines
void calibrWht() { //A funtion that is called in the setup to help calibrate sensors for the conditions at hand
    buttonA.waitForPress();
    lcd.print("Prs A Cal");
    delay(250);
    buttonA.waitForPress();
    readSensors(sensorState);
    threshold = ((sensorValues[0] + sensorValues[4]) / 2 + 20); //takes the mean value of far left and right sensors and adds some margin to create a threshold
    threshold2 = ((sensorValues[1] + sensorValues[3]) / 2 + 20);
    threshold3 = (sensorValues[2] + 20);
    delay(250);
    Serial.println(threshold); //prints threshold once at the beginning of the code
    Serial.println(threshold2);
    lcd.clear();
}

//Function that fills struct with values from sensorValues and compares to thresholds
void readSensors(LineSensorsWhite& state) {  // Next line reads the sensor values and store them in the array lineSensorValues
    lineSensors.read(sensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF); //Retrieves data from sensors
    state = { false,false,false,false,false }; // state of the sensors is ALWAYS set to negative in the structure, so that the if statements below only change the boolean to true when the conditions are met
    if (sensorValues[0] < threshold) {
        state.left = true;
    }
    if (sensorValues[1] < threshold2) {
        state.leftCenter = true;
    }
    if (sensorValues[2] < threshold3) {
        state.Center = true;
    }
    if (sensorValues[3] < threshold2) {
        state.rightCenter = true;
    }
    if (sensorValues[4] < threshold) {
        state.right = true;
    }
}

//Code for using gyro
void turnSensorSetup()
{
    Wire.begin();
    imu.init();
    imu.enableDefault();
    imu.configureForTurnSensing();

    lcd.clear();
    lcd.print(F("Gyro cal"));

    // Turn on the yellow LED in case the LCD is not available.
    ledYellow(1);

    // Delay to give the user time to remove their finger.
    delay(500);

    // Calibrate the gyro.
    int32_t total = 0;
    for (uint16_t i = 0; i < 1024; i++)
    {
        // Wait for new data to be available, then read it.
        while (!imu.gyroDataReady()) {}
        imu.readGyro();

        // Add the Z axis reading to the total.
        total += imu.g.z;
    }
    ledYellow(0);
    gyroOffset = total / 1024;

    // Display the angle (in degrees from -180 to 180) until the
    // user presses A.
    lcd.clear();
}


void turnSensorReset()
{
    gyroLastUpdate = micros();
    turnAngle = 0;
}

void turnSensorUpdate()
{
    imu.readGyro();
    turnRate = imu.g.z - gyroOffset;
    uint16_t m = micros();
    uint16_t dt = m - gyroLastUpdate;
    gyroLastUpdate = m;
    int32_t d = (int32_t)turnRate * dt;
    turnAngle += (int64_t)d * 14680064 / 17578125;
}

