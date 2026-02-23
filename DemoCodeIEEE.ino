#include <AccelStepper.h>
#include <Servo.h>
#include <Adafruit_MCP23X17.h>

///////////////////////////////////////////////////////////////
// MOTOR INTERFACE AND ACCELSTEPPER
///////////////////////////////////////////////////////////////

#define MotorInterfaceType 4 // Sets AccelStepper to driver mode (L298N)

AccelStepper FrontRight(MotorInterfaceType, 31,33,35,37); // Blue (in1), red(in2), green(in3), black(in4) respectively
AccelStepper RearRight(MotorInterfaceType, 41,43,45,47); // Blue (in1), red(in2), green(in3), black(in4) respectively
AccelStepper FrontLeft(MotorInterfaceType, 36,34,32,30); // Blue (in1), red(in2), green(in3), black(in4) respectively 
AccelStepper RearLeft(MotorInterfaceType, 46,44,42,40); // Blue (in1), red(in2), green(in3), black(in4) respectively
AccelStepper CrankStepper(MotorInterfaceType, 22,23,24,25);

///////////////////////////////////////////////////////////////
// FLAG SERVO AND SOLENOIDS
///////////////////////////////////////////////////////////////

Servo FlagServo; // Orange/White: PWM, Red: 5V, and Brown/Black: GND
#define FlagServoPin 10
#define Solenoid7 0 // presses 7 key
#define Solenoid3 1 // presses 3 key
#define Solenoid8 2 // presses 8 key
#define SolenoidP 3 // presses # key
Adafruit_MCP23X17 mcp;

///////////////////////////////////////////////////////////////
// PHOTORESISTOR
///////////////////////////////////////////////////////////////

#define PHOTORESISTOR_PIN A0

const int LIGHT_THRESHOLD = 50;
int currentLight = 0;
int previousLight = 0;

///////////////////////////////////////////////////////////////
// MOVEMENT AND POSITIONING CONSTANTS
///////////////////////////////////////////////////////////////

const int stepsPerInch = 16;
const int stepsPerInchStrafe =  20;
const int turn90 = 293;
const int stdDelay = 1500; // Every function includes a delay(stdDelay)

///////////////////////////////////////////////////////////////
// MAIN LOOPS
///////////////////////////////////////////////////////////////

void setup()
{
    int speed = 200; // Steps per second, 200 steps per rotation // tried 100 before
    int strafeSpeed = 125; // Applies to all strafe functions instead of speed
    int accel = 1000; // This value is required for anything to move at all

    // Front Right Motor
    FrontRight.setMaxSpeed(speed);
    FrontRight.setAcceleration(accel);

    // Rear Right Motor
    RearRight.setMaxSpeed(speed);
    RearRight.setAcceleration(accel);

    // Front Left Motor
    FrontLeft.setMaxSpeed(speed);
    FrontLeft.setAcceleration(accel);

    // Rear Left Motor
    RearLeft.setMaxSpeed(speed);
    RearLeft.setAcceleration(accel);

    // Solenoid Init
    mcp.begin_I2C();

    mcp.pinMode(Solenoid7, OUTPUT); // Sets all as output
    mcp.pinMode(Solenoid3, OUTPUT);
    mcp.pinMode(Solenoid8, OUTPUT);
    mcp.pinMode(SolenoidP, OUTPUT);

    mcp.digitalWrite(Solenoid7, LOW); // Sets all as initially off
    mcp.digitalWrite(Solenoid3, LOW);
    mcp.digitalWrite(Solenoid8, LOW);
    mcp.digitalWrite(SolenoidP, LOW);

    FlagServo.attach(FlagServoPin); // Orange wire on servo

    // Photoresistor Init
    Serial.begin(9600);
    pinMode(PHOTORESISTOR_PIN, INPUT_PULLUP);
    previousLight = analogRead(PHOTORESISTOR_PIN);

    // Presently unused pins for communicating with the Raspberry Pi
    pinMode(26, INPUT);
    pinMode(27, OUTPUT);
}

void loop() 
{
    photoresistStart(); // Auto-starts using a white LED start bar: 15 points

    // MOVE TO BUTTON
    Forward(28); // Robot leaves starting area: 10 points
    MoveFlagServo(180); // Plant flag outside starting area: 10 points
    TripleButtonWombo(); // Nails the button three times: 15 points (antenna one of four)
    Backward(10);

    // MOVE TO KEYPAD
    StrafeRight(23.5); // Line up horizontally with the keypad
    
    ///////////////////////////////////////////////////////////////////////
    /////// This is where we write to the Pi and make the drone fly ///////
    // digitalWrite(27, HIGH);
    // Pi breaks out of loop thanks to receiving HIGH from Arduino
    // Pi launches drone: up 18 inches, right 18 inches, left 18 inches: 30 points
    // Pi does its best to make the drone land right back on the robot: 50 points
    // Pi writes HIGH to Arduino which breaks out of this func's loop
    ///////////////////////////////////////////////////////////////////////

    Backward(9.5); // Get right up on the keypad, ready to push keys
    KeypadAlignPress(); // Enters the code 73738#: 15 points (antenna two of four)

    // MOVE TO CRANK
    Forward(10); // Away from the keypad. May need to shorten due to falling in the crater
    DiagStrafeRearRight(29); // Strafes past the crater with very little clearance
    StrafeRight(34); // Ideally lines us square with the wall, may have some excess movement
    Forward(26); // Reaches the crank antenna, putting us exactly in position
    // CrankHog(); // Rotates the crank many degrees: 15 points (antenna three of four)
    Backward(10); // Takes us away from the crank

    // THIS IS MAJOR TOM TO GROUND CONTROL
    DiagStrafeFrontLeft(29); // Strafes past the crater and lines directly in with the wall
    StrafeLeft(42); // Takes us all the way back to the area with the first two antennas
    Backward(24); // Pulls back to middle of the area (make sure we don't run into the flag)
    StrafeLeft(22); // Lines us up with the wall, may have some excess movement
    Backward(14); // Brings us right into the green, ending round in starting area (green means good): 15 points
    delay(3000); // Some kind of aura farming while we all twist and shout or whatever

    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //
    // RECEIPT FOR ALL THE COOL STUFF WE DID:
    // Robot leaves the starting area....................................10 points
    // Robot plants flag outside starting area...........................10 points
    // Antenna turned on (x3).....................................15*3 = 45 points
    // Ends the round in the starting area...............................15 points
    // Launch of the UAV (including 15 inch movement V/H)................30 points
    // Retrieval of the UAV on the robot.................................50 points
    // Robot autostarts using a white LED bar............................15 points
    // Will's awesome t-shirt being entered in the student design comp...15 points
    //
    // TOTAL INTENDED EARNED POINTS.....................................190 points
    // POSSIBLE COMPETITION POINTS......................................370 points
    // POSSIBLE POINTS W/ EXTRA NONSENSE................................430 points
    //
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
}

///////////////////////////////////////////////////////////////
// FUNCTION WAREHOUSE
///////////////////////////////////////////////////////////////

void Forward(float inches) {
    FrontRight.move(inches*stepsPerInch);
    RearRight.move(inches*stepsPerInch);

    FrontLeft.move(inches*stepsPerInch);
    RearLeft.move(inches*stepsPerInch);

    while (FrontRight.distanceToGo() != 0 || FrontLeft.distanceToGo() != 0 || RearLeft.distanceToGo() != 0 || RearRight.distanceToGo() != 0) {
        FrontRight.run();
        FrontLeft.run();
        RearLeft.run();
        RearRight.run();
    }

    delay(stdDelay);
}

void Backward(float inches) {
    FrontRight.move(-inches*stepsPerInch);
    RearRight.move(-inches*stepsPerInch);

    FrontLeft.move(-inches*stepsPerInch);
    RearLeft.move(-inches*stepsPerInch);

    while (FrontRight.distanceToGo() != 0 || FrontLeft.distanceToGo() != 0 || RearLeft.distanceToGo() != 0 || RearRight.distanceToGo() != 0) {
        FrontRight.run();
        FrontLeft.run();
        RearLeft.run();
        RearRight.run();
    }

    delay(stdDelay);
}

void StrafeLeft(float inches) {
    FrontRight.move(inches*stepsPerInchStrafe);
    RearRight.move(-inches*stepsPerInchStrafe);

    FrontLeft.move(-inches*stepsPerInchStrafe);
    RearLeft.move(inches*stepsPerInchStrafe);

    while (FrontRight.distanceToGo() != 0 || FrontLeft.distanceToGo() != 0 || RearLeft.distanceToGo() != 0 || RearRight.distanceToGo() != 0) {
        FrontRight.run();
        FrontLeft.run();
        RearLeft.run();
        RearRight.run();
    }

    delay(stdDelay);
}

void StrafeRight(float inches) {
    FrontRight.move(-inches*stepsPerInchStrafe);
    RearRight.move(inches*stepsPerInchStrafe);

    FrontLeft.move(inches*stepsPerInchStrafe);
    RearLeft.move(-inches*stepsPerInchStrafe);

    while (FrontRight.distanceToGo() != 0 || FrontLeft.distanceToGo() != 0 || RearLeft.distanceToGo() != 0 || RearRight.distanceToGo() != 0) {
        FrontRight.run();
        FrontLeft.run();
        RearLeft.run();
        RearRight.run();
    }

    delay(stdDelay);
}

void DiagStrafeRearRight(float inches) {
    FrontRight.move(-inches*stepsPerInchStrafe);

    RearLeft.move(-inches*stepsPerInchStrafe);

    while (FrontRight.distanceToGo() != 0 || RearLeft.distanceToGo() != 0) {
        FrontRight.run();
        RearLeft.run();
    }

    delay(stdDelay);
}

void DiagStrafeFrontLeft(float inches) {
    FrontRight.move(inches*stepsPerInchStrafe);

    RearLeft.move(inches*stepsPerInchStrafe);

    while (FrontRight.distanceToGo() != 0 || RearLeft.distanceToGo() != 0) {
        FrontRight.run();
        RearLeft.run();
    }

    delay(stdDelay);
}

void RotateLeft(float degrees) {
    FrontRight.move(degrees);

    FrontLeft.move(-degrees);

    while (FrontRight.distanceToGo() != 0 || FrontLeft.distanceToGo() != 0) {
        FrontRight.run();
        FrontLeft.run();
        RearLeft.run();
        RearRight.run();
    }

    delay(stdDelay);
}

void RotateRight(float degrees) {
    FrontRight.move(-degrees);

    FrontLeft.move(degrees);

    while (FrontRight.distanceToGo() != 0 || FrontLeft.distanceToGo() != 0) {
        FrontRight.run();
        FrontLeft.run();
        RearLeft.run();
        RearRight.run();
    }

    delay(stdDelay);
}

void TripleButtonWombo() { // The delays are an unfortunate part of life here
    Forward(1);
    Backward(1);

    Forward(1);
    Backward(1);

    Forward(1);
    Backward(1); // Should land us where we started...
}

void KeypadAlignPress() {
    // Square Up
    Backward(1);

    // Fire 7
    mcp.digitalWrite(Solenoid7, HIGH);
    delay(250);
    mcp.digitalWrite(Solenoid7, LOW);
    delay(250);

    // Fire 3
    mcp.digitalWrite(Solenoid3, HIGH);
    delay(250);
    mcp.digitalWrite(Solenoid3, LOW);
    delay(250);

    // Fire 7
    mcp.digitalWrite(Solenoid7, HIGH);
    delay(250);
    mcp.digitalWrite(Solenoid7, LOW);
    delay(250);

    // Fire 3
    mcp.digitalWrite(Solenoid3, HIGH);
    delay(250);
    mcp.digitalWrite(Solenoid3, LOW);
    delay(250);

    // Fire 8
    mcp.digitalWrite(Solenoid8, HIGH);
    delay(250);
    mcp.digitalWrite(Solenoid8, LOW);
    delay(250);

    // Fire #
    mcp.digitalWrite(SolenoidP, HIGH);
    delay(250);
    mcp.digitalWrite(SolenoidP, LOW);
    delay(250);

    delay(stdDelay);
}

void CrankHog() {
    CrankStepper.move(400); // Degrees I think

    while (CrankStepper.distanceToGo() != 0) {
        CrankStepper.run();
    }

    delay(stdDelay);
}

void MoveFlagServo(float degrees) {
    FlagServo.write(degrees);
    delay(stdDelay);
}

void HomeFlagServo() {
    FlagServo.write(0);
    delay(stdDelay);
}

void photoresistStart() {
    bool Bounds = 0;
    int i = 0;

    Serial.print("-----------------------------");

    while (Bounds == 0 || i <= 10) {
        currentLight = analogRead(PHOTORESISTOR_PIN);
        Serial.print("Current Light: ");
        Serial.println(currentLight);
        int lightChange = currentLight - previousLight;
        if (lightChange >= LIGHT_THRESHOLD) {
            Bounds = 1;
        }
        delay(1000);
        i++;
    }
}
