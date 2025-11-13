#include <AccelStepper.h>
#include <Servo.h>
#include <Adafruit_MCP23X17.h>

// Use AccelStepper in DRIVER mode (L298N)
#define MotorInterfaceType 4

AccelStepper FrontRight(MotorInterfaceType, 41,43,45,47); // Yellow (in1), orange(in2), red(in3), brown(in4) respectively
AccelStepper RearRight(MotorInterfaceType, 40,42,44,46); // Yellow (in1), orange(in2), red(in3), brown(in4) respectively

AccelStepper FrontLeft(MotorInterfaceType, 31,33,35,37); // White (in1), grey(in2), purple(in3), blue(in4) respectively 
AccelStepper RearLeft(MotorInterfaceType, 30,32,34,36); // White (in1), grey(in2), purple(in3), blue(in4) respectively 

// Servo wiring: Orange/White--PWM, Red--5V, Brown/Black--GND
Servo FlagServo;
#define FlagServoPin 10

// Solenoid pins
#define Solenoid7 2 // presses 7 key
#define Solenoid3 3 // presses 3 key
#define Solenoid8 4 // presses 8 key
#define SolenoidP 5 // presses # key
Adafruit_MCP23X17 mcp;

// Movement and distance defs
const float inchesPerStep = 0.06; // Inches per step with 3.8 inch wheels; unused at present
const int stepsPerInch = 16;
const int stepsPerInchStrafe =  20;
const int turn90 = 293;

void setup()
{
    int speed = 150; // Steps per second, 200 steps per rotation
    int accel = 1000; // required for thingus to move at all

    FrontRight.setMaxSpeed(speed);
    RearRight.setMaxSpeed(speed);

    FrontLeft.setMaxSpeed(speed);
    RearLeft.setMaxSpeed(speed);

    FrontRight.setAcceleration(accel);
    RearRight.setAcceleration(accel);

    FrontLeft.setAcceleration(accel);
    RearLeft.setAcceleration(accel);

    FlagServo.attach(FlagServoPin); // Orange wire on servo

    mcp.begin_I2C();
    // Set pins as outputs
    mcp.pinMode(Solenoid7, OUTPUT);
    mcp.pinMode(Solenoid3, OUTPUT);
    mcp.pinMode(Solenoid8, OUTPUT);
    mcp.pinMode(SolenoidP, OUTPUT);

    // Initially off (LOW)
    mcp.digitalWrite(Solenoid7, LOW);
    mcp.digitalWrite(Solenoid3, LOW);
    mcp.digitalWrite(Solenoid8, LOW);
    mcp.digitalWrite(SolenoidP, LOW);
}

void loop() 
{
    // MoveFlagServo(180);
    // delay(1000);
    // HomeFlagServo();
    // delay(1000);

    Forward(28);
    delay(1000);
    TripleButtonWombo();
    delay(1000);
    Reverse(16);
    delay(1000);

    StrafeRight(24);
    delay(2000);

    // enterCombination();

    // HomeAllServos();
}

// STEPPER FUNCTIONS
void Forward(int inches)
{
    FrontRight.move(inches*stepsPerInch);
    RearRight.move(inches*stepsPerInch);

    FrontLeft.move(inches*stepsPerInch);
    RearLeft.move(inches*stepsPerInch);

    while (FrontRight.distanceToGo() != 0 || FrontLeft.distanceToGo() != 0 || RearLeft.distanceToGo() != 0 || RearRight.distanceToGo() != 0) 
    {
        FrontRight.run();
        FrontLeft.run();
        RearLeft.run();
        RearRight.run();
    }
}

void Reverse(int inches)
{
    FrontRight.move(-inches*stepsPerInch);
    RearRight.move(-inches*stepsPerInch);

    FrontLeft.move(-inches*stepsPerInch);
    RearLeft.move(-inches*stepsPerInch);

    while (FrontRight.distanceToGo() != 0 || FrontLeft.distanceToGo() != 0 || RearLeft.distanceToGo() != 0 || RearRight.distanceToGo() != 0) 
    {
        FrontRight.run();
        FrontLeft.run();
        RearLeft.run();
        RearRight.run();
    }
}

void StrafeLeft(int inches)
{
    FrontRight.move(inches*stepsPerInchStrafe);
    RearRight.move(inches*stepsPerInchStrafe);

    FrontLeft.move(-inches*stepsPerInchStrafe);
    RearLeft.move(-inches*stepsPerInchStrafe);

    while (FrontRight.distanceToGo() != 0 || FrontLeft.distanceToGo() != 0 || RearLeft.distanceToGo() != 0 || RearRight.distanceToGo() != 0) 
    {
        FrontRight.run();
        FrontLeft.run();
        RearLeft.run();
        RearRight.run();
    }
}

void StrafeRight(int inches)
{
    float strafeAdjust = (stepsPerInch * (10/9));
    FrontRight.move(-inches*stepsPerInchStrafe);
    RearRight.move(-inches*stepsPerInchStrafe);

    FrontLeft.move(inches*stepsPerInchStrafe);
    RearLeft.move(inches*stepsPerInchStrafe);

    while (FrontRight.distanceToGo() != 0 || FrontLeft.distanceToGo() != 0 || RearLeft.distanceToGo() != 0 || RearRight.distanceToGo() != 0) 
    {
        FrontRight.run();
        FrontLeft.run();
        RearLeft.run();
        RearRight.run();
    }
}

void RotateLeft(int degrees) // Function to rotate left
{
    FrontRight.move(degrees);

    FrontLeft.move(-degrees);

    while (FrontRight.distanceToGo() != 0 || FrontLeft.distanceToGo() != 0) 
    {
        FrontRight.run();
        FrontLeft.run();
        RearLeft.run();
        RearRight.run();
    }

}

void RotateRight(int degrees) //Function to rotate right
{
    FrontRight.move(-degrees);

    FrontLeft.move(degrees);

    while (FrontRight.distanceToGo() != 0 || FrontLeft.distanceToGo() != 0) 
    {
        FrontRight.run();
        FrontLeft.run();
        RearLeft.run();
        RearRight.run();
    }
}

// SPECIFIC FUNCTIONS

void TripleButtonWombo() // Antenna One
{
    Forward(1);
    delay(500);
    Reverse(1);
    delay(500);

    Forward(1);
    delay(500);
    Reverse(1);
    delay(500);

    Forward(1);
    delay(500);
    Reverse(1); // should land us where we started...
    delay(500);
}

void enterCombination() 
{
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
}

// SERVO FUNCTIONS
void MoveFlagServo(int degrees) // Moves servo n degrees
{
    FlagServo.write(degrees);
}

void HomeFlagServo() // Homes flag servo (0 degrees)
{
    FlagServo.write(0);
}

void HomeAllServos() // Homes all servos (0 degrees) add others here
{
    FlagServo.write(0);
}