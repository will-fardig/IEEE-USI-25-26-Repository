#include <AccelStepper.h>
#include <Servo.h>
#include <Adafruit_MCP23X17.h>

// Use AccelStepper in DRIVER mode (L298N)
#define MotorInterfaceType 4

AccelStepper FrontRight(MotorInterfaceType, 41,43,45,47); // Blue (in1), Red (in2), Green (in3), Black (in4) respectively
AccelStepper RearRight(MotorInterfaceType, 40,42,44,46); // Blue (in1), Red (in2), Green (in3), Black (in4) respectively

AccelStepper FrontLeft(MotorInterfaceType, 31,33,35,37); // Blue (in1), Red (in2), Green (in3), Black (in4) respectively 
AccelStepper RearLeft(MotorInterfaceType, 30,32,34,36); // Blue (in1), Red (in2), Green (in3), Black (in4) respectively

AccelStepper CrankStepper(MotorInterfaceType, 22,23,24,25); // (in1), (in2), (in3), (in4) respectively

// Servo wiring: Orange/White--PWM, Red--5V, Brown/Black--GND
Servo FlagServo; // Definition of flag servo objects
#define FlagServoPin 10 //Definition of variable to store flag servo pin number

// Solenoid pins
#define Solenoid7 0 // presses 7 key
#define Solenoid3 1 // presses 3 key
#define SolenoidP 2 // presses 8 key
#define Solenoid8 3 // presses # key
Adafruit_MCP23X17 mcp;

// Movement and distance defs
const float inchesPerStep = 0.06; // Inches per step with 3.8 inch wheels; unused at present
const int stepsPerInch = 16;
const int stepsPerInchStrafe =  20;
const int turn90 = 293;

void setup() // Definitions of max speed/acceleration, attatchment of servo pins to servor objects, mcp pin setting for solenoids/solenoid driver and forcing solenoid pins to low.
{
    int speed = 200; // Steps per second, 200 steps per rotation
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
    // DropFlag(0); // move servo to 0 degree position (veritcal, arm pointing up)
    // delay(1000);
    // HomeFlagServo();
    // delay(2000);

    Forward(28);
    delay(1000);
    TripleButtonWombo();
    delay(1000);
    Reverse(16);
    delay(1000);

    StrafeRight(23.5);
    delay(2000);

    EnterCombination();

    // TurnCrank();

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

void EnterCombination() 
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

void TurnCrank()
{
    CrankStepper.move(400);

    while (CrankStepper.distanceToGo() != 0) 
    {
        CrankStepper.run();
    }
}

// SERVO FUNCTIONS
void DropFlag(int degrees) // Moves servo n degrees
{
    FlagServo.write(degrees);
}

void HomeFlagServo() // Homes flag servo 180 degrees (arm pointing down)
{
    FlagServo.write(180);
}

void HomeAllServos() // Homes all servos (0 degrees) add others here
{
   HomeFlagServo();
}
