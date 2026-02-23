#include <AccelStepper.h>
#include <Servo.h>
#include <Adafruit_MCP23X17.h>

// Use AccelStepper in DRIVER mode (L298N)
#define MotorInterfaceType 4

AccelStepper FrontRight(MotorInterfaceType, 31,33,35,37); // Blue (in1), red(in2), green(in3), black(in4) respectively
AccelStepper RearRight(MotorInterfaceType, 41,43,45,47); // Blue (in1), red(in2), green(in3), black(in4) respectively

AccelStepper FrontLeft(MotorInterfaceType, 36,34,32,30); // Blue (in1), red(in2), green(in3), black(in4) respectively 
AccelStepper RearLeft(MotorInterfaceType, 46,44,42,40); // Blue (in1), red(in2), green(in3), black(in4) respectively

AccelStepper CrankStepper(MotorInterfaceType, 22,23,24,25); // (in1), (in2), (in3), (in4) respectively

// Servo wiring: Orange/White--PWM, Red--5V, Brown/Black--GND
Servo FlagServo;
#define FlagServoPin 10

// Solenoid pins
#define Solenoid7 0 // presses 7 key
#define Solenoid3 1 // presses 3 key
#define Solenoid8 2 // presses 8 key
#define SolenoidP 3 // presses # key
Adafruit_MCP23X17 mcp;

// config photoresistor

#define PHOTORESISTOR_PIN A0
const int LIGHT_THRESHOLD = 50;

// light vars
int currentLight = 0;
int previousLight = 0;

// Movement and distance defs
const float inchesPerStep = 0.06; // Inches per step with 3.8 inch wheels; unused at present
const int stepsPerInch = 16;
const int stepsPerInchStrafe =  20;
const int turn90 = 293;
const int stdDelay = 1500;

void setup()
{
    int speed = 200; // Steps per second, 200 steps per rotation // tried 100 before
    int strafeSpeed = 125; // hee hee hoo hoo
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

    Serial.begin(9600);
    pinMode(PHOTORESISTOR_PIN, INPUT_PULLUP);

    previousLight = analogRead(PHOTORESISTOR_PIN);

    pinMode(26, INPUT);
    pinMode(27, OUTPUT);
}

void loop() 
{
  // NOT IN USE
    // DropFlag(0);
    // delay(1000);
    // HomeFlagServo();
    // delay(2000);

    photoresistStart();

  // ACTUAL RUN
    Forward(28);
    TripleButtonWombo(); // nail the button three times
    Backward(10);

    StrafeRight(23.5); // lines up horizontally w/ keypad
    // This is where we write to the Pi and make the drone fly
    

    Backward(9.5); // ideally squares us up with the keypad
    KeypadAlignPress(); // ts the keypad func

    Forward(10); // might need to be less to prevent tipping into the crate
    DiagStrafeRearRight(29); // strafing past the crater
    StrafeRight(34); // should line evenly with wall

    Forward(26);
    delay(3000); // IN PLACE OF CRANKING IT
    Backward(10);

    DiagStrafeFrontLeft(29); // strafing past crater, should line with wall
    StrafeLeft(42);
    Backward(24);
    StrafeLeft(22);
    Backward(14); // brings us home in the green (green means good)
    delay(3000);
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// STEPPER FUNCTIONS
void Forward(float inches)
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

    delay(stdDelay);
}

void Backward(float inches)
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

    delay(stdDelay);
}

void StrafeLeft(float inches)
{
    FrontRight.move(inches*stepsPerInchStrafe);
    RearRight.move(-inches*stepsPerInchStrafe);

    FrontLeft.move(-inches*stepsPerInchStrafe);
    RearLeft.move(inches*stepsPerInchStrafe);

    while (FrontRight.distanceToGo() != 0 || FrontLeft.distanceToGo() != 0 || RearLeft.distanceToGo() != 0 || RearRight.distanceToGo() != 0) 
    {
        FrontRight.run();
        FrontLeft.run();
        RearLeft.run();
        RearRight.run();
    }

    delay(stdDelay);
}

void StrafeRight(float inches)
{
    FrontRight.move(-inches*stepsPerInchStrafe);
    RearRight.move(inches*stepsPerInchStrafe);

    FrontLeft.move(inches*stepsPerInchStrafe);
    RearLeft.move(-inches*stepsPerInchStrafe);

    while (FrontRight.distanceToGo() != 0 || FrontLeft.distanceToGo() != 0 || RearLeft.distanceToGo() != 0 || RearRight.distanceToGo() != 0) 
    {
        FrontRight.run();
        FrontLeft.run();
        RearLeft.run();
        RearRight.run();
    }

    delay(stdDelay);
}

void DiagStrafeRearRight(float inches)
{
    FrontRight.move(-inches*stepsPerInchStrafe);

    RearLeft.move(-inches*stepsPerInchStrafe);

    while (FrontRight.distanceToGo() != 0 || RearLeft.distanceToGo() != 0) 
    {
        FrontRight.run();
        RearLeft.run();
    }

    delay(stdDelay);
}

void DiagStrafeFrontLeft(float inches)
{
    FrontRight.move(inches*stepsPerInchStrafe);

    RearLeft.move(inches*stepsPerInchStrafe);

    while (FrontRight.distanceToGo() != 0 || RearLeft.distanceToGo() != 0) 
    {
        FrontRight.run();
        RearLeft.run();
    }

    delay(stdDelay);
}

void RotateLeft(float degrees) // Function to rotate left
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

    delay(stdDelay);
}

void RotateRight(float degrees) //Function to rotate right
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

    delay(stdDelay);
}

// SPECIFIC FUNCTIONS

void TripleButtonWombo() // Antenna One
{
    Forward(1);
    Backward(1);

    Forward(1);
    Backward(1);

    Forward(1);
    Backward(1); // should land us where we started...
}

void KeypadAlignPress() 
{
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

void CrankHog()
{
    CrankStepper.move(400);

    while (CrankStepper.distanceToGo() != 0) 
    {
        CrankStepper.run();
    }

    delay(stdDelay);
}

// SERVO FUNCTIONS
void MoveFlagServo(float degrees) // Moves servo n degrees
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

// TEST FUNCTIONS
void ForwardN(float inches)
{
    FrontRight.setSpeed(300);
    RearRight.setSpeed(300);

    FrontLeft.setSpeed(300);
    RearLeft.setSpeed(300);

    FrontRight.move(inches*stepsPerInch);
    RearRight.move(inches*stepsPerInch);

    FrontLeft.move(inches*stepsPerInch);
    RearLeft.move(inches*stepsPerInch);

    while (FrontRight.distanceToGo() != 0 || FrontLeft.distanceToGo() != 0 || RearLeft.distanceToGo() != 0 || RearRight.distanceToGo() != 0) 
    {
        FrontRight.runSpeed();
        FrontLeft.runSpeed();
        RearLeft.runSpeed();
        RearRight.runSpeed();
    }
}

void BackwardN(float inches)
{
    FrontRight.setSpeed(600);
    RearRight.setSpeed(600);

    FrontLeft.setSpeed(600);
    RearLeft.setSpeed(600);

    FrontRight.move(-inches*stepsPerInch);
    RearRight.move(-inches*stepsPerInch);

    FrontLeft.move(-inches*stepsPerInch);
    RearLeft.move(-inches*stepsPerInch);

    while (FrontRight.distanceToGo() != 0 || FrontLeft.distanceToGo() != 0 || RearLeft.distanceToGo() != 0 || RearRight.distanceToGo() != 0) 
    {
        FrontRight.runSpeed();
        FrontLeft.runSpeed();
        RearLeft.runSpeed();
        RearRight.runSpeed();
    }
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
