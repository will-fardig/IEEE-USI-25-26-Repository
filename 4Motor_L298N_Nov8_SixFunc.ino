#include <AccelStepper.h>
#include <Servo.h>

// Use AccelStepper in DRIVER mode (L298N)
#define MotorInterfaceType 4

AccelStepper FrontRight(MotorInterfaceType, 41,43,45,47); // Yellow (in1), orange(in2), red(in3), brown(in4) respectively
AccelStepper RearRight(MotorInterfaceType, 40,42,44,46); // Yellow (in1), orange(in2), red(in3), brown(in4) respectively

AccelStepper FrontLeft(MotorInterfaceType, 31,33,35,37); // White (in1), grey(in2), purple(in3), blue(in4) respectively 
AccelStepper RearLeft(MotorInterfaceType, 30,32,34,36); // White (in1), grey(in2), purple(in3), blue(in4) respectively 

// Servo wiring: Orange/White--PWM, Red--5V, Brown/Black--GND
Servo FlagServo;
#define FlagServoPin 10

// Solenoid pins boy
#define Solenoid7 2 // presses 7 key
#define Solenoid3 3 // presses 3 key
#define Solenoid8 4 // presses 8 key
#define SolenoidP 5 // presses # key

// Movement and distance defs
float inchesPerStep = 0.06; // Inches per step with 3.8 inch wheels; unused at present
int stepsPerInch = 16;
int turn90 = 293;

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
}

void loop() 
{
    // MoveFlagServo(180);
    // delay(1000);
    // HomeFlagServo();
    // delay(1000);
    
    // Forward(12);
    // delay(1000);

    // Reverse(12);
    // delay(1000);

    // StrafeLeft(12);
    // delay(1000);

    // StrafeRight(12);
    // delay(1000);

    RotateLeft(turn90);
    delay(1000);

    RotateRight(turn90);
    delay(1000);

    // HomeAllServos();
}

// STEPPER FUNCTIONS
void Forward(int inches)
{
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
}

void Reverse(int inches)
{
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
}

void StrafeLeft(int inches)
{
    FrontRight.move(inches*stepsPerInch);
    RearRight.move(inches*stepsPerInch);

    FrontLeft.move(-inches*stepsPerInch);
    RearLeft.move(-inches*stepsPerInch);

    while (FrontRight.distanceToGo() != 0 || FrontLeft.distanceToGo() != 0 || RearLeft.distanceToGo() != 0 || RearRight.distanceToGo() != 0) {
        FrontRight.run();
        FrontLeft.run();
        RearLeft.run();
        RearRight.run();
    }
}

void StrafeRight(int inches)
{
    FrontRight.move(-inches*stepsPerInch);
    RearRight.move(-inches*stepsPerInch);

    FrontLeft.move(inches*stepsPerInch);
    RearLeft.move(inches*stepsPerInch);

    while (FrontRight.distanceToGo() != 0 || FrontLeft.distanceToGo() != 0 || RearLeft.distanceToGo() != 0 || RearRight.distanceToGo() != 0) {
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

    while (FrontRight.distanceToGo() != 0 || FrontLeft.distanceToGo() != 0) {
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

    while (FrontRight.distanceToGo() != 0 || FrontLeft.distanceToGo() != 0) {
        FrontRight.run();
        FrontLeft.run();
        RearLeft.run();
        RearRight.run();
    }
}

// SERVO FUNCTIONS
void MoveFlagServo(int degrees) // Moves servo n degrees
{
    FlagServo.write(degrees);
}

void HomeFlagServo() // Brings flag servo on home (zero the grease)
{
    FlagServo.write(0);
}

void HomeAllServos() // Brings all servos on home (zero the grease); add others here
{
    FlagServo.write(0);
}

// SOLENOID FUNCTIONS
void EnterCombination()
{
    digitalWrite(Solenoid7, HIGH); // Presses the 7 key on the keypad
    delay(1000);
    digitalWrite(Solenoid7, LOW);
    delay(1000);

    digitalWrite(Solenoid3, HIGH); // Presses the 3 key on the keypad
    delay(1000);
    digitalWrite(Solenoid3, LOW);
    delay(1000);

    digitalWrite(Solenoid7, HIGH); // Presses the 7 key on the keypad
    delay(1000);
    digitalWrite(Solenoid7, LOW);
    delay(1000);

    digitalWrite(Solenoid3, HIGH); // Presses the 3 key on the keypad
    delay(1000);
    digitalWrite(Solenoid3, LOW);
    delay(1000);

    digitalWrite(Solenoid8, HIGH); // Presses the 8 key on the keypad
    delay(1000);
    digitalWrite(Solenoid8, LOW);
    delay(1000);

    digitalWrite(SolenoidP, HIGH); // Presses the # key on the keypad
    delay(1000);
    digitalWrite(SolenoidP, LOW);
    delay(1000);
}
