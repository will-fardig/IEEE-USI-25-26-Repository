#include <AccelStepper.h>
#include <Servo.h>

// Use AccelStepper in DRIVER mode (L298N)
#define MotorInterfaceType 4

// Define Stepper drive motors
AccelStepper FrontRight(MotorInterfaceType, 30,32,34,36);   //Yellow (in1), orange(in2), red(in3), brown(in4) respectively
AccelStepper FrontLeft(MotorInterfaceType, 31,33,35,37);    //Blue (in1), purple(in2), gray(in3), white(in4) respectively 
AccelStepper RearLeft(MotorInterfaceType, 40,42,44,46);     // (in1), (in2), (in3), (in4) respectively
AccelStepper RearRight(MotorInterfaceType, 41,43,45,47);    // (in1), (in2), (in3), (in4) respectively

// Define Servo motors. Functions for control are at end of code 
// ADD ALL SERVOS TO "HomeAllServos()" FUNCTION + DEFINE "HomeXServo()" FUNCTION FOR ALL NEW SERVOS
// Servo wiring: Orange/White--PWM, Red--5v, Brown/Black--GND

Servo FlagServo;          // Servo that pushes duck flag off robot
#define FlagServoPin 10 //Define pin for FlagServo as pin 10


//Define linear Solenoid positive and negative pins
#define Solenoid7Pos 31 // Solenoid that presses 7 key positive wire
#define Solenoid7Neg 33 // Solenoid that presses 7 key negative wire

#define Solenoid3Pos 35 // Solenoid that presses 7 key positive wire
#define Solenoid3Neg 37 // Solenoid that presses 7 key negative wire

#define Solenoid8Pos 39 // Solenoid that presses 7 key positive wire
#define Solenoid8Neg 41 // Solenoid that presses 7 key negative wire

float InchesPerStep = 0.06; // Inches per step with 3.8 inch wheels
int StepsPerInch = 16;
int Turn90 = 293;           

void setup() // Defines steppers' speed + acceleration and servo pins
{
    int Speed = 250;   // Steps per second,  200 steps per rotation
    int Accel = 1000;   // Acceleration 

    FrontRight.setMaxSpeed(Speed);  // Sets Front right motor speed
    FrontLeft.setMaxSpeed(Speed);   // Sets Front left motor speed
    RearLeft.setMaxSpeed(Speed);    // Sets Rear right motor speed
    RearRight.setMaxSpeed(Speed);   // Sets Rear left motor speed

    FrontRight.setAcceleration(Accel);  // Sets Front right motor acceleration
    FrontLeft.setAcceleration(Accel);   // Sets Front left motor acceleration
    RearLeft.setAcceleration(Accel);    // Sets Rear right motor acceleration
    RearRight.setAcceleration(Accel);   // Sets Rear left motor acceleration

    FlagServo.attach(FlagServoPin); // Connects Flag servo to PWM pin 10. (Orange wire on Servo)
}

void loop() 
{
    //Knock duck flag off robot
    MoveFlagServo(180); //Move Flag servo to 90 degrees
    delay(1000);
    HomeFlagServo();    //Move Flag servo to 0 degrees
    delay(1000);
    
    // Move forward
    Forward(12*StepsPerInch);
    delay(1000);

    // Move backward
    Reverse(12*StepsPerInch);
    delay(1000);

    // Strafe left
    StrafeLeft(12*StepsPerInch);
    delay(1000);

    // Strafe right
    StrafeRight(12*StepsPerInch);
    delay(1000);

    // Rotate left
    RotateLeft(Turn90);
    delay(1000);

    // Rotate right
    RotateRight(Turn90);
    delay(1000);

    HomeAllServos();
}

// Stepper functions
void Forward(int steps) // Function to move forward
{
    FrontRight.moveTo(FrontRight.currentPosition() + steps);
    FrontLeft.moveTo(FrontLeft.currentPosition() + steps);
    RearLeft.moveTo(RearLeft.currentPosition() + steps);
    RearRight.moveTo(RearRight.currentPosition() + steps);

    while (FrontRight.distanceToGo() != 0 ||
           FrontLeft.distanceToGo() != 0 ||
           RearLeft.distanceToGo() != 0 ||
           RearRight.distanceToGo() != 0) {
        FrontRight.run();
        FrontLeft.run();
        RearLeft.run();
        RearRight.run();
    }
}

void Reverse(int steps) // Function to move backward
{
    FrontRight.moveTo(FrontRight.currentPosition() - steps);
    FrontLeft.moveTo(FrontLeft.currentPosition() - steps);
    RearLeft.moveTo(RearLeft.currentPosition() - steps);
    RearRight.moveTo(RearRight.currentPosition() - steps);

    while (FrontRight.distanceToGo() != 0 ||
           FrontLeft.distanceToGo() != 0 ||
           RearLeft.distanceToGo() != 0 ||
           RearRight.distanceToGo() != 0) {
        FrontRight.run();
        FrontLeft.run();
        RearLeft.run();
        RearRight.run();
    }
}

void StrafeLeft(int steps) // Function to strafe left
{
    FrontRight.moveTo(FrontRight.currentPosition() + steps);
    FrontLeft.moveTo(FrontLeft.currentPosition() - steps);
    RearLeft.moveTo(RearLeft.currentPosition() + steps);
    RearRight.moveTo(RearRight.currentPosition() - steps);

    while (FrontRight.distanceToGo() != 0 ||
           FrontLeft.distanceToGo() != 0 ||
           RearLeft.distanceToGo() != 0 ||
           RearRight.distanceToGo() != 0) {
        FrontRight.run();
        FrontLeft.run();
        RearLeft.run();
        RearRight.run();
    }
}

void StrafeRight(int steps) // Function to strafe right
{
    FrontRight.moveTo(FrontRight.currentPosition() - steps);
    FrontLeft.moveTo(FrontLeft.currentPosition() + steps);
    RearLeft.moveTo(RearLeft.currentPosition() - steps);
    RearRight.moveTo(RearRight.currentPosition() + steps);

    while (FrontRight.distanceToGo() != 0 ||
           FrontLeft.distanceToGo() != 0 ||
           RearLeft.distanceToGo() != 0 ||
           RearRight.distanceToGo() != 0) {
        FrontRight.run();
        FrontLeft.run();
        RearLeft.run();
        RearRight.run();
    }
}

void RotateLeft(int steps) // Function to rotate left
{
    FrontRight.moveTo(FrontRight.currentPosition() + steps);
    FrontLeft.moveTo(FrontLeft.currentPosition() - steps);
    RearLeft.moveTo(RearLeft.currentPosition() - steps);
    RearRight.moveTo(RearRight.currentPosition() + steps);

    while (FrontRight.distanceToGo() != 0 ||
           FrontLeft.distanceToGo() != 0 ||
           RearLeft.distanceToGo() != 0 ||
           RearRight.distanceToGo() != 0) {
        FrontRight.run();
        FrontLeft.run();
        RearLeft.run();
        RearRight.run();
    }
}

void RotateRight(int steps) //Function to rotate right
{
    FrontRight.moveTo(FrontRight.currentPosition() - steps);
    FrontLeft.moveTo(FrontLeft.currentPosition() + steps);
    RearLeft.moveTo(RearLeft.currentPosition() + steps);
    RearRight.moveTo(RearRight.currentPosition() - steps);

    while (FrontRight.distanceToGo() != 0 ||
           FrontLeft.distanceToGo() != 0 ||
           RearLeft.distanceToGo() != 0 ||
           RearRight.distanceToGo() != 0) {
        FrontRight.run();
        FrontLeft.run();
        RearLeft.run();
        RearRight.run();
    }
}

//Servo Functions
void MoveFlagServo(int degrees) // Moves FlagServo X degrees
{
    FlagServo.write(degrees);
}

void HomeFlagServo()            // Sets FlagServo to the 0 degree position
{
    FlagServo.write(0);
}

void HomeAllServos()            // Sets all servos to the 0 degree position
{
    FlagServo.write(0);
    // Add other servos as needed.
}

// // Solenoid presses for keypad
// void EnterCombination()
// {
//     digitalWrite(Solenoid7Pos, HIGH); // Presses the 7 key on the keypad
//     digitalWrite(Solenoid7Neg, LOW);
//     delay(1000);
//     digitalWrite(Solenoid7Pos, LOW);
//     digitalWrite(Solenoid7Neg, HIGH);
//     delay(1000);

//     digitalWrite(Solenoid3Pos, HIGH); // Presses the 3 key on the keypad
//     digitalWrite(Solenoid3Neg, LOW);
//     delay(1000);
//     digitalWrite(Solenoid3Pos, LOW);
//     digitalWrite(Solenoid3Neg, HIGH);
//     delay(1000);

//     digitalWrite(Solenoid7Pos, HIGH); // Presses the 7 key on the keypad
//     digitalWrite(Solenoid7Neg, LOW);
//     delay(1000);
//     digitalWrite(Solenoid7Pos, LOW);
//     digitalWrite(Solenoid7Neg, HIGH);
//     delay(1000);

//     digitalWrite(Solenoid3Pos, HIGH); // Presses the 3 key on the keypad
//     digitalWrite(Solenoid3Neg, LOW);
//     delay(1000);
//     digitalWrite(Solenoid3Pos, LOW);
//     digitalWrite(Solenoid3Neg, HIGH);
//     delay(1000);

//     digitalWrite(Solenoid8Pos, HIGH); // Presses the 8 key on the keypad
//     digitalWrite(Solenoid8Neg, LOW);
//     delay(1000);
//     digitalWrite(Solenoid8Pos, LOW);
//     digitalWrite(Solenoid8Neg, HIGH);
//     delay(1000);
// }