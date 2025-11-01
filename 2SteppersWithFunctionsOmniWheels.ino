#include <AccelStepper.h>
#include <Servo.h>

// Use AccelStepper in DRIVER mode (A4988)
#define MotorInterfaceType 1

// Assign pins for each stepper motor (front right, front left, rear left, rear right)
#define stepPinFR 2 //Front Right motor step pin
#define dirPinFR 3  //Front Right motor direction pin

#define stepPinFL 4 //Front Left motor step pin
#define dirPinFL 5  // Front left motor direction pin

#define stepPinRL 6 //Rear left motor step pin
#define dirPinRL 7  //Rear left motor direction pin

#define stepPinRR 8 //Rear right motor step pin
#define dirPinRR 9  //Rear right motor direction pin

// Define Stepper drive motors
AccelStepper FrontRight(MotorInterfaceType, stepPinFR, dirPinFR);   //Front right motor
AccelStepper FrontLeft(MotorInterfaceType, stepPinFL, dirPinFL);    // Front left motor
AccelStepper RearLeft(MotorInterfaceType, stepPinRL, dirPinRL);     //Rear left motor
AccelStepper RearRight(MotorInterfaceType, stepPinRR, dirPinRR);    // Rear right motor

// Define Servo motors. Functions for control are at end of code 
// ADD ALL SERVOS TO "HomeAllServos()" FUNCTION + DEFINE "HomeXServo()" FUNCTION FOR ALL NEW SERVOS
// Servo wiring: Orange/White--PWM, Red--5v, Brown/Black--GND
Servo FlagServo;          // Servo that pushes duck flag off robot
    #define FlagServoPin 10 //Define pin for FlagServo as pin 10


//Define linear actuator/Solenoid positive and negative pins
#define Actuator7Pos 31 // Actuator that presses 7 key positive wire
#define Actuator7Neg 33 // Actuator that presses 7 key negative wire

#define Actuator3Pos 35 // Actuator that presses 7 key positive wire
#define Actuator3Neg 37 // Actuator that presses 7 key negative wire

#define Actuator8Pos 39 // Actuator that presses 7 key positive wire
#define Actuator8Neg 41 // Actuator that presses 7 key negative wire

float InchesPerStep = 0.06; // Calibrate this for your wheels
int StepsPerInch = 16;
int Turn90 = 293;           // Calibrate for your robot

void setup() // Defines steppers' speed + acceleration and servo pins
{
    int Speed = 1000;   // Steps per second,  200 steps per rotation
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

// // Linear actuator/Soleniod for keypad
// void EnterCombination()
// {
//     digitalWrite(Actuator7Pos, HIGH); // Presses the 7 key on the keypad
//     digitalWrite(Actuator7Neg, LOW);
//     delay(1000);
//     digitalWrite(Actuator7Pos, LOW);
//     digitalWrite(Actuator7Neg, HIGH);
//     delay(1000);

//     digitalWrite(Actuator3Pos, HIGH); // Presses the 3 key on the keypad
//     digitalWrite(Actuator3Neg, LOW);
//     delay(1000);
//     digitalWrite(Actuator3Pos, LOW);
//     digitalWrite(Actuator3Neg, HIGH);
//     delay(1000);

//     digitalWrite(Actuator7Pos, HIGH); // Presses the 7 key on the keypad
//     digitalWrite(Actuator7Neg, LOW);
//     delay(1000);
//     digitalWrite(Actuator7Pos, LOW);
//     digitalWrite(Actuator7Neg, HIGH);
//     delay(1000);

//     digitalWrite(Actuator3Pos, HIGH); // Presses the 3 key on the keypad
//     digitalWrite(Actuator3Neg, LOW);
//     delay(1000);
//     digitalWrite(Actuator3Pos, LOW);
//     digitalWrite(Actuator3Neg, HIGH);
//     delay(1000);

//     digitalWrite(Actuator8Pos, HIGH); // Presses the 8 key on the keypad
//     digitalWrite(Actuator8Neg, LOW);
//     delay(1000);
//     digitalWrite(Actuator8Pos, LOW);
//     digitalWrite(Actuator8Neg, HIGH);
//     delay(1000);
// }