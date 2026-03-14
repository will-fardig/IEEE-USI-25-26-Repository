#include <AccelStepper.h>
#include <Servo.h>
#include <Adafruit_MCP23X17.h>
#include <stdlib.h>

#include <Arduino.h>
#include <IRremote.hpp>

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
// PI COMMUNICATIONS
///////////////////////////////////////////////////////////////

#define FromPi 26 // Signals received from the Pi to the Arduino
#define ToPi 27  // Signals sent from the Arduino to the Pi

///////////////////////////////////////////////////////////////
// FLAG SERVO, ARM SERVO, AND SOLENOIDS AND THE HEX ONE
///////////////////////////////////////////////////////////////

Servo FlagServo; // Orange/White: PWM, Red: 5V, and Brown/Black: GND
Servo ArmServo;
// Servo HexServo;
#define FlagServoPin 10
#define ArmServoPin 9
// #define HexServoPin 8
#define Solenoid7 0 // presses 7 key
#define Solenoid3 1 // presses 3 key
#define Solenoid8 3 // presses 8 key
#define SolenoidP 2 // presses # key
Adafruit_MCP23X17 mcp;

///////////////////////////////////////////////////////////////
// PHOTORESISTOR AND TRANSMITTER AND UH LEDs
///////////////////////////////////////////////////////////////

#define PHOTORESISTOR_PIN A0

const int LIGHT_THRESHOLD = 50;
int currentLight = 0;
int previousLight = 0;
const int CALIBRATED_LIGHT = 0;

#define SENDER_PIN 11

#define LEDBOARD_PIN 12

///////////////////////////////////////////////////////////////
// MOVEMENT AND POSITIONING CONSTANTS
///////////////////////////////////////////////////////////////

const int stepsPerInch = 16;
const int stepsPerInchStrafe =  20;
const float turnDegree = 2.38888888889; // 860/360;
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

    // Photoresistor Init
    Serial.begin(9600);
    pinMode(PHOTORESISTOR_PIN, INPUT_PULLUP);
    previousLight = analogRead(PHOTORESISTOR_PIN);

    // Presently used pins for communicating with the Raspberry Pi
    pinMode(26, INPUT);
    pinMode(27, OUTPUT);

    // Sets up the IR Sender object
    Serial.begin(115200);
    IrSender.begin(SENDER_PIN);

    // LED Board Init
    pinMode(LEDBOARD_PIN, OUTPUT);
}

void loop() 
{
    PhotoresistStart(); // Auto-starts using a white LED start bar: 15 points

    KeypadButtonFunc(); // Keypad movement, line up on wall, button function, rehome.

    DuckAtKeypad(); // Strafe right, turn and push keypadducky, rerotating, hitting the wall, and rehoming.

    FuckSweeper(); // Move to go counterclockwise around the crater. Rotating to catch ducks in 30 degree increments, pushing them in to endzone.

    StupidChudDuck(); // catches the duck sitting by the button antenna, and squares up all other ducks in endzone, then rehome.

    // PiFunctionality(); // Drone runs and does it stupid chud robot stuff and then lands, and we rehome.
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

void DiagStrafeFrontRight(float inches) {
    FrontLeft.move(inches*stepsPerInchStrafe);

    RearRight.move(inches*stepsPerInchStrafe);

    while (FrontLeft.distanceToGo() != 0 || RearRight.distanceToGo() != 0) {
        FrontLeft.run();
        RearRight.run();
    }

    delay(stdDelay);
}

void RotateLeft(float degreeds) {
    FrontRight.move(degreeds*turnDegree);
    RearRight.move(degreeds*turnDegree);

    FrontLeft.move(-degreeds*turnDegree);
    RearLeft.move(-degreeds*turnDegree);

    while (FrontRight.distanceToGo() != 0 || FrontLeft.distanceToGo() != 0 || RearLeft.distanceToGo() != 0 || RearRight.distanceToGo() != 0) {
        FrontRight.run();
        RearRight.run();
        FrontLeft.run();
        RearLeft.run();
    }

    delay(stdDelay);
}

void RotateRight(float degreeds) {
    FrontRight.move(-degreeds*turnDegree);
    RearRight.move(-degreeds*turnDegree);

    FrontLeft.move(degreeds*turnDegree);
    RearLeft.move(degreeds*turnDegree);

    while (FrontRight.distanceToGo() != 0 || FrontLeft.distanceToGo() != 0 || RearLeft.distanceToGo() != 0 || RearRight.distanceToGo() != 0) {
        FrontRight.run();
        RearRight.run();
        FrontLeft.run();
        RearLeft.run();
    }

    delay(stdDelay);
}

void TripleButtonWombo() { // The delays are an unfortunate part of life here
    Backward(1.25);

    Forward(1.5);
    Backward(1.25);

    Forward(1.5);
    Backward(1.25);

    Forward(1.5);
    Backward(1.25); // Should land us where we started...
}

void KeypadAlignPress() {
    Backward(2);
    // Preliminary pounding
    mcp.digitalWrite(SolenoidP, HIGH);
    delay(250);
    mcp.digitalWrite(SolenoidP, LOW);
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

void MoveFlagServo() {
    FlagServo.write(0);
    delay(stdDelay);
}

void HomeFlagServo() {
    FlagServo.write(180);
    delay(stdDelay);
}

void MoveArmServo() {
    ArmServo.write(80);
    delay(stdDelay);
}

void HomeArmServo() {
    ArmServo.write(180);
    delay(stdDelay);
}

void PhotoresistStart() { // 15 points
    bool Bounds = 0;
    int i = 0;

    Serial.print("-----------------------------");
    Serial.print("Previous Light: ");
    Serial.println(previousLight);
    while (Bounds == 0 && i <= 40) {
        currentLight = analogRead(PHOTORESISTOR_PIN);
        Serial.print("Current Light: ");
        Serial.println(currentLight);
        int lightChange = currentLight - previousLight; // could say
        if (lightChange >= LIGHT_THRESHOLD) {
            Bounds = 1; // ash baby
            Serial.print("WE BROKE OUT BITCH!!!");
            return;
        }
        delay(250);
        i = i + 1;
    }
}

///////////////////////////////////////////////////////////////
// MODULAR INSTRUCTIONS TO EARN POINTS
///////////////////////////////////////////////////////////////

void DuckAtKeypad() { // 5 points
    // Move the keypad duck over to the blue zone and shove him in there
    StrafeRight(10); // 8
    Forward(1.5);
    RotateLeft(30);
    StrafeRight(8);
    RotateLeft(15);
    StrafeRight(10); //new
    RotateLeft(45); //new
    StrafeRight(8); //new

    IRTransmitter();

    delay(10000);

    StrafeLeft(8); //new
    RotateRight(90);
    StrafeLeft(24);
    Backward(17.5);
}

void KeypadButtonFunc() { // 50 points
    // MOVE TO BUTTON
    Forward(16);
    StrafeRight(16);
    RotateRight(180);
    StrafeLeft(16);

    FlagServo.attach(FlagServoPin); // Orange wire on servo

    MoveFlagServo(); // Plant flag outside starting area: 10 points
    HomeFlagServo();

    delay(10000);

    StrafeRight(16);
    RotateRight(180);
    StrafeLeft(18);

    ArmServo.attach(ArmServoPin);

    MoveArmServo();
    Forward(14);

    // Forward(29.5); // 11
    TripleButtonWombo(); // Nails the button three times: 15 points (antenna one of four)

    // WE ARE REHOMING AT THE GREE (SHORT FOR GREEN)
    StrafeLeft(2);
    Backward(28);
    HomeArmServo();
    delay(1000);

}

void PiFunctionality() { // 80 points
    Forward(2);
    // StrafeLeft(23.25); // away from the wall, to make drone room
    Serial.println("Pi stuff is starting");
    /////////////////////////////////////////////////////////////////////
    ///// This is where we write to the Pi and make the drone fly ///////
    digitalWrite(ToPi, HIGH);
    Serial.println("Writing to Pi");
    delay(1000);
    // Pi breaks out of loop thanks to receiving HIGH from Arduino
    // Pi launches drone: does its thing and hovers during movement: 30 points
    // Pi does its best to make the drone land right back on the robot: 50 points
    while (digitalRead(FromPi) == LOW) {
         Serial.println("Waiting for drone to finish...");
         delay(1000);
    }
    Serial.println("Pi is done. Arduino resumes.");
    Backward(2);
    ///// This is where we resume Arduino. No more of the Pi stuff! ///////
    ///////////////////////////////////////////////////////////////////////
}

void FuckSweeper() { // 15 points
    // we are moving all ducks on da far half into the blue here ideally
    // once ve are at ze endzone, we drop the flag
    Forward(16);
    StrafeRight(28.5);
    RotateRight(60);
    StrafeRight(10);
    Forward(2.5);
    MoveArmServo();
    delay(1000);
    HomeArmServo();
    Backward(1.75);
    RotateLeft(60);
    Backward(10);
    // now i am just doing guesswork
    HomeArmServo();
    StrafeRight(32); // facing keypad wall
    StrafeLeft(3);
    RotateLeft(90);
    StrafeLeft(4); // lined with DtapIn
    StrafeRight(12);

    delay(10000);

    RotateLeft(90);
    Backward(24);
    StrafeRight(33);
    StrafeLeft(1);
    Forward(13);

    StrafeRight(14);
    RotateRight(180);
    StrafeLeft(12);
    Backward(14);
    delay(10000);
}

void StupidChudDuck() { // 20 points
    // this is the one where we square them up again and bring in the lonely chud duck by the button
    Forward(24);
    StrafeRight(2);
    RotateLeft(45);
    StrafeRight(12);
    RotateRight(45);
    StrafeRight(8); // lowkey just do it
    StrafeLeft(2);

    delay(10000);
    
    Backward(14);
    StrafeLeft(18);
    Backward(20);
    // ending in starting area: 15 points
}

void IRTransmitter() {
    Serial.println("Sending IR signal bitch");
    IrSender.sendNEC(0xBB, 0x5C, 5); // better than our chances of becoming a billionaire off of blackjack
    delay(3000);
    digitalWrite(LEDBOARD_PIN, HIGH);
}
