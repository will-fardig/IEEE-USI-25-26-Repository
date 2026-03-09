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
// PI COMMUNICATIONS
///////////////////////////////////////////////////////////////

#define FromPi 26 // Signals received from the Pi to the Arduino
#define ToPi 27  // Signals sent from the Arduino to the Pi

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
    HomeFlagServo();
    // PhotoresistStart(); // Auto-starts using a white LED start bar: 15 points

    // KeypadButtonFunc(); // Keypad movement, line up on wall, button function, rehome.

    // DuckAtKeypad(); // Strafe right, turn and push keypadducky, rerotating, hitting the wall, and rehoming.

    // FuckSweeper(); // Move to go counterclockwise around the crater. Rotating to catch ducks in 30 degree increments, pushing them in to endzone.
    // ^ also diagstrafing past the bluezone to rehome.

    // StupidChudDuck(); // catches the duck sitting by the button antenna, and squares up all other ducks in endzone, then rehome.

    PiFunctionality(); // Drone runs and does it stupid chud robot stuff and then lands, and we rehome.

    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //
    // RECEIPT FOR ALL THE COOL STUFF WE DID:
    // Robot leaves the starting area....................................10 points
    // Robot plants flag outside starting area...........................10 points
    // Antenna turned on (x2).....................................15*2 = 30 points
    // Ends the round in the starting area...............................15 points
    // Launch of the UAV (including 15 inch movement V/H)................30 points
    // Retrieval of the UAV on the robot.................................50 points
    // Robot autostarts using a white LED bar............................15 points
    // Five duckies acquired.......................................5*5 = 25 points
    // Will's awesome t-shirt being entered in the student design comp...15 points
    //
    // TOTAL INTENDED EARNED POINTS.....................................200 points
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
    Forward(1.5);
    Backward(1.25);

    Forward(1.5);
    Backward(1.25);

    Forward(1.5);
    Backward(1.25);

    Forward(1.5);
    Backward(1.25); // Should land us where we started...
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

void CrankHog() { // not used anymore
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
    FlagServo.write(180);
    delay(stdDelay);
}

void PhotoresistStart() { // 15 points
    bool Bounds = 0;
    int i = 0;

    Serial.print("-----------------------------");
    Serial.print("Previous Light: ");
    Serial.println(previousLight);
    while (Bounds == 0 && i <= 10) { 
        currentLight = analogRead(PHOTORESISTOR_PIN);
        Serial.print("Current Light: ");
        Serial.println(currentLight);
        int lightChange = currentLight - previousLight;
        if (lightChange >= LIGHT_THRESHOLD) {
            Bounds = 1; // ash baby
            Serial.print("WE BROKE OUT BITCH!!!");
            return;
        }
        delay(1000);
        i = i + 1;
    }
}

///////////////////////////////////////////////////////////////
// MODULAR INSTRUCTIONS TO EARN POINTS
///////////////////////////////////////////////////////////////

void DuckAtKeypad() { // 5 points
    // Move the keypad duck over to the blue zone and shove him in there
    StrafeRight(8);
    Forward(1.5);
    RotateLeft(30);
    StrafeRight(5);
    RotateLeft(15);
    StrafeRight(12);
    RotateRight(45);
    StrafeLeft(22);
    Backward(3);
    StrafeRight(24);
    Forward(12);
    StrafeLeft(24);
    Backward(16);
}

void KeypadButtonFunc() { // 50 points
    // MOVE TO KEYPAD FIRST
    Forward(16); // Robot leaves starting area: 10 points
    StrafeRight(23.375); // Line up horiz w/ keypad
    Backward(7.5);
    KeypadAlignPress(); // Keypad nonsense: 15 points
    Forward(10); // Gets us away from the keypad
    
    // MOVE TO BUTTON
    StrafeLeft(24);
    Forward(11.5); // look at
    TripleButtonWombo(); // Nails the button three times: 15 points (antenna one of four)

    // WE ARE REHOMING AT THE GREE (SHORT FOR GREEN)
    StrafeLeft(2);
    Backward(28);
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
    MoveFlagServo(90); // Plant flag outside starting area: 10 points
    HomeFlagServo();
}

void StupidChudDuck() { // 20 points
    // this is the one where we square them up again and bring in the lonely chud duck by the button

    // ending in starting area: 15 points
}

void AllAroundDeprecated() {
    // this can be used for fuck sweeper
    // starts when 10 away from the keypad
    DiagStrafeRearRight(29); // Strafes past the crater with very little clearance
    StrafeRight(34); // Ideally lines us square with the wall, may have some excess movement
    Forward(26); // Reaches the crank antenna, putting us exactly in position
    Backward(10); // Takes us away from the crank

    // REHOMING AND STUFF
    DiagStrafeFrontLeft(29); // Strafes past the crater and lines directly in with the wall
    StrafeLeft(42); // Takes us all the way back to the area with the first two antennas
    Backward(24); // Pulls back to middle of the area (make sure we don't run into the flag)
    StrafeLeft(22); // Lines us up with the wall, may have some excess movement
    Backward(14); // Brings us right into the green, ending round in starting area (green means good): 15 points
    delay(3000); // Some kind of aura farming while we all twist and shout or whatever
}
