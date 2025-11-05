#include <AccelStepper.h>

// Define the interface type (4 pins = full-step)
#define MotorInterfaceType 4

// Right and Left side stepper motors
AccelStepper RightSide(MotorInterfaceType, 8,9,10,11); //Yellow (in1), orange(in2), red(in3), brown(in4) respectively
AccelStepper LeftSide(MotorInterfaceType, 7,6,5,4);    //Blue (in1), purple(in2), gray(in3), white(in4) respectively 

void setup() {
  RightSide.setMaxSpeed(300);
  LeftSide.setMaxSpeed(300);

  RightSide.setAcceleration(200);
  LeftSide.setAcceleration(200);
  
  // Serial.begin(9600);
}
// 0.0443 inches per step with 2.82 inch wheels
// 0.06 inches perstep with 3.82 inch wheels

void loop() 
{
  int Turn90 = 293;
  
  float RegMove1Inch = 1/ 0.0443; // moves one inch forward with rubber wheels
  float OmniMove1Inch = 1/ 0.06;  // moves one inch forward with omni wheels

  delay(1000);
  // Serial.println("Now to move 27 in forward");
  Forward(27*OmniMove1Inch);
  delay(1000);
  // Serial.println("Now to move 18 in backward");
  Reverse(18*OmniMove1Inch);
  delay(1000);
  // Serial.println("Now to strafe x to the right");
  StrafeRight(Turn90); 
  delay(1000);
 }

// Move both motors forward by stepCount
void Forward(int stepCount) 
{
  RightSide.moveTo(RightSide.currentPosition() + stepCount);
  LeftSide.moveTo(LeftSide.currentPosition() + stepCount);

  while (RightSide.distanceToGo() != 0 || LeftSide.distanceToGo() != 0) {
    RightSide.run();
    LeftSide.run();
  }
}

// Move both motors backward by stepCount
void Reverse(int stepCount) 
{
  RightSide.moveTo(RightSide.currentPosition() - stepCount);
  LeftSide.moveTo(LeftSide.currentPosition() - stepCount);

  while (RightSide.distanceToGo() != 0 || LeftSide.distanceToGo() != 0) {
    RightSide.run();
    LeftSide.run();
  }
}

// Turn left (left motor backward, right motor forward)
void StrafeLeft(int stepCount) 
{
  RightSide.moveTo(RightSide.currentPosition() + stepCount);
  LeftSide.moveTo(LeftSide.currentPosition() - stepCount);

  while (RightSide.distanceToGo() != 0 || LeftSide.distanceToGo() != 0) {
    RightSide.run();
    LeftSide.run();
  }
}

// Turn right (right motor backward, left motor forward)
void StrafeRight(int stepCount) 
{
  RightSide.moveTo(RightSide.currentPosition() - stepCount);
  LeftSide.moveTo(LeftSide.currentPosition() + stepCount);

  while (RightSide.distanceToGo() != 0 || LeftSide.distanceToGo() != 0) {
    RightSide.run();
    LeftSide.run();
  }
}