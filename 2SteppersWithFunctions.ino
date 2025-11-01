#include <AccelStepper.h>

// Define the interface type (4 pins = full-step)
#define MotorInterfaceType 4

// Right and Left side stepper motors
AccelStepper RightSide(MotorInterfaceType, 8,9,10,11); //Yellow (in1), orange(in2), red(in3), brown(in4) respectively
AccelStepper LeftSide(MotorInterfaceType, 4,5,6,7);    //Blue (in1), purple(in2), gray(in3), white(in4) respectively 

void setup() {
  RightSide.setMaxSpeed(500);
  LeftSide.setMaxSpeed(500);

  RightSide.setAcceleration(1000);
  LeftSide.setAcceleration(1000);
  
}
// 0.0443 inches per step with 2.82 inch wheels
// 0.06 inches perstep with 3.82 inch wheels

void loop() 
{
  int Turn90 = 293;
  
  float RegMove1Inch = 1/ 0.0443; // moves one inch forward with rubber wheels
  float OmniMove1Inch = 1/ 0.06;  // moves one inch forward with omni wheels

  delay(1000);
  // Forward(5*RegMove1Inch);    // Forward
  // delay(1000);
  // Reverse(5*RegMove1Inch);    // Backward
  // delay(1000);
  TurnLeft(Turn90);    // Turn left
  delay(1000);
  TurnRight(295);   // Turn right
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
void TurnLeft(int stepCount) 
{
  RightSide.moveTo(RightSide.currentPosition() + stepCount);
  LeftSide.moveTo(LeftSide.currentPosition() - stepCount);

  while (RightSide.distanceToGo() != 0 || LeftSide.distanceToGo() != 0) {
    RightSide.run();
    LeftSide.run();
  }
}

// Turn right (right motor backward, left motor forward)
void TurnRight(int stepCount) 
{
  RightSide.moveTo(RightSide.currentPosition() - stepCount);
  LeftSide.moveTo(LeftSide.currentPosition() + stepCount);

  while (RightSide.distanceToGo() != 0 || LeftSide.distanceToGo() != 0) {
    RightSide.run();
    LeftSide.run();
  }
}
