#include <AccelStepper.h>

// Define the interface type (4 pins = full-step)
#define MotorInterfaceType 4

// Right and Left side stepper motors
AccelStepper RightSide(MotorInterfaceType, 8, 9, 10, 11);
AccelStepper LeftSide(MotorInterfaceType, 4, 5, 6, 7);

void setup() {
  RightSide.setMaxSpeed(500);
  LeftSide.setMaxSpeed(500);

  RightSide.setAcceleration(1000);
  LeftSide.setAcceleration(1000);
}
// 0.0443 inches per step with 2.82 inch wheels
// 0.06 inches perstep with 3.82 inch wheels

// Move both motors forward by stepCount
void Forward(long stepCount) {
  RightSide.moveTo(RightSide.currentPosition() + stepCount);
  LeftSide.moveTo(LeftSide.currentPosition() + stepCount);

  while (RightSide.distanceToGo() != 0 || LeftSide.distanceToGo() != 0) {
    RightSide.run();
    LeftSide.run();
  }
}

// Move both motors backward by stepCount
void Reverse(long stepCount) {
  RightSide.moveTo(RightSide.currentPosition() - stepCount);
  LeftSide.moveTo(LeftSide.currentPosition() - stepCount);

  while (RightSide.distanceToGo() != 0 || LeftSide.distanceToGo() != 0) {
    RightSide.run();
    LeftSide.run();
  }
}

// Turn left (left motor backward, right motor forward)
void TurnLeft(long stepCount) {
  RightSide.moveTo(RightSide.currentPosition() + stepCount);
  LeftSide.moveTo(LeftSide.currentPosition() - stepCount);

  while (RightSide.distanceToGo() != 0 || LeftSide.distanceToGo() != 0) {
    RightSide.run();
    LeftSide.run();
  }
}

// Turn right (right motor backward, left motor forward)
void TurnRight(long stepCount) {
  RightSide.moveTo(RightSide.currentPosition() - stepCount);
  LeftSide.moveTo(LeftSide.currentPosition() + stepCount);

  while (RightSide.distanceToGo() != 0 || LeftSide.distanceToGo() != 0) {
    RightSide.run();
    LeftSide.run();
  }
}

void loop() {
  Forward(2000);    // Both sides forward
  delay(1000);
  Reverse(2000);    // Both sides backward
  delay(1000);
  TurnLeft(2000);    // Turn left
  delay(1000);
  TurnRight(2000);   // Turn right
  delay(1000);
}
