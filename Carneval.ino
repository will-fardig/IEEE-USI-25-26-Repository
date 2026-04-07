#include <AccelStepper.h>
#include <Servo.h>
#include <Adafruit_MCP23X17.h>
#include <stdlib.h>

// green, blue, purple, grey

#define BUTTON_PIN 1

#define ENABLE_MOTOR 2 // enable pin for motor, controls speed. goes to ENA on encoder
#define MOTOR_IN1 3 // both inputs control the motor direction. IN1 is red?
#define MOTOR_IN2 4 // IN2 is black?

// solenoid defs
#define SOLENOID_7 0 // likely 0 through 3
#define SOLENOID_3 1
#define SOLENOID_8 3
#define SOLENOID_P 2
Adafruit_MCP23X17 mcp;

// photoresistor defs
#define PHOTORESISTOR_PIN A0
#define LED_PHOTO 12

// servo defs
Servo ARM_SERVO; // red=5V, blk=GND, white=pin
#define ARM_SERVO_PIN 13

void setup() {
  // helpers for base functions
  pinMode(BUTTON_PIN, INPUT);
  Serial.begin(9600);

  // helpers for DemoMotor
  pinMode(ENABLE_MOTOR, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  // helpers for DemoSolenoid
  mcp.begin_I2C();
  mcp.pinMode(SOLENOID_7, OUTPUT);
  mcp.pinMode(SOLENOID_3, OUTPUT);
  mcp.pinMode(SOLENOID_8, OUTPUT);
  mcp.pinMode(SOLENOID_P, OUTPUT);

  mcp.digitalWrite(SOLENOID_7, LOW);
  mcp.digitalWrite(SOLENOID_3, LOW);
  mcp.digitalWrite(SOLENOID_8, LOW);
  mcp.digitalWrite(SOLENOID_P, LOW);

  // helpers for DemoPhotoresistor
  pinMode(PHOTORESISTOR_PIN, INPUT_PULLUP);
  pinMode(LED_PHOTO, OUTPUT);

  // helpers for DemoServo
  ARM_SERVO.attach(ARM_SERVO_PIN);
  ARM_SERVO.write(180);
}

void loop() {
  AwaitButton();
  DemoMotor();
}

void AwaitButton() {
  while (digitalRead(BUTTON_PIN) == HIGH) {
    // Waiting for switch to be pushed
    delay(250);
  }
}

void DemoMotor() { // WORKS PROPERLY I GUESS
  digitalWrite(ENABLE_MOTOR, 255); // sets to speed of 255
  digitalWrite(MOTOR_IN1, HIGH); // 1-2 high-low spins the motor forward
  digitalWrite(MOTOR_IN2, LOW);
  delay(10000);
  digitalWrite(ENABLE_MOTOR, 0);
}

void DemoSolenoids() {
  // fire 7
  mcp.digitalWrite(SOLENOID_7, HIGH);
  delay(250);
  mcp.digitalWrite(SOLENOID_7, LOW);
  delay(250);

  // fire 3
  mcp.digitalWrite(SOLENOID_3, HIGH);
  delay(250);
  mcp.digitalWrite(SOLENOID_3, LOW);
  delay(250);

  // fire 7
  mcp.digitalWrite(SOLENOID_7, HIGH);
  delay(250);
  mcp.digitalWrite(SOLENOID_7, LOW);
  delay(250);

  // fire 3
  mcp.digitalWrite(SOLENOID_3, HIGH);
  delay(250);
  mcp.digitalWrite(SOLENOID_3, LOW);
  delay(250);

  // fire 8
  mcp.digitalWrite(SOLENOID_8, HIGH);
  delay(250);
  mcp.digitalWrite(SOLENOID_8, LOW);
  delay(250);

  // fire #
  mcp.digitalWrite(SOLENOID_P, HIGH);
  delay(250);
  mcp.digitalWrite(SOLENOID_P, LOW);
  delay(250);
}

void DemoPhotoresistor() { // WORKING CORRECTLY, BUT INVERTED
  int calibratedLight = analogRead(PHOTORESISTOR_PIN);
  Serial.println("------------------------------");
  Serial.print("Calibrated Light: ");
  Serial.println(calibratedLight);
  delay(1000);
  for (int i = 0; i < 10; i++) {
    Serial.println("Waiting for the great escape...");
    if (analogRead(PHOTORESISTOR_PIN) - calibratedLight >= 50) {
      Serial.println("Light dimmed. Breaking out!");
      digitalWrite(LED_PHOTO, HIGH);
      break;
    }
    delay(1000);
  }
}

void DemoServo() { // FINISHED AND WORKING PROPERLY
  for (int i = 0; i < 3; i++) {
  ARM_SERVO.write(180);
  delay(2000);
  ARM_SERVO.write(55);
  delay(2000);
  }
}