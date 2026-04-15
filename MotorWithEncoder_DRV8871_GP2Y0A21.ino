// DRV8871 motor control With GP2Y0A21 lidar
#include <GP2Y0A21YK.h>

#define PIN_PSD A0
#define IN1 9
#define IN2 10
#define IR_PIN A0

GP2Y0A21YK *psd;

const int stopDistanceCm = 20;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  psd = new GP2Y0A21YK(PIN_PSD);
  Serial.begin(9600);
}

void ON() {
  analogWrite(IN1, 128);
  digitalWrite(IN2, LOW);
}

void OFF() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
 digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
}

void loop() {
  float distance = psd->distance();
  float distanceIN = distance/2.54;
  Serial.print("Distance: ");
  Serial.print(distanceIN);
  Serial.println(" inches");
  delay(500);

  if (distance <= stopDistanceCm) {
    OFF();
  } else {
    ON();
  }

  delay(250);
}
