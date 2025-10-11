int IN1 = 23;
int IN2 = 22;

static void pushShit(int delayTime) {
  // Pushes servo OUT, total of 4ish seconds to complete
  digitalWrite(IN1, LOW); // change to HIGH to create a pause
  digitalWrite(IN2, HIGH);
  delay(delayTime);
}

static void pullShit(int delayTime) {
  // Pulls servo IN, total of 4ish seconds to complete
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW); // change to HIGH to create a pause
  delay(delayTime);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
}

void loop() {
  pushShit(3750);
  pullShit(3750);
}

