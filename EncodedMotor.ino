#define ENA 4   // Enable pin for motor A controls motor speed (0-255) (has to be PWM)
#define IN1 30  // Inputs 1 and 2 control motor direction (can be any digital pin)
#define IN2 31

#define ENC_A 2  // Encoder channel A (has to be PWM)
#define ENC_B 3  // Encoder channel B (has to be PWM)

void setup() 
{
  // Set control pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Set encoder pins as inputs with internal pullup resistors
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
}

void loop() 
{
  digitalWrite(ENA, 255);   // Enable the motor driver (0-255) controls speed
  digitalWrite(IN1, LOW);   // Set IN1 HIGH and IN2 LOW to spin motor forward and vice versa for reverse
  digitalWrite(IN2, HIGH);  
}
