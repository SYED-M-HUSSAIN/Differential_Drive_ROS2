// Motor encoder output pulse per rotation (change as required)
double enc_count_rev = 1120; // 16*70 CPR

// Encoder output to Arduino Interrupt pin
#define ENCA 2
#define ENCB 3

// PWM/Speed pins for Motors
#define ENA 8
#define ENB 9

// Motor Directions
#define IN1A 51
#define IN2A 47
#define IN1B 50
#define IN2B 46

// PID constants
double ki = 0.7;
double kp = 0.5;
float error = 0;
long cumerror = 0;
long out = 0;
double gain = 1.1;  // have to set this by comparing motors

// RPM calculation
long currentMillis;
long prevMillis = 0;
double interval = 10;     // ms
double left_rpm, right_rpm;
double elapsedTime;
volatile long encoderValueA;
volatile long encoderValueB;

char command; // Stores the command received from Serial Monitor

void setup() {
  // Setup Serial Monitor
  Serial.begin(9600);

  // Set PWM and DIR connections as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1A, OUTPUT);
  pinMode(IN2A, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1B, OUTPUT);
  pinMode(IN2B, OUTPUT);

  // Attach interrupt
  attachInterrupt(digitalPinToInterrupt(ENCA), updateEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB), updateEncoderB, RISING);
  Serial.println("STARTING ");

}

void loop() {
  currentMillis = millis();
  elapsedTime = currentMillis - prevMillis;
  if (elapsedTime > interval) {
    noInterrupts();
    prevMillis = millis();
    right_rpm = (encoderValueA / enc_count_rev) * 60 * 1000;
    right_rpm = right_rpm / interval;
    left_rpm = (encoderValueB / enc_count_rev) * 60 * 1000;
    left_rpm = left_rpm / interval;
    encoderValueA = 0;
    encoderValueB = 0;
    interrupts();
  }
  error = right_rpm - left_rpm;
  cumerror += error * elapsedTime / 1000;
  out = kp * error + ki * cumerror;

  if (error >= 0) {
    left_rpm = left_rpm + abs(out) * gain;
    left_rpm = left_rpm * (255 / 150);
    analogWrite(ENB, left_rpm);
  } else {
    left_rpm = left_rpm - abs(out) * gain;
    left_rpm = left_rpm * (255 / 150);
    analogWrite(ENB, left_rpm);
  }

  // Read and process commands from the Serial Monitor
  if (Serial.available() > 0) {
    command = Serial.read();
    processCommand(command);
  }

//  // Print motor status
//  Serial.print(error);
//  Serial.println(" error");
//  Serial.print(left_rpm);
//  Serial.println(" left rpm");
//  Serial.print(right_rpm);
//  Serial.println(" right rpm");
//  Serial.println(" ");
//  delay(10);
}

void updateEncoderA() {
  // Increment value for each pulse from encoder
  encoderValueA++;
}
void updateEncoderB() {
  // Increment value for each pulse from encoder
  encoderValueB++;
}

void processCommand(char cmd) {
  // Add code to handle commands from the Serial Monitor
  switch (cmd) {
    case 'F':
      // Code to move forward
      digitalWrite(IN1A, HIGH);
      digitalWrite(IN2A, LOW);
      analogWrite(ENA, 400);
      digitalWrite(IN1B, LOW);
      digitalWrite(IN2B, HIGH);
      analogWrite(ENB, 420);
      Serial.println("MOVING FORWARD");
      break;
    case 'B':
      // Code to move backward
      digitalWrite(IN1A, LOW);
      digitalWrite(IN2A, HIGH);
      analogWrite(ENA, 100);
      digitalWrite(IN1B, HIGH);
      digitalWrite(IN2B, LOW);
      analogWrite(ENB, 120);
      Serial.println("MOVING BACKWARD");
      break;
    case 'S':
      // Code to stop
      digitalWrite(IN1A, LOW);
      digitalWrite(IN2A, LOW);
      analogWrite(ENA, 100);
      digitalWrite(IN1B, LOW);
      digitalWrite(IN2B, LOW);
      analogWrite(ENB, 120);
      Serial.println("STOP!!");
      break;
    // Add more cases for other commands as needed
  }
}
