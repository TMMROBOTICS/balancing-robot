#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Motor pins
int IN1 = 8;
int IN2 = 9;
int IN3 = 10;
int IN4 = 11;
int ENA = 5;
int ENB = 6;

// PID values (YOU MUST TUNE)
float Kp = 15;
float Ki = 0;
float Kd = 0.8;

float targetAngle = 0;   // Upright position
float angle, error, lastError;
float integral, derivative;
float pidOutput;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 failed");
    while (1);
  }
}

void loop() {
  angle = getAngle();

  error = targetAngle - angle;
  integral += error;
  derivative = error - lastError;

  pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);
  pidOutput = constrain(pidOutput, -255, 255);

  moveMotor(pidOutput);

  lastError = error;

  Serial.println(angle);
}

float getAngle() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  float angle = atan2(ay, az) * 180 / PI;
  return angle;
}

void moveMotor(int speed) {
  if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    speed = -speed;
  }

  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}