#include <QTRSensors.h>

QTRSensors qtr;   
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

int AIN1 = 2;
int AIN2 = 3;
int PWMA = 5;
int BIN1 = 4;
int BIN2 = 7;
int PWMB = 6;
int STBY = 8;


int baseSpeed = 120;   
float integral = 0;
float lastError = 0;
float kp = 0.09;  
float ki = 0.0;
float kd = 0.0;

void setup() {
  
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  
  Serial.begin(9600);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(12);  

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println("Calibrating...");
  for (uint16_t i = 0; i < 250; i++) {
    qtr.calibrate();
    delay(20);
  }
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Calibration Done!");

  Serial.println("Min values:");
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print('\t');
  }
  Serial.println();
  Serial.println("Max values:");
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print('\t');
  }
  Serial.println();
  Serial.println("---- Calibration Finished ----");
  delay(2000);
}

void loop() {
  qtr.readCalibrated(sensorValues);
  for (int i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }

  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - 3500;  

  integral += error;
  int derivative = error - lastError;
  int correction = kp * error + ki * integral + kd * derivative;

  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;


  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);


  setMotor(AIN1, AIN2, PWMA, leftSpeed);
  setMotor(BIN1, BIN2, PWMB, rightSpeed);

  lastError = error;

  Serial.print(" | Pos: "); Serial.print(position);
  Serial.print(" Err: "); Serial.print(error);
  Serial.print(" L: "); Serial.print(leftSpeed);
  Serial.print(" R: "); Serial.println(rightSpeed);

  delay(100);
}

void setMotor(int in1, int in2, int pwm, int speed) {
  if (speed >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm, speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm, -speed);
  }
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}
