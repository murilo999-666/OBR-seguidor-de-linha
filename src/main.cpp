/*
resolver o problema do gap, preciso arrumar um jeito, provavelmente utilizando o giroscopio, pra ele analisar em varias direções
possiveis se tem linha, e voltando pra posição original, sabendo que o tamanho máximo de um gap é 15cm, e ele sempre esta em uma linha reta.
*/

#include <Arduino.h>
#include <NewPing.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <L298N.h>
#include <QTRSensors.h>

const int pinEcho = 18;
const int pinTrig = 19;
const int maxDistance = 200;
int distancia;

NewPing sensorUltra(pinTrig, pinEcho, maxDistance);

MPU6050 mpu6050(Wire);

#undef LED_BUILTIN
#define LED_BUILTIN 5

#define AIN1 21
#define BIN1 25
#define AIN2 22
#define BIN2 33
#define PWMA 23
#define PWMB 32
#define BUZZER 15

const int offsetA = 1;
const int offsetB = 1;

L298N motor1(PWMA, AIN1, AIN2);
L298N motor2(PWMB, BIN1, BIN2);

QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
int threshold[SensorCount];

float Kp = 12;
float Ki = 0;
float Kd = 0.3;

uint8_t multiP = 1;
uint8_t multiI  = 1;
uint8_t multiD = 1;
uint8_t Kpfinal;
uint8_t Kifinal;
uint8_t Kdfinal;
float Pvalue;
float Ivalue;
float Dvalue;

float angleZ = 0;
float offsetZ = 0;

uint16_t position;
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 150;

// Forward declarations
void resetAngleZ();
void beep();
void parar();
void lerDistancia();
void updateGyro();
void desviar();
void robot_control();
void PID_Linefollow(int error);
void motor_drive(int left, int right);

void setup()
{
  Serial.begin(115200);
  Wire.begin(16, 17);

  beep();
  mpu6050.begin();
  delay(1000);
  mpu6050.calcGyroOffsets(true);
  beep();

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){26, 27, 14, 12, 13}, SensorCount);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    threshold[i] = (qtr.calibrationOn.minimum[i] + qtr.calibrationOn.maximum[i]) / 2;
  }

  beep();
  delay(1000);
  beep();
}

void loop()
{
  lerDistancia();
  updateGyro();

  if (distancia < 8) {
    parar();
    delay(500);
  } else {
    robot_control();
  }
}

void resetAngleZ() {
  offsetZ = mpu6050.getAngleZ();
}

void beep() {
  analogWrite(BUZZER, 180);
  delay(150);
  analogWrite(BUZZER, 0);
}

void parar() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
}

void lerDistancia() {
  static unsigned long tempo = millis();

  if ((millis() - tempo) > 10) {
    distancia = sensorUltra.ping_cm();
    tempo = millis();
  }
}

void updateGyro() {
  mpu6050.update();
  angleZ = mpu6050.getAngleZ() - offsetZ;
}

void desviar() {
  parar();

  motor1.setSpeed(120);
  motor2.setSpeed(120);

  delay(300);

  resetAngleZ();

  unsigned long tempo = millis();

  while (angleZ < 80 && millis() - tempo < 3000) {
    updateGyro();
    Serial.println(angleZ);

    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);

    delay(1);
  }

  parar();
  delay(100);

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  delay(1000);
  parar();
}

void robot_control() {
  position = qtr.readLineBlack(sensorValues);
  error = 2000 - position;
  while (sensorValues[0] >= 980 && sensorValues[1] >= 980 && sensorValues[2] >= 980 &&
         sensorValues[3] >= 980 && sensorValues[4] >= 980) {
    if (previousError > 0) {
      motor_drive(-150, 150);
    } else {
      motor_drive(150, -150);
    }
    position = qtr.readLineBlack(sensorValues);
  }

  PID_Linefollow(error);
}

void PID_Linefollow(int error) {
  P = error;
  I = I + error;
  D = error - previousError;

  Pvalue = (Kp / pow(10, multiP)) * P;
  Ivalue = (Ki / pow(10, multiI)) * I;
  Dvalue = (Kd / pow(10, multiD)) * D;

  float PIDvalue = Pvalue + Ivalue + Dvalue;
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  if (lsp > 180) lsp = 180;
  if (lsp < -180) lsp = -180;
  if (rsp > 180) rsp = 180;
  if (rsp < -180) rsp = -180;

  motor_drive(lsp, rsp);
}

void motor_drive(int left, int right) {
  if (right > 0) {
    motor2.setSpeed(right);
    motor2.forward();
  } else {
    motor2.setSpeed(-right);
    motor2.backward();
  }

  if (left > 0) {
    motor1.setSpeed(left);
    motor1.forward();
  } else {
    motor1.setSpeed(-left);
    motor1.backward();
  }
}
