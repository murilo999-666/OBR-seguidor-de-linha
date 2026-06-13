#include <Arduino.h>
#include <Wire.h>
#include <L298N.h>
#include <QTRSensors.h>

#define LED_BUILTIN 5

#define AIN1 21
#define BIN1 25
#define AIN2 22
#define BIN2 33
#define PWMA 23
#define PWMB 32

const int offsetA = 1;
const int offsetB = 1;

L298N motor1(PWMA, AIN1, AIN2);
L298N motor2(PWMB, BIN1, BIN2);

QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
int threshold[SensorCount];

float Kp = 30;
float Ki = 2;
float Kd = 12;

uint8_t multiP = 2;
uint8_t multiI  = 3;
uint8_t multiD = 1;
uint8_t Kpfinal;
uint8_t Kifinal;
uint8_t Kdfinal;
float Pvalue;
float Ivalue;
float Dvalue;

uint16_t position;
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 200;

void parar();
void robot_control();
void PID_Linefollow(int error);
void motor_drive(int left, int right);

void setup()
{
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

  Wire.begin(34, 35);

  delay(1000);
}

void loop()
{
  robot_control();
}

void parar()
{
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
}

void robot_control()
{
  position = qtr.readLineBlack(sensorValues);
  error = 2000 - position;
  while (sensorValues[0] >= 980 && sensorValues[1] >= 980 && sensorValues[2] >= 980 && sensorValues[3] >= 980 && sensorValues[4] >= 980)
  {
    digitalWrite(LED_BUILTIN, LOW);
    if (previousError > 0)
    {
      motor_drive(-120, 120);
    }
    else
    {
      motor_drive(120, -120);
    }
    position = qtr.readLineBlack(sensorValues);
  }

  if (abs(error) < 300)
    digitalWrite(LED_BUILTIN, HIGH);
  else
    digitalWrite(LED_BUILTIN, LOW);

  PID_Linefollow(error);
}

void PID_Linefollow(int error)
{
  P = error;
  I = I + error;
  if (I > 5000)  I = 5000;
  if (I < -5000) I = -5000;
  D = error - previousError;

  Pvalue = (Kp / pow(10, multiP)) * P;
  Ivalue = (Ki / pow(10, multiI)) * I;
  Dvalue = (Kd / pow(10, multiD)) * D;

  float PIDvalue = Pvalue + Ivalue + Dvalue;
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  if (lsp > 255) lsp = 255;
  if (lsp < 0)   lsp = 0;
  if (rsp > 255) rsp = 255;
  if (rsp < 0)   rsp = 0;

  motor_drive(lsp, rsp);
}

void motor_drive(int left, int right)
{
  if (right > 0)
  {
    motor2.setSpeed(right);
    motor2.forward();
  }
  else
  {
    motor2.setSpeed(-right);
    motor2.backward();
  }

  if (left > 0)
  {
    motor1.setSpeed(left);
    motor1.backward();
  }
  else
  {
    motor1.setSpeed(-left);
    motor1.forward();
  }
}
