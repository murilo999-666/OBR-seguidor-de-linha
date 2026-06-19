#include <Arduino.h>
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

  // Por padrao o robo SO anda pra frente: nenhuma roda gira pra tras (minimo 0).
  // Excecao: |erro| > 1700 (linha no ultimo sensor) indica curva de 90graus,
  // entao a roda interna pode girar pra tras para fazer o pivo fechado.
  int minSpeed = (abs(error) > 1700) ? -lfspeed : 0;

  lsp = constrain(lsp, minSpeed, 255);
  rsp = constrain(rsp, minSpeed, 255);

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