/*
 * ════════════════════════════════════════════════════════════════
 *  OBR 2026 — SEGUIDOR DE LINHA (PID) + DESVIO DE OBSTÁCULO — ESP32
 *  Fusão de:
 *    • seguidor_de_linha_PID_esp32.ino  (segue a linha com PID + QTR)
 *    • RoboDesvio.ino                   (desvia de obstáculos — contorno em U)
 *
 *  COMPORTAMENTO
 *  ──────────────────────────────────────────────────────────────
 *   - Por padrão SEGUE A LINHA usando o PID original (NÃO ALTERADO).
 *   - Quando o ultrassom detecta um obstáculo (<= DIST_PARADA cm),
 *     executa o CONTORNO EM U guiado pelo giroscópio (MPU-6050),
 *     retornando à linha logo após o obstáculo (regra OBR p.27).
 *   - Ao terminar o desvio, o PID volta a assumir e reencontra a
 *     linha (recuperação "linha perdida" do próprio código do PID).
 *
 *  O QUE FOI / NÃO FOI ALTERADO
 *  ──────────────────────────────────────────────────────────────
 *   - O CÁLCULO e a LÓGICA do PID são EXATAMENTE os do original
 *     (robot_control / PID_Linefollow / motor_drive — intactos).
 *   - A ROTINA DE DESVIO é EXATAMENTE a do RoboDesvio (mesma
 *     sequência de curvas/avanços, mesma integração do giroscópio,
 *     mesmos tempos e ângulos de compensação).
 *   - Única adaptação: as funções de baixo nível do desvio
 *     (moverFrente/moverRe/girarAngulo/pararMotores) passaram a usar
 *     a MESMA biblioteca L298N do seguidor, pois o PWM manual (ledc)
 *     do RoboDesvio conflitaria com o PWM da L298N nos mesmos pinos.
 *     As DIREÇÕES físicas continuam idênticas às do RoboDesvio.
 *
 *  MAPA DE PINOS
 *  ──────────────────────────────────────────────────────────────
 *   PONTE H        AIN1=21  AIN2=22  PWMA=23 | BIN1=25  BIN2=33  PWMB=32
 *   SENSORES QTR   26, 27, 14, 12, 13   (5 canais analógicos)
 *   ULTRASSOM      TRIG=19  ECHO=18
 *   MPU-6050 (I2C) SDA=16   SCL=17
 *   LED seguidor   5   |  LED desvio  2  |  BUZZER 15  |  START(BOOT) 0
 * ════════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <Wire.h>
#include <L298N.h>
#include <QTRSensors.h>
#include <MPU6050.h>
#include <Preferences.h>   // memória interna (NVS) p/ guardar a calibração do IR

// ═════════════════════════════════════════════════════════════
//  SEÇÃO 1 — SEGUIDOR DE LINHA (PID)   [original, não alterado]
// ═════════════════════════════════════════════════════════════
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

// ═════════════════════════════════════════════════════════════
//  SEÇÃO 2 — DESVIO DE OBSTÁCULO   [original RoboDesvio]
// ═════════════════════════════════════════════════════════════

// Sensor ultrassônico HC-SR04
#define PIN_TRIG   19
#define PIN_ECHO   18

// I2C do giroscópio MPU-6050
#define PIN_SDA    16
#define PIN_SCL    17

// Indicadores de estado do desvio
#define PIN_LED_INT 2     // LED sólido durante o desvio
#define PIN_BUZZER  15    // buzzer ativo

// Botão de start (GPIO0 = botão BOOT da placa, ativo LOW)
#define PIN_BTN_START 0

const int DIST_PARADA = 5;   // cm — distância para acionar o desvio

const uint8_t VEL_NORMAL = 255;
const uint8_t VEL_CURVA  = 220;

// Tempos do contorno em U (ajuste conforme velocidade e piso)
const int T_LATERAL = 1750;            // ms — avanço lateral (ida e volta)
const int T_FRENTE  = 3100;            // ms — avanço frontal (passa o obstáculo)
const int T_RETORNO = T_LATERAL + 0;   // ms — último lateral de retorno à linha

// Compensação de inércia nas curvas: motores param COMP_ANGULO° antes do
// alvo para que o deslizamento residual complete o ângulo.
const float COMP_ANGULO        = 12.0f;
const float COMP_ANGULO_2CURVA =  9.0f;

// Curva crítica (curva 3 do desvio): velocidade reduzida + comp mínima
const uint8_t VEL_CURVA_CRITICA   = 185;
const float   COMP_ANGULO_CRITICA =  3.0f;

MPU6050 mpu;
int16_t gyroZoffset = 0;

Preferences prefs;   // armazenamento da calibração do IR na flash interna

enum EstadoRobo { LIVRE, ATENTO, DESVIANDO, RECUANDO };
EstadoRobo estadoAtual = LIVRE;
long distCm = 999;

// ═════════════════════════════════════════════════════════════
//  FORWARD DECLARATIONS
// ═════════════════════════════════════════════════════════════
// Seguidor (PID)
void parar();
void robot_control();
void PID_Linefollow(int error);
void motor_drive(int left, int right);

// Desvio
void moverFrente(uint8_t vel);
void moverRe(uint8_t vel);
void girarAngulo(float graus, uint8_t vel, float comp = COMP_ANGULO);
void pararMotores();
static void aguardar(unsigned long ms);
void desviarObstaculo();
void calibrarIMU();
long lerDistancia();
static void _bip(int ms);
void beepInicio();
void beepAviso();

// Calibração do IR (QTR) com persistência na flash
bool carregarCalibracaoQTR();
void salvarCalibracaoQTR();
void calibrarQTR_fina();
bool esperarBotao(unsigned long ms);
void aguardarStart();

// ═════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════
void setup()
{
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n========================================"));
  Serial.println(F("  OBR 2026 — Seguidor PID + Desvio ESP32 "));
  Serial.println(F("========================================\n"));

  // — Motores parados ao iniciar (biblioteca L298N) —
  motor1.stop();
  motor2.stop();

  // — Periféricos do desvio —
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_LED_INT, OUTPUT);  digitalWrite(PIN_LED_INT, LOW);
  pinMode(PIN_BUZZER,  OUTPUT);  digitalWrite(PIN_BUZZER,  LOW);
  pinMode(PIN_BTN_START, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  // — I2C + MPU-6050 (giroscópio usado nas curvas do desvio) —
  //   Obs.: o original do PID chamava Wire.begin(34,35), mas 34/35 são
  //   pinos SOMENTE-ENTRADA do ESP32 e não servem para I2C; o MPU exige
  //   os pinos SDA=16 / SCL=17 do RoboDesvio.
  Wire.begin(PIN_SDA, PIN_SCL);
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println(F("[!] MPU-6050 nao encontrado! Verifique a ligacao I2C."));
  } else {
    Serial.println(F("MPU-6050 OK"));
    calibrarIMU();                 // robô deve ficar PARADO durante esta etapa
  }

  // — Configura os sensores de linha (QTR) —
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){26, 27, 14, 12, 13}, SensorCount);
  delay(200);

  // — Calibração do IR: feita UMA vez e guardada na memória interna —
  //   Tenta carregar a calibração salva na flash.
  bool temCal = carregarCalibracaoQTR();
  if (temCal) Serial.println(F("Calibracao IR encontrada na memoria interna."));
  else        Serial.println(F("Nenhuma calibracao IR salva ainda."));

  // — Janela de 3 s: aperte BOOT para RECALIBRAR os sensores IR —
  //   (se não apertar e já houver calibração salva, segue normalmente)
  Serial.println(F("Aperte BOOT em 3s para RECALIBRAR os sensores IR..."));
  beepAviso();
  bool querRecal = esperarBotao(3000);

  if (querRecal || !temCal) {
    // Calibração fina e demorada (~12 s) + gravação na flash
    calibrarQTR_fina();
    salvarCalibracaoQTR();
    // Após recalibrar, aguarda um toque no BOOT para iniciar a rodada
    Serial.println(F("Calibracao concluida. Aperte BOOT para iniciar."));
    aguardarStart();
  } else {
    Serial.println(F("Usando calibracao salva — seguindo normalmente."));
  }

  beepInicio();
  delay(300);
  Serial.println(F("Pronto! Seguindo a linha.\n"));
}

// ═════════════════════════════════════════════════════════════
//  LOOP PRINCIPAL
//  Segue a linha com o PID; se houver obstáculo à frente, desvia.
// ═════════════════════════════════════════════════════════════
void loop()
{
  distCm = lerDistancia();

  if (distCm > 0 && distCm <= DIST_PARADA) {
    desviarObstaculo();   // contorna o obstáculo (exatamente como no RoboDesvio)
  } else {
    robot_control();      // segue a linha com o PID (lógica/cálculo INALTERADOS)
  }
}

// ═════════════════════════════════════════════════════════════
//  SEGUIDOR DE LINHA — PID   [IDÊNTICO ao código original]
// ═════════════════════════════════════════════════════════════
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

// ═════════════════════════════════════════════════════════════
//  MOVIMENTOS DO DESVIO
//  Mesmas direções físicas do RoboDesvio, agora via L298N
//  (para usar o MESMO PWM do seguidor e não conflitar).
//
//  Equivalência de direções (conferida pelos pinos):
//    Frente do robô  ->  motor1.backward() + motor2.forward()
//    Ré do robô      ->  motor1.forward()  + motor2.backward()
//    Girar direita   ->  motor1.backward() + motor2.backward()
//    Girar esquerda  ->  motor1.forward()  + motor2.forward()
// ═════════════════════════════════════════════════════════════
void moverFrente(uint8_t vel) {
  motor1.setSpeed(vel); motor1.backward();
  motor2.setSpeed(vel); motor2.forward();
}

void moverRe(uint8_t vel) {
  motor1.setSpeed(vel); motor1.forward();
  motor2.setSpeed(vel); motor2.backward();
}

// graus > 0 = direita, graus < 0 = esquerda
// (integração do giroscópio idêntica à do RoboDesvio)
void girarAngulo(float graus, uint8_t vel, float comp) {
  // Offset local: 10 amostras espaçadas em 15 ms (~150 ms) para garantir
  // que qualquer vibração residual do movimento anterior morreu.
  int16_t ax, ay, az, gx, gy, gz;
  long offSum = 0;
  for (int i = 0; i < 10; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    offSum += gz;
    aguardar(15);
  }
  int16_t offsetAtual = (int16_t)(offSum / 10);

  bool paraDireita = graus > 0;
  float acumulado = 0.0f;
  // Para COMP_ANGULO° antes do alvo — o deslizamento cobre o restante
  float alvo = fmaxf(fabsf(graus) - comp, 0.0f);
  unsigned long tAnt = micros();

  // Direção/velocidade do giro (constantes durante a curva)
  if (paraDireita) {
    motor1.setSpeed(vel); motor1.backward();   // motor A "frente"
    motor2.setSpeed(vel); motor2.backward();   // motor B "ré"
  } else {
    motor1.setSpeed(vel); motor1.forward();    // motor A "ré"
    motor2.setSpeed(vel); motor2.forward();    // motor B "frente"
  }

  while (acumulado < alvo) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    unsigned long tAgora = micros();
    float dt = (tAgora - tAnt) * 1e-6f;
    tAnt = tAgora;

    // 131 LSB/(°/s) para faixa ±250°/s (padrão MPU-6050)
    acumulado += fabsf((gz - offsetAtual) / 131.0f) * dt;
  }
  pararMotores();
}

void pararMotores() {
  motor1.stop();
  motor2.stop();
}

static void aguardar(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms);
}

// ═════════════════════════════════════════════════════════════
//  DESVIO DE OBSTÁCULO OBR — CONTORNO EM U   [IDÊNTICO ao RoboDesvio]
//  Sequência: +90° → frente → -90° → frente → -90° → frente → +90°
//  Robô retorna à linha logo após o obstáculo (regra OBR p.27).
// ═════════════════════════════════════════════════════════════
void desviarObstaculo() {
  estadoAtual = DESVIANDO;
  digitalWrite(PIN_LED_INT, HIGH);  // LED sólido = executando desvio
  beepAviso();
  pararMotores();
  aguardar(200);

  Serial.println(F("  OBR: iniciando contorno em U"));

  // 1 — Curva 90° à direita (sai da linha)
  girarAngulo(90.0f, VEL_CURVA);
  aguardar(450);

  // 2 — Avança lateralmente (ultrapassa a largura do obstáculo)
  moverFrente(VEL_NORMAL);
  aguardar(T_LATERAL);
  pararMotores();
  aguardar(450);

  // 3 — Curva 90° à esquerda (retoma direção original) — CURVA CRÍTICA
  aguardar(400);  // acomodação extra antes de amostrar offset
  girarAngulo(-90.0f, VEL_CURVA_CRITICA, COMP_ANGULO_CRITICA);
  aguardar(500);

  // 4 — Avança à frente (ultrapassa o comprimento do obstáculo)
  moverFrente(VEL_NORMAL);
  aguardar(T_FRENTE);
  pararMotores();
  aguardar(450);

  // 5 — Curva 90° à esquerda (aponta de volta à linha)
  girarAngulo(-90.0f, VEL_CURVA);
  aguardar(450);

  // 6 — Avança lateral de retorno (alcança a linha)
  moverFrente(VEL_NORMAL);
  aguardar(T_RETORNO);
  pararMotores();
  aguardar(450);

  // 7 — Curva 90° à direita (restabelece direção original)
  girarAngulo(90.0f, VEL_CURVA);
  aguardar(450);

  Serial.println(F("  OBR: desvio concluido — retomando seguidor de linha"));
  digitalWrite(PIN_LED_INT, LOW);   // apaga LED ao retomar a linha
  estadoAtual = LIVRE;
  // A partir daqui o loop() volta a chamar robot_control() e o PID
  // reencontra a linha (recuperação "linha perdida" do próprio PID).
}

// ═════════════════════════════════════════════════════════════
//  CALIBRAÇÃO IMU (MPU-6050)   [IDÊNTICO ao RoboDesvio]
// ═════════════════════════════════════════════════════════════
void calibrarIMU() {
  Serial.println(F("  Calibrando IMU — mantenha o robo parado..."));
  digitalWrite(LED_BUILTIN, HIGH);   // LED sólido = calibrando giroscópio (não mova o robô)
  long soma = 0;
  const int N = 500;
  for (int i = 0; i < N; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    soma += gz;
    delay(2);
  }
  digitalWrite(LED_BUILTIN, LOW);    // LED apaga = giroscópio pronto
  gyroZoffset = (int16_t)(soma / N);
  Serial.printf("  Offset Z: %d | Calibracao concluida!\n", gyroZoffset);
}

// ═════════════════════════════════════════════════════════════
//  CALIBRAÇÃO DO IR (QTR) — UMA vez, guardada na flash (NVS)
// ═════════════════════════════════════════════════════════════

// Carrega min/max salvos. Retorna true se havia calibração válida.
bool carregarCalibracaoQTR() {
  prefs.begin("qtrcal", true);            // namespace, modo somente-leitura
  bool ok = prefs.getBool("ok", false);
  if (ok) {
    // Uma chamada de calibrate() força a biblioteca a alocar os vetores
    // calibrationOn.minimum/maximum; em seguida sobrescrevemos com a flash.
    qtr.calibrate();
    for (uint8_t i = 0; i < SensorCount; i++) {
      char km[8], kx[8];
      sprintf(km, "min%u", (unsigned)i);
      sprintf(kx, "max%u", (unsigned)i);
      qtr.calibrationOn.minimum[i] = prefs.getUShort(km, 0);
      qtr.calibrationOn.maximum[i] = prefs.getUShort(kx, 1023);
      threshold[i] = (qtr.calibrationOn.minimum[i] + qtr.calibrationOn.maximum[i]) / 2;
    }
  }
  prefs.end();
  return ok;
}

// Grava os min/max atuais (após uma calibração) na flash interna.
void salvarCalibracaoQTR() {
  prefs.begin("qtrcal", false);           // modo leitura/escrita
  for (uint8_t i = 0; i < SensorCount; i++) {
    char km[8], kx[8];
    sprintf(km, "min%u", (unsigned)i);
    sprintf(kx, "max%u", (unsigned)i);
    prefs.putUShort(km, qtr.calibrationOn.minimum[i]);
    prefs.putUShort(kx, qtr.calibrationOn.maximum[i]);
  }
  prefs.putBool("ok", true);
  prefs.end();
  Serial.println(F("Calibracao do IR salva na memoria interna."));
}

// Calibração FINA e DEMORADA (~12 s): quanto mais amostras enquanto você
// passa o robô sobre a linha, mais preciso fica o min/max de cada sensor.
void calibrarQTR_fina() {
  Serial.println(F("Calibracao FINA do IR (~12s) — passe o robo sobre a"));
  Serial.println(F("linha e o fundo claro repetidamente, devagar..."));
  beepAviso();

  qtr.resetCalibration();                 // zera min/max antigos (recalibração limpa)
  digitalWrite(LED_BUILTIN, HIGH);

  unsigned long t0 = millis();
  const unsigned long DURACAO = 12000;    // ms — aumente para ainda mais precisão
  while (millis() - t0 < DURACAO) {
    qtr.calibrate();
    digitalWrite(PIN_LED_INT, (millis() / 120) % 2);  // LED pisca = calibrando
  }

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(PIN_LED_INT, LOW);

  for (uint8_t i = 0; i < SensorCount; i++)
    threshold[i] = (qtr.calibrationOn.minimum[i] + qtr.calibrationOn.maximum[i]) / 2;

  beepAviso();
  Serial.println(F("Calibracao fina concluida."));
}

// Aguarda um toque no BOOT por até 'ms'. Retorna true se foi pressionado.
bool esperarBotao(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    if (digitalRead(PIN_BTN_START) == LOW) {     // ativo em LOW
      delay(50);                                 // debounce
      if (digitalRead(PIN_BTN_START) == LOW) {
        while (digitalRead(PIN_BTN_START) == LOW) delay(10);  // espera soltar
        digitalWrite(PIN_LED_INT, LOW);
        return true;
      }
    }
    digitalWrite(PIN_LED_INT, (millis() / 150) % 2);  // LED pisca rápido = janela aberta
    delay(10);
  }
  digitalWrite(PIN_LED_INT, LOW);
  return false;
}

// Bloqueia até o BOOT ser pressionado (usado para iniciar a rodada).
void aguardarStart() {
  while (digitalRead(PIN_BTN_START) == HIGH) {
    digitalWrite(PIN_LED_INT, (millis() / 400) % 2);  // LED pisca devagar = aguardando start
    delay(10);
  }
  delay(50);                                            // debounce
  while (digitalRead(PIN_BTN_START) == LOW) delay(10);  // espera soltar
  digitalWrite(PIN_LED_INT, LOW);
}

// ═════════════════════════════════════════════════════════════
//  SENSOR ULTRASSÔNICO HC-SR04   [IDÊNTICO ao RoboDesvio]
// ═════════════════════════════════════════════════════════════
long lerDistancia() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  long duracao = pulseIn(PIN_ECHO, HIGH, 30000UL);

  if (duracao == 0) return 999;

  return duracao / 58L;
}

// ═════════════════════════════════════════════════════════════
//  BUZZER ATIVO   [IDÊNTICO ao RoboDesvio]
// ═════════════════════════════════════════════════════════════
static void _bip(int ms) {
  digitalWrite(PIN_BUZZER, HIGH);
  delay(ms);
  digitalWrite(PIN_BUZZER, LOW);
}

void beepInicio() {
  for (int i = 0; i < 3; i++) {
    _bip(100);
    delay(100);
  }
}

void beepAviso() {
  _bip(220);
}
