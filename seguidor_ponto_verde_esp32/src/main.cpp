/*
 * ════════════════════════════════════════════════════════════════
 *  SEGUIDOR DE LINHA (PID) + PONTO VERDE — ESP32 / OBR Resgate
 * ════════════════════════════════════════════════════════════════
 *
 *  Baseado no "seguidor_de_linha_PID" (QTRSensors + L298N).
 *  Adicionado: detecção e tratamento dos MARCADORES VERDES das
 *  interseções, usando DOIS sensores de cor TCS34725 (esq. e dir.).
 *
 *  >>> MODO ATUAL: o robô anda RETO e testa o ponto verde (SEM
 *      seguidor de linha). Para ligar o PID depois, mude os flags
 *      MODO_SEGUIDOR_LINHA e USAR_IR na seção "MODO DE OPERAÇÃO". <<<
 *
 *  ───────────────────────────────────────────────────────────────
 *  LÓGICA DO PONTO VERDE
 *  ───────────────────────────────────────────────────────────────
 *   • Verde nos DOIS lados ......................... gira 180° (beco sem saída)
 *   • Verde só num lado  E  IR com 1 preto (linha normal) .. gira 90° p/ esse lado
 *   • Verde + IR com VÁRIOS pretos (cruzamento) ... segue RETO
 *   • Sem verde .................................... só segue a linha (PID)
 *
 *  ───────────────────────────────────────────────────────────────
 *  SENSORES DE COR — dois TCS34725 em barramentos I2C separados
 *  ───────────────────────────────────────────────────────────────
 *  Os dois TCS34725 têm o mesmo endereço I2C (0x29), então cada um
 *  usa um barramento I2C de hardware diferente do ESP32:
 *    - Cor ESQUERDA : Wire  (SDA=16, SCL=17)  → I2C0
 *    - Cor DIREITA  : Wire1 (SDA=4,  SCL=5)   → I2C1
 *
 *  ───────────────────────────────────────────────────────────────
 *  MONTAGEM
 *  ───────────────────────────────────────────────────────────────
 *  Os 2 TCS34725 devem ficar UM POUCO À FRENTE do array de IR,
 *  um à esquerda e outro à direita da linha.
 * ════════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <L298N.h>
#include <QTRSensors.h>

// ─────────────────────────────────────────────────────────────
//  PINOS
// ─────────────────────────────────────────────────────────────
#define AIN1 21
#define AIN2 22
#define PWMA 23
#define BIN1 25
#define BIN2 33
#define PWMB 32

#define PIN_BUZZER 15

// I2C dos sensores de cor — dois barramentos de hardware
#define COR_ESQ_SDA 16   // Wire  (I2C0)
#define COR_ESQ_SCL 17
#define COR_DIR_SDA 4    // Wire1 (I2C1)
#define COR_DIR_SCL 5

// ─────────────────────────────────────────────────────────────
//  OBJETOS
// ─────────────────────────────────────────────────────────────
L298N motorEsq(PWMA, AIN1, AIN2);
L298N motorDir(PWMB, BIN1, BIN2);

QTRSensors qtr;
const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

// Dois TwoWire de hardware — ESP32 tem I2C0 e I2C1
TwoWire I2C_ESQ = TwoWire(0);
TwoWire I2C_DIR = TwoWire(1);

Adafruit_TCS34725 corEsq = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);
Adafruit_TCS34725 corDir = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);

// ─────────────────────────────────────────────────────────────
//  PID
// ─────────────────────────────────────────────────────────────
float Kp = 35;
float Ki = 0;
float Kd = 0;
uint8_t multiP = 1, multiI = 1, multiD = 1;
float Pvalue, Ivalue, Dvalue;
int P, I, D, previousError, error;
int lsp, rsp;
int lfspeed = 120;

// ─────────────────────────────────────────────────────────────
//  PARÂMETROS — CALIBRAR NO ROBÔ REAL
// ─────────────────────────────────────────────────────────────
const uint16_t LIMIAR_PRETO = 600;
const int N_PRETOS_CRUZAMENTO = 4;

const int VEL_GIRO      = 130;
const int VEL_GIRO_FINO = 110;
const int VEL_BUSCA     = 120;
const int T_GIRO_90     = 360;    // ← CALIBRAR
const int T_GIRO_180    = 720;    // ← CALIBRAR
const int T_AVANCO_EIXO = 220;

const int   COR_CLEAR_MIN   = 120;
const float COR_FATOR_VERDE = 1.20;
const float COR_PROP_VERDE  = 0.38;

const unsigned long COOLDOWN_VERDE = 700;

// ─────────────────────────────────────────────────────────────
//  MODO DE OPERAÇÃO
// ─────────────────────────────────────────────────────────────
const bool MODO_SEGUIDOR_LINHA = false;  // false = anda reto | true = segue linha por PID
const bool USAR_IR = false;              // false = ignora IR | true = lê QTR
const bool DEBUG   = true;

// ─────────────────────────────────────────────────────────────
//  ESTADO
// ─────────────────────────────────────────────────────────────
uint16_t rE, gE, bE, cE;
uint16_t rD, gD, bD, cD;
unsigned long ignorarVerdeAte = 0;

#define V_NENHUM 0
#define V_ESQ    1
#define V_DIR    2
#define V_AMBOS  3

// ─────────────────────────────────────────────────────────────
//  FORWARD DECLARATIONS
// ─────────────────────────────────────────────────────────────
void seguirLinhaPID(int err);
void motor_drive(int esq, int dir);
void parar();
void avancarReto(int vel, int t);
void andarReto();
void girar(int graus);
void reencontrarLinha(int dir);
int  contarPreto();
bool cruzamentoCompleto();
bool linhaPerdida();
bool ehVerde(uint16_t r, uint16_t g, uint16_t b, uint16_t c);
uint8_t lerVerde();
void beepCurto();
void debugPrint(uint8_t verde, bool cruz);

// ═════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  if (USAR_IR) {
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){26, 27, 14, 12, 13}, SensorCount);
  }

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  // Inicializa os dois barramentos I2C com pinos customizados
  I2C_ESQ.begin(COR_ESQ_SDA, COR_ESQ_SCL);
  I2C_DIR.begin(COR_DIR_SDA, COR_DIR_SCL);

  if (!corEsq.begin(TCS34725_ADDRESS, &I2C_ESQ))
    Serial.println(F("TCS34725 ESQUERDO nao encontrado (SDA16/SCL17)"));
  if (!corDir.begin(TCS34725_ADDRESS, &I2C_DIR))
    Serial.println(F("TCS34725 DIREITO  nao encontrado (SDA4/SCL5)"));

  // LED dos sensores sempre ligado (leitura estável)
  corEsq.setInterrupt(false);
  corDir.setInterrupt(false);

  if (USAR_IR) {
    beepCurto();
    for (uint16_t i = 0; i < 400; i++) qtr.calibrate();
    beepCurto(); delay(80); beepCurto();
  } else {
    beepCurto(); delay(80); beepCurto();
  }

  delay(500);
}

// ═════════════════════════════════════════════════════════════
//  LOOP
// ═════════════════════════════════════════════════════════════
void loop() {
  if (USAR_IR) {
    uint16_t position = qtr.readLineBlack(sensorValues);
    error = 2000 - position;
  }

  uint8_t verdeRaw = lerVerde();
  uint8_t verde = (millis() < ignorarVerdeAte) ? V_NENHUM : verdeRaw;

  bool cruz = USAR_IR ? cruzamentoCompleto() : false;

  if (verde == V_AMBOS) {
    girar(180);
    ignorarVerdeAte = millis() + COOLDOWN_VERDE + 200;

  } else if (verde == V_ESQ && !cruz) {
    girar(-90);
    ignorarVerdeAte = millis() + COOLDOWN_VERDE;

  } else if (verde == V_DIR && !cruz) {
    girar(+90);
    ignorarVerdeAte = millis() + COOLDOWN_VERDE;

  } else if (verde != V_NENHUM && cruz) {
    beepCurto();
    avancarReto(lfspeed, T_AVANCO_EIXO + 120);
    ignorarVerdeAte = millis() + COOLDOWN_VERDE;

  } else {
    if (MODO_SEGUIDOR_LINHA && USAR_IR) seguirLinhaPID(error);
    else                                andarReto();
  }

  if (DEBUG) debugPrint(verdeRaw, cruz);
}

// ═════════════════════════════════════════════════════════════
//  SEGUIDOR DE LINHA (PID)
// ═════════════════════════════════════════════════════════════
void seguirLinhaPID(int err) {
  if (linhaPerdida()) {
    if (previousError > 0) motor_drive(-VEL_BUSCA, VEL_BUSCA);
    else                   motor_drive(VEL_BUSCA, -VEL_BUSCA);
    return;
  }

  P = err;
  I = I + err;
  D = err - previousError;

  Pvalue = (Kp / pow(10, multiP)) * P;
  Ivalue = (Ki / pow(10, multiI)) * I;
  Dvalue = (Kd / pow(10, multiD)) * D;

  float PIDvalue = Pvalue + Ivalue + Dvalue;
  previousError = err;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  lsp = constrain(lsp, -150, 150);
  rsp = constrain(rsp, -150, 150);

  motor_drive(lsp, rsp);
}

// ═════════════════════════════════════════════════════════════
//  MOTORES
// ═════════════════════════════════════════════════════════════
void motor_drive(int esq, int dir) {
  if (dir >= 0) { motorDir.setSpeed(dir);  motorDir.forward(); }
  else          { motorDir.setSpeed(-dir); motorDir.backward(); }

  if (esq >= 0) { motorEsq.setSpeed(esq);  motorEsq.forward(); }
  else          { motorEsq.setSpeed(-esq); motorEsq.backward(); }
}

void parar() {
  motorEsq.stop();
  motorDir.stop();
}

void avancarReto(int vel, int t) {
  motor_drive(vel, vel);
  delay(t);
}

void andarReto() {
  motor_drive(lfspeed, lfspeed);
}

// ═════════════════════════════════════════════════════════════
//  GIRO
// ═════════════════════════════════════════════════════════════
void girar(int graus) {
  beepCurto();
  parar(); delay(80);

  avancarReto(lfspeed, T_AVANCO_EIXO);
  parar(); delay(60);

  int dir = (graus < 0) ? -1 : +1;
  int t   = (abs(graus) >= 180) ? T_GIRO_180 : T_GIRO_90;

  if (dir < 0) motor_drive(-VEL_GIRO, VEL_GIRO);
  else         motor_drive(VEL_GIRO, -VEL_GIRO);
  delay(t);
  parar(); delay(60);

  reencontrarLinha(dir);
}

void reencontrarLinha(int dir) {
  unsigned long t0 = millis();
  while (millis() - t0 < 600) {
    qtr.readLineBlack(sensorValues);
    if (sensorValues[2] > LIMIAR_PRETO) break;
    if (dir < 0) motor_drive(-VEL_GIRO_FINO, VEL_GIRO_FINO);
    else         motor_drive(VEL_GIRO_FINO, -VEL_GIRO_FINO);
  }
  parar(); delay(40);
}

// ═════════════════════════════════════════════════════════════
//  IR / QTR — auxiliares
// ═════════════════════════════════════════════════════════════
int contarPreto() {
  int n = 0;
  for (uint8_t i = 0; i < SensorCount; i++)
    if (sensorValues[i] > LIMIAR_PRETO) n++;
  return n;
}

bool cruzamentoCompleto() { return contarPreto() >= N_PRETOS_CRUZAMENTO; }
bool linhaPerdida()       { return contarPreto() == 0; }

// ═════════════════════════════════════════════════════════════
//  COR — detecção de VERDE por proporção de canais
// ═════════════════════════════════════════════════════════════
bool ehVerde(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  if (c < COR_CLEAR_MIN) return false;
  long soma = (long)r + g + b;
  if (soma <= 0) return false;
  float prop = (float)g / soma;
  return (g > r * COR_FATOR_VERDE) &&
         (g > b * COR_FATOR_VERDE) &&
         (prop > COR_PROP_VERDE);
}

uint8_t lerVerde() {
  corEsq.getRawData(&rE, &gE, &bE, &cE);
  corDir.getRawData(&rD, &gD, &bD, &cD);

  uint8_t res = V_NENHUM;
  if (ehVerde(rE, gE, bE, cE)) res |= V_ESQ;
  if (ehVerde(rD, gD, bD, cD)) res |= V_DIR;
  return res;
}

// ═════════════════════════════════════════════════════════════
//  BUZZER
// ═════════════════════════════════════════════════════════════
void beepCurto() {
  digitalWrite(PIN_BUZZER, HIGH);
  delay(60);
  digitalWrite(PIN_BUZZER, LOW);
}

// ═════════════════════════════════════════════════════════════
//  DEBUG (Monitor Serial @115200)
// ═════════════════════════════════════════════════════════════
void debugPrint(uint8_t verde, bool cruz) {
  static unsigned long t = 0;
  if (millis() - t < 200) return;
  t = millis();

  if (USAR_IR) {
    Serial.print(F("IR:"));
    for (uint8_t i = 0; i < SensorCount; i++)
      Serial.print(sensorValues[i] > LIMIAR_PRETO ? '#' : '.');
    Serial.print(F("  cruz=")); Serial.print(cruz);
  } else {
    Serial.print(F("IR:off"));
  }

  Serial.print(F("  Verde="));
  Serial.print(verde == V_AMBOS ? "AMBOS" : verde == V_ESQ ? "ESQ" :
               verde == V_DIR   ? "DIR"   : "-");

  Serial.print(F("  | E rgbc=")); Serial.print(rE); Serial.print(',');
  Serial.print(gE); Serial.print(','); Serial.print(bE); Serial.print(','); Serial.print(cE);
  Serial.print(F("  D rgbc=")); Serial.print(rD); Serial.print(',');
  Serial.print(gD); Serial.print(','); Serial.print(bD); Serial.print(','); Serial.print(cD);
  Serial.println();
}

/*
 * ════════════════════════════════════════════════════════════
 *  COMO CALIBRAR
 * ════════════════════════════════════════════════════════════
 *  1) IR / LIMIAR_PRETO e N_PRETOS_CRUZAMENTO
 *     Com DEBUG = true, abra o Monitor Serial. Coloque o robô na
 *     linha e veja o padrão "IR: . # . . .". Sobre um cruzamento (+)
 *     deve aparecer "# # # # #". Ajuste LIMIAR_PRETO até '#'
 *     aparecer só onde há preto.
 *
 *  2) COR / verde
 *     Veja os valores "E rgbc=" e "D rgbc=" passando cada sensor
 *     sobre: branco, preto e VERDE do marcador.
 *     Ajuste COR_FATOR_VERDE e COR_PROP_VERDE.
 *
 *  3) GIRO / T_GIRO_90 e T_GIRO_180
 *     Faça o robô girar e meça o ângulo. Bateria mais fraca gira
 *     menos — recalibre se trocar a carga.
 *
 * ════════════════════════════════════════════════════════════
 *  UPGRADE: GIROSCÓPIO (giro 90°/180° preciso)
 * ════════════════════════════════════════════════════════════
 *  Os pinos 16,17,4,5 já estão usados pelos sensores de cor.
 *  Para adicionar MPU6050 sem conflito, use um multiplexador
 *  I2C TCA9548A: tudo no Wire (SDA=16/SCL=17), e o mux dá um
 *  canal para o giroscópio e um para cada TCS34725.
 * ════════════════════════════════════════════════════════════
 */
