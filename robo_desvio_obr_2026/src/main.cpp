/*
 * ════════════════════════════════════════════════════════════════
 *  ROBÔ DESVIADOR DE OBSTÁCULOS — ESP32
 *  Inspirado no projeto Q1088 — Canal Brincando com Ideias
 *  Adaptado para ESP32 com pinagem personalizada
 *  Preparado para integração futura com seguidor de linha (5× IR)
 * ════════════════════════════════════════════════════════════════
 *
 *  ┌─────────────────────────────────────────────────────────┐
 *  │                    MAPA DE PINOS                        │
 *  ├──────────────────────┬──────────────────────────────────┤
 *  │ PONTE H              │ AIN1=D21  AIN2=D22  PWMA=D23     │
 *  │ (TB6612FNG/DRV8833)  │ BIN1=D25  BIN2=D33  PWMB=D32     │
 *  ├──────────────────────┼──────────────────────────────────┤
 *  │ ULTRASSOM HC-SR04    │ TRIG=D19  ECHO=D18               │
 *  ├──────────────────────┼──────────────────────────────────┤
 *  │ SENSORES IR (5×)     │ EE=D26  E=D27  C=D14             │
 *  │ (futura integração)  │ D=D12   ED=D13                   │
 *  ├──────────────────────┼──────────────────────────────────┤
 *  │ BUZZER ATIVO         │ D15                              │
 *  ├──────────────────────┼──────────────────────────────────┤
 *  │ I2C (display opt.)   │ SDA=D16  SCL=D17                 │
 *  └──────────────────────┴──────────────────────────────────┘
 *
 *  COMPATIBILIDADE PWM
 *  ────────────────────────────────────────────────────────────
 *  Core 2.x → ledcSetup() + ledcAttachPin() + ledcWrite(canal,v)
 *  Core 3.x → ledcAttach(pino,freq,bits)    + ledcWrite(pino,v)
 *  O código detecta a versão do core automaticamente.
 *
 *  ⚠  AVISOS GPIO DO ESP32
 *  ────────────────────────────────────────────────────────────
 *  GPIO12 (IR_D): nível deve ser LOW durante boot. O módulo IR
 *                 externo geralmente já garante isso, mas se
 *                 houver falha de boot substitua por D34/D35
 *                 (somente entrada) ou use resistor 10kΩ ao GND.
 *  GPIO0:  botão BOOT — nunca force LOW em operação normal.
 *  GPIO2:  LED onboard em muitas placas — usar com cautela.
 * ════════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>

// ─────────────────────────────────────────────────────────────
//  SEÇÃO 1 — PINAGEM
// ─────────────────────────────────────────────────────────────

// Ponte H
#define PIN_AIN1   21
#define PIN_AIN2   22
#define PIN_PWMA   23
#define PIN_BIN1   25
#define PIN_BIN2   33
#define PIN_PWMB   32

// Sensor ultrassônico
#define PIN_TRIG   19
#define PIN_ECHO   18

// Sensores IR — 5 canais (reservados para seguidor de linha)
#define PIN_IR_EE  26
#define PIN_IR_E   27
#define PIN_IR_C   14
#define PIN_IR_D   12   // ⚠ ver aviso GPIO12
#define PIN_IR_ED  13

// Botão de start
#define PIN_BTN_START 0   // GPIO0 = botão BOOT da placa (ativo LOW)

// LED interno (indicador de estado)
#define PIN_LED_INT 2     // GPIO2 = LED azul onboard do DOIT DevKit V1

// Buzzer ativo
#define PIN_BUZZER 15     // GPIO15 (conforme tabela de pinos no cabeçalho)

// I2C (ex.: OLED 0.96" ou LCD + módulo PCF8574)
#define PIN_SDA    16
#define PIN_SCL    17

// ─────────────────────────────────────────────────────────────
//  SEÇÃO 2 — PWM (detecta Core 2.x ou 3.x automaticamente)
// ─────────────────────────────────────────────────────────────

#define PWM_FREQ   1000
#define PWM_BITS   8

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define LEDC_INIT() \
    ledcAttach(PIN_PWMA, PWM_FREQ, PWM_BITS); \
    ledcAttach(PIN_PWMB, PWM_FREQ, PWM_BITS)
  #define SET_PWMA(v)  ledcWrite(PIN_PWMA, (v))
  #define SET_PWMB(v)  ledcWrite(PIN_PWMB, (v))
#else
  #define LEDC_CH_A    0
  #define LEDC_CH_B    1
  #define LEDC_INIT() \
    ledcSetup(LEDC_CH_A, PWM_FREQ, PWM_BITS); \
    ledcAttachPin(PIN_PWMA, LEDC_CH_A); \
    ledcSetup(LEDC_CH_B, PWM_FREQ, PWM_BITS); \
    ledcAttachPin(PIN_PWMB, LEDC_CH_B)
  #define SET_PWMA(v)  ledcWrite(LEDC_CH_A, (v))
  #define SET_PWMB(v)  ledcWrite(LEDC_CH_B, (v))
#endif

// ─────────────────────────────────────────────────────────────
//  SEÇÃO 3 — PARÂMETROS
// ─────────────────────────────────────────────────────────────

const int DIST_PARADA = 5;  // cm — distância para acionar desvio

const uint8_t VEL_NORMAL = 255;
const uint8_t VEL_CURVA  = 220;

// Tempos do contorno em U (ajuste conforme velocidade e piso)
const int T_LATERAL = 1750;              // ms — avanço lateral (ida e volta)
const int T_FRENTE  = 3100;             // ms — avanço frontal (passa o obstáculo)
const int T_RETORNO = T_LATERAL + 1000;  // ms — último lateral; +500 ms para garantir retorno à linha

// Compensação de inércia nas curvas: motores param COMP_ANGULO° antes do alvo
// para que o deslizamento residual complete o ângulo. Aumente se ainda girar
// demais, reduza se ficar curto.
const float COMP_ANGULO        = 12.0f;
const float COMP_ANGULO_2CURVA =  9.0f;  // 2ª curva (crítica): margem menor → erra levemente para mais

// ─────────────────────────────────────────────────────────────
//  SEÇÃO 4 — TIPOS E VARIÁVEIS GLOBAIS
// ─────────────────────────────────────────────────────────────

MPU6050 mpu;
int16_t gyroZoffset = 0;

enum EstadoRobo { LIVRE, ATENTO, DESVIANDO, RECUANDO };
EstadoRobo estadoAtual = LIVRE;
long distCm = 999;

struct LeituraIR {
  bool ee;
  bool e;
  bool c;
  bool d;
  bool ed;
};

// ─────────────────────────────────────────────────────────────
//  FORWARD DECLARATIONS
// ─────────────────────────────────────────────────────────────
void moverFrente(uint8_t vel);
void moverRe(uint8_t vel);
void girarAngulo(float graus, uint8_t vel, float comp = COMP_ANGULO);
void pararMotores();
static void aguardar(unsigned long ms);
void seguirLinha(uint8_t vel);
void desviarObstaculo();
void calibrarIMU();
long lerDistancia();
LeituraIR lerSensoresIR();
static void _bip(int ms);
void beepInicio();
void beepAviso();
void beepAlerta();
void logStatus();

// ═════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n================================"));
  Serial.println(F("  Robo Desviador de Obstaculos  "));
  Serial.println(F("     ESP32  —  versao 1.0       "));
  Serial.println(F("================================\n"));

  Wire.begin(PIN_SDA, PIN_SCL);

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println(F("[!] MPU-6050 nao encontrado! Verifique a ligacao I2C."));
  } else {
    Serial.println(F("MPU-6050 OK"));
    calibrarIMU();
  }

  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);

  LEDC_INIT();

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  pinMode(PIN_IR_EE, INPUT);
  pinMode(PIN_IR_E,  INPUT);
  pinMode(PIN_IR_C,  INPUT);
  pinMode(PIN_IR_D,  INPUT);
  pinMode(PIN_IR_ED, INPUT);

  pinMode(PIN_LED_INT, OUTPUT);
  digitalWrite(PIN_LED_INT, LOW);

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  pinMode(PIN_BTN_START, INPUT_PULLUP);

  pararMotores();
  delay(200);

  Serial.println(F("► Aguardando botão de start (GPIO0)..."));
  while (digitalRead(PIN_BTN_START) == HIGH) {
    delay(20);
  }
  delay(50); // debounce

  beepInicio();
  delay(500);

  Serial.println(F("► Pronto!\n"));
}

// ═════════════════════════════════════════════════════════════
//  LOOP PRINCIPAL
// ═════════════════════════════════════════════════════════════

void loop() {
  distCm = lerDistancia();
  logStatus();

  if (distCm > 0 && distCm <= DIST_PARADA) {
    desviarObstaculo();
  } else {
    estadoAtual = LIVRE;
    // LED pisca lentamente: LIVRE / seguindo linha
    static unsigned long tLed = 0;
    if (millis() - tLed >= 300) {
      tLed = millis();
      digitalWrite(PIN_LED_INT, !digitalRead(PIN_LED_INT));
    }
    seguirLinha(VEL_NORMAL);
  }
}

// ═════════════════════════════════════════════════════════════
//  MOVIMENTOS DOS MOTORES
// ═════════════════════════════════════════════════════════════

void moverFrente(uint8_t vel) {
  digitalWrite(PIN_AIN1, LOW);  digitalWrite(PIN_AIN2, HIGH); SET_PWMA(vel);
  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW);  SET_PWMB(vel);
}

void moverRe(uint8_t vel) {
  digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW);  SET_PWMA(vel);
  digitalWrite(PIN_BIN1, LOW);  digitalWrite(PIN_BIN2, HIGH); SET_PWMB(vel);
}

// graus > 0 = direita, graus < 0 = esquerda
void girarAngulo(float graus, uint8_t vel, float comp) {
  // Offset local: 10 amostras espaçadas em 15 ms (~150 ms no total) para
  // garantir que qualquer vibração residual do movimento anterior morreu.
  // Isso corrige o drift térmico que causava curvas inconsistentes.
  int16_t ax, ay, az, gx, gy, gz;
  long offSum = 0;
  for (int i = 0; i < 10; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    offSum += gz;
    aguardar(15);
  }
  int16_t offsetAtual = (int16_t)(offSum / 10);

  bool paraDireita = graus > 0;
  // acumulado é local: começa em 0 a cada chamada (sem carry entre curvas)
  float acumulado = 0.0f;
  // Para COMP_ANGULO° antes do alvo — o deslizamento cobre o restante
  float alvo = fmaxf(fabsf(graus) - comp, 0.0f);
  unsigned long tAnt = micros();

  while (acumulado < alvo) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    unsigned long tAgora = micros();
    float dt = (tAgora - tAnt) * 1e-6f;
    tAnt = tAgora;

    // 131 LSB/(°/s) para faixa ±250°/s (padrão MPU-6050)
    acumulado += fabsf((gz - offsetAtual) / 131.0f) * dt;

    if (paraDireita) {
      digitalWrite(PIN_AIN1, LOW);  digitalWrite(PIN_AIN2, HIGH); SET_PWMA(vel);
      digitalWrite(PIN_BIN1, LOW);  digitalWrite(PIN_BIN2, HIGH); SET_PWMB(vel);
    } else {
      digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW);  SET_PWMA(vel);
      digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW);  SET_PWMB(vel);
    }
  }
  pararMotores();
}

void pararMotores() {
  digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, LOW); SET_PWMA(0);
  digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, LOW); SET_PWMB(0);
}

static void aguardar(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms);
}

// ═════════════════════════════════════════════════════════════
//  SEGUIDOR DE LINHA — 5 SENSORES IR
// ═════════════════════════════════════════════════════════════

void seguirLinha(uint8_t vel) {
  LeituraIR ir = lerSensoresIR();

  // Posição ponderada: negativo = linha à esquerda, positivo = à direita
  int pos = 0;
  int cnt = 0;
  if (ir.ee) { pos -= 2; cnt++; }
  if (ir.e)  { pos -= 1; cnt++; }
  if (ir.c)  {           cnt++; }
  if (ir.d)  { pos += 1; cnt++; }
  if (ir.ed) { pos += 2; cnt++; }

  if (cnt == 0) {
    // Linha perdida — mantém reto esperando reencontrar
    moverFrente(vel);
    return;
  }

  const int Kp = 30;
  int correcao = pos * Kp;

  // Motor A = esquerdo, Motor B = direito
  // linha à direita (pos > 0): acelera esquerdo, freia direito → gira à direita
  uint8_t vA = (uint8_t)constrain((int)vel + correcao, 0, 255);
  uint8_t vB = (uint8_t)constrain((int)vel - correcao, 0, 255);

  digitalWrite(PIN_AIN1, LOW);  digitalWrite(PIN_AIN2, HIGH); SET_PWMA(vA);
  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW);  SET_PWMB(vB);
}

// ═════════════════════════════════════════════════════════════
//  DESVIO DE OBSTÁCULO OBR — CONTORNO EM U
//  Sequência: +90° → frente → -90° → frente → -90° → frente → +90°
//  Robô retorna à linha logo após o obstáculo (regra OBR p.27)
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
  //     Settle maior + comp menor: erra levemente para mais se errar
  aguardar(300);  // acomodação extra antes de amostrar offset
  girarAngulo(-90.0f, VEL_CURVA, COMP_ANGULO_2CURVA);
  aguardar(450);

  // 4 — Avança à frente (ultrapassa o comprimento do obstáculo)
  moverFrente(VEL_NORMAL);
  aguardar(T_FRENTE);
  pararMotores();
  aguardar(450);

  // 5 — Curva 90° à esquerda (aponta de volta à linha)
  girarAngulo(-90.0f, VEL_CURVA);
  aguardar(450);

  // 6 — Avança lateral de retorno (alcança a linha; +500 ms extras)
  moverFrente(VEL_NORMAL);
  aguardar(T_RETORNO);
  pararMotores();
  aguardar(450);

  // 7 — Curva 90° à direita (restabelece direção original)
  girarAngulo(90.0f, VEL_CURVA);
  aguardar(450);

  Serial.println(F("  OBR: desvio concluido — retomando seguidor de linha"));
  digitalWrite(PIN_LED_INT, LOW);   // apaga LED ao retomar linha
  estadoAtual = LIVRE;
}

// ═════════════════════════════════════════════════════════════
//  CALIBRAÇÃO IMU (MPU-6050)
// ═════════════════════════════════════════════════════════════

void calibrarIMU() {
  Serial.println(F("  Calibrando IMU — mantenha o robo parado..."));
  long soma = 0;
  const int N = 500;
  for (int i = 0; i < N; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    soma += gz;
    delay(2);
  }
  gyroZoffset = (int16_t)(soma / N);
  Serial.printf("  Offset Z: %d | Calibracao concluida!\n", gyroZoffset);
}

// ═════════════════════════════════════════════════════════════
//  SENSOR ULTRASSÔNICO HC-SR04
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
//  SENSORES IR — 5 CANAIS
// ═════════════════════════════════════════════════════════════

LeituraIR lerSensoresIR() {
  LeituraIR ir;
  ir.ee = !digitalRead(PIN_IR_EE);
  ir.e  = !digitalRead(PIN_IR_E);
  ir.c  = !digitalRead(PIN_IR_C);
  ir.d  = !digitalRead(PIN_IR_D);
  ir.ed = !digitalRead(PIN_IR_ED);
  return ir;
}

// ═════════════════════════════════════════════════════════════
//  BUZZER ATIVO
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

void beepAlerta() {
  _bip(80); delay(60);
  _bip(80);
}

// ═════════════════════════════════════════════════════════════
//  LOG SERIAL
// ═════════════════════════════════════════════════════════════

void logStatus() {
  static unsigned long tUlt = 0;
  if (millis() - tUlt < 250) return;
  tUlt = millis();

  const char* nomes[] = {
    "LIVRE    ", "ATENTO   ", "DESVIANDO", "RECUANDO "
  };

  LeituraIR ir = lerSensoresIR();

  Serial.printf("[%s] Dist: %3ld cm | IR: %c%c%c%c%c\n",
    nomes[estadoAtual], distCm,
    ir.ee ? '#' : '.',
    ir.e  ? '#' : '.',
    ir.c  ? '#' : '.',
    ir.d  ? '#' : '.',
    ir.ed ? '#' : '.'
  );
}

/*
 * ════════════════════════════════════════════════════════════
 *  GUIA DE CALIBRAÇÃO
 * ════════════════════════════════════════════════════════════
 *
 *  1. VELOCIDADES
 *     Execute o robô no chão real. Se girar rápido demais
 *     ou lento demais, ajuste VEL_CURVA e VEL_NORMAL.
 *
 *  2. DISTÂNCIAS DO CONTORNO EM U
 *     T_LATERAL: tempo em ms para percorrer ~20 cm lateralmente.
 *     T_FRENTE:  tempo em ms para percorrer ~30 cm à frente.
 *     T_RETORNO: último avanço lateral (T_LATERAL + 500 ms).
 *     Ajuste conforme a bateria e o piso.
 *
 *  3. COMPENSAÇÃO DE INÉRCIA (COMP_ANGULO)
 *     Os motores param COMP_ANGULO° antes do alvo para que o
 *     deslizamento complete o ângulo. Aumente se ainda girar
 *     demais, reduza se ficar curto.
 *
 *  4. SENTIDO DOS MOTORES
 *     Se ao chamar moverFrente() o robô recuar, inverta
 *     HIGH e LOW nos pinos AIN1/AIN2 (ou BIN1/BIN2).
 *
 *  5. SENSORES IR
 *     Verifique no Monitor Serial se '#' aparece quando o
 *     sensor está sobre uma linha preta. Se aparecer ao
 *     contrário, remova o "!" em lerSensoresIR().
 *
 *  6. GPIO12 (IR Direita)
 *     Se o robô não inicializar (boot loop), desligue o
 *     módulo IR do D12 e tente novamente. Adicione um
 *     resistor de 10 kΩ entre D12 e GND para forçar
 *     nível baixo durante o boot.
 * ════════════════════════════════════════════════════════════
 */
