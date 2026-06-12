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
 *  LÓGICA DE DESVIO (sensor ultrassom fixo, sem servo)
 *  ────────────────────────────────────────────────────────────
 *  1. Avança em velocidade normal
 *  2. Zona amarela  (< DIST_AVISO cm)  → reduz velocidade
 *  3. Zona laranja  (< DIST_PARADA cm) → para + scan lateral
 *  4. Zona vermelha (< DIST_CRITICA cm)→ recua + scan lateral
 *
 *  Scan lateral:
 *    a) Gira ~20° à esquerda  → mede distância
 *    b) Gira ~40° à direita   → mede distância
 *    c) Volta ao centro
 *    d) Vira para o lado mais livre (+ recua/180° se ambos bloqueados)
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

#include <Wire.h>

// ─────────────────────────────────────────────────────────────
//  SEÇÃO 1 — PINAGEM
// ─────────────────────────────────────────────────────────────

// Ponte H
#define PIN_AIN1   21   // Motor A — sentido 1
#define PIN_AIN2   22   // Motor A — sentido 2
#define PIN_PWMA   23   // Motor A — PWM de velocidade
#define PIN_BIN1   25   // Motor B — sentido 1
#define PIN_BIN2   33   // Motor B — sentido 2
#define PIN_PWMB   32   // Motor B — PWM de velocidade
//
// Se usar TB6612FNG com pino STBY, adicione:
//   #define PIN_STBY  XX   ← escolha uma porta livre
// e no setup: pinMode(PIN_STBY, OUTPUT); digitalWrite(PIN_STBY, HIGH);

// Sensor ultrassônico
#define PIN_TRIG   19   // Trigger (disparo)
#define PIN_ECHO   18   // Echo (retorno)

// Sensores IR — 5 canais (reservados para seguidor de linha)
#define PIN_IR_EE  26   // Extrema Esquerda
#define PIN_IR_E   27   // Esquerda
#define PIN_IR_C   14   // Centro
#define PIN_IR_D   12   // Direita       ⚠ ver aviso GPIO12
#define PIN_IR_ED  13   // Extrema Direita

// Buzzer ativo
#define PIN_BUZZER 2   // HIGH = ligado

// I2C (ex.: OLED 0.96" ou LCD + módulo PCF8574)
#define PIN_SDA    16
#define PIN_SCL    17

// ─────────────────────────────────────────────────────────────
//  SEÇÃO 2 — PWM (detecta Core 2.x ou 3.x automaticamente)
// ─────────────────────────────────────────────────────────────

#define PWM_FREQ   1000   // Frequência em Hz
#define PWM_BITS   8      // Resolução: 0–255

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  // ── Core 3.x: API simplificada, pino direto ───────────────
  #define LEDC_INIT() \
    ledcAttach(PIN_PWMA, PWM_FREQ, PWM_BITS); \
    ledcAttach(PIN_PWMB, PWM_FREQ, PWM_BITS)
  #define SET_PWMA(v)  ledcWrite(PIN_PWMA, (v))
  #define SET_PWMB(v)  ledcWrite(PIN_PWMB, (v))
#else
  // ── Core 2.x: canais LEDC explícitos ─────────────────────
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
//  SEÇÃO 3 — PARÂMETROS (ajuste conforme o seu robô)
// ─────────────────────────────────────────────────────────────

// Distâncias em cm
const int DIST_AVISO   = 30;  // Zona amarela  → desacelera
const int DIST_PARADA  = 20;  // Zona laranja  → para e desvia
const int DIST_CRITICA = 10;  // Zona vermelha → recua imediato

// Velocidades 0–255
const uint8_t VEL_NORMAL    = 210;  // Avanço padrão
const uint8_t VEL_REDUZIDA  = 140;  // Zona de atenção
const uint8_t VEL_CURVA     = 185;  // Rotações e scan

// Tempos em ms (calibre girando o robô em superfície real)
const int T_RE     = 450;   // Duração do recuo
const int T_SCAN   = 180;   // Inclinação lateral para medir (~20°)
const int T_DESVIO = 400;   // Curva final de desvio (~50°)

// ─────────────────────────────────────────────────────────────
//  SEÇÃO 4 — VARIÁVEIS GLOBAIS
// ─────────────────────────────────────────────────────────────

enum EstadoRobo { LIVRE, ATENTO, DESVIANDO, RECUANDO };
EstadoRobo estadoAtual = LIVRE;
long distCm = 999;

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

  // I2C com pinos customizados (display opcional)
  Wire.begin(PIN_SDA, PIN_SCL);

  // Pinos de direção da ponte H
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);

  // Inicializa PWM (macro escolhe Core 2.x ou 3.x)
  LEDC_INIT();

  // Sensor ultrassônico
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  // Sensores IR — INPUT sem pull-up interno
  // (os módulos de linha já possuem resistor pull-up externo)
  pinMode(PIN_IR_EE, INPUT);
  pinMode(PIN_IR_E,  INPUT);
  pinMode(PIN_IR_C,  INPUT);
  pinMode(PIN_IR_D,  INPUT);
  pinMode(PIN_IR_ED, INPUT);

  // Buzzer
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  // Garante motores parados na inicialização
  pararMotores();
  delay(200);

  // Sequência de bipes: 3 curtos = sistema OK
  beepInicio();
  delay(500);

  Serial.println(F("► Pronto!\n"));
}

// ═════════════════════════════════════════════════════════════
//  LOOP PRINCIPAL
// ═════════════════════════════════════════════════════════════

void loop() {
  distCm = lerDistancia();
  logStatus();  // Imprime no serial (debug)

  // ── Zona vermelha: muito perto — recua imediatamente ─────
  if (distCm > 0 && distCm <= DIST_CRITICA) {
    estadoAtual = RECUANDO;
    beepAlerta();
    pararMotores();  delay(60);
    moverRe(VEL_NORMAL);
    delay(T_RE);
    pararMotores();  delay(100);
    desviarComScan();

  // ── Zona laranja: obstáculo à frente — para e desvia ─────
  } else if (distCm > 0 && distCm <= DIST_PARADA) {
    if (estadoAtual != DESVIANDO) {
      estadoAtual = DESVIANDO;
      beepAviso();
      pararMotores();  delay(150);
      desviarComScan();
    }

  // ── Zona amarela: obstáculo detectado — desacelera ───────
  } else if (distCm > 0 && distCm <= DIST_AVISO) {
    estadoAtual = ATENTO;
    moverFrente(VEL_REDUZIDA);

  // ── Caminho livre — velocidade normal ────────────────────
  } else {
    estadoAtual = LIVRE;
    moverFrente(VEL_NORMAL);
  }
}

// ═════════════════════════════════════════════════════════════
//  MOVIMENTOS DOS MOTORES
//
//  Assumindo:
//    Motor A (AIN1/AIN2/PWMA) = motor ESQUERDO
//    Motor B (BIN1/BIN2/PWMB) = motor DIREITO
//
//  Se os motores girarem ao contrário, troque:
//    HIGH ↔ LOW nos pinos AIN1/AIN2 (ou nos BIN1/BIN2)
// ═════════════════════════════════════════════════════════════

void moverFrente(uint8_t vel) {
  digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW); SET_PWMA(vel);
  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW); SET_PWMB(vel);
}

void moverRe(uint8_t vel) {
  digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, HIGH); SET_PWMA(vel);
  digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, HIGH); SET_PWMB(vel);
}

// Motor esquerdo (A) avança + Motor direito (B) recua → gira à DIREITA
void girarDireita(uint8_t vel) {
  digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW);  SET_PWMA(vel);
  digitalWrite(PIN_BIN1, LOW);  digitalWrite(PIN_BIN2, HIGH); SET_PWMB(vel);
}

// Motor esquerdo (A) recua + Motor direito (B) avança → gira à ESQUERDA
void girarEsquerda(uint8_t vel) {
  digitalWrite(PIN_AIN1, LOW);  digitalWrite(PIN_AIN2, HIGH); SET_PWMA(vel);
  digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW);  SET_PWMB(vel);
}

void pararMotores() {
  digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, LOW); SET_PWMA(0);
  digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, LOW); SET_PWMB(0);
}

// ═════════════════════════════════════════════════════════════
//  DESVIO COM SCAN LATERAL
//
//  Como o sensor ultrassom é fixo (sem servo), o robô
//  "olha" para cada lado rotacionando levemente sobre si mesmo:
//
//    ① Gira ~20° à esquerda      → mede dEsq
//    ② Gira ~40° à direita       → mede dDir  (passa pelo centro)
//    ③ Volta ~20° ao centro
//    ④ Compara e vira para o lado mais livre
//    ⑤ Se ambos bloqueados → recua mais + rotação 180°
// ═════════════════════════════════════════════════════════════

void desviarComScan() {
  estadoAtual = DESVIANDO;

  // ① Olha à esquerda (T_SCAN ms ≈ 20°)
  girarEsquerda(VEL_CURVA);
  delay(T_SCAN);
  pararMotores(); delay(80);
  long dEsq = lerDistancia();

  // ② Varre para a direita (2× T_SCAN: vai de -20° a +20°)
  girarDireita(VEL_CURVA);
  delay(T_SCAN * 2);
  pararMotores(); delay(80);
  long dDir = lerDistancia();

  // ③ Volta ao centro (-20° → 0°)
  girarEsquerda(VEL_CURVA);
  delay(T_SCAN);
  pararMotores(); delay(100);

  Serial.printf("  Scan: Esq=%ld cm | Dir=%ld cm\n", dEsq, dDir);

  // ④ Ambos os lados bloqueados? Escape de beco
  if (dEsq <= DIST_PARADA && dDir <= DIST_PARADA) {
    Serial.println(F("  [!] Ambos os lados bloqueados! Recuo extra + 180 graus"));
    beepAlerta();
    moverRe(VEL_NORMAL);
    delay(T_RE * 2);
    pararMotores(); delay(100);
    girarDireita(VEL_CURVA);
    delay(T_DESVIO * 2);        // ~180°
    pararMotores(); delay(100);
    estadoAtual = LIVRE;
    return;
  }

  // ⑤ Vira para o lado com mais espaço
  if (dEsq >= dDir) {
    Serial.println(F("  <- Desviando pela ESQUERDA"));
    girarEsquerda(VEL_CURVA);
  } else {
    Serial.println(F("  -> Desviando pela DIREITA"));
    girarDireita(VEL_CURVA);
  }
  delay(T_DESVIO);
  pararMotores(); delay(150);

  estadoAtual = LIVRE;
}

// ═════════════════════════════════════════════════════════════
//  SENSOR ULTRASSÔNICO HC-SR04
//
//  Retorna distância em cm.
//  Retorna 999 se não houver eco (caminho livre > ~5 m)
//  ou se o timeout de 30 ms for atingido.
// ═════════════════════════════════════════════════════════════

long lerDistancia() {
  // Pulso de disparo
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // Timeout de 30 000 µs (~500 cm) para não travar o loop
  long duracao = pulseIn(PIN_ECHO, HIGH, 30000UL);

  if (duracao == 0) return 999;    // Sem eco = caminho livre

  return duracao / 58L;            // µs ÷ 58 ≈ centímetros
}

// ═════════════════════════════════════════════════════════════
//  SENSORES IR — 5 CANAIS
//  Reservados para o modo seguidor de linha (integração futura)
//
//  A lógica abaixo inverte a leitura porque a maioria dos
//  módulos TCRT5000 emite LOW ao detectar linha preta.
//  Se o seu módulo emitir HIGH, remova o "!" de cada linha.
//
//  TODO: criar função modoSeguidorLinha() usando estes sensores
// ═════════════════════════════════════════════════════════════

struct LeituraIR {
  bool ee;   // Extrema Esquerda
  bool e;    // Esquerda
  bool c;    // Centro
  bool d;    // Direita
  bool ed;   // Extrema Direita
};

LeituraIR lerSensoresIR() {
  LeituraIR ir;
  ir.ee = !digitalRead(PIN_IR_EE);   // true = detectou linha preta
  ir.e  = !digitalRead(PIN_IR_E);
  ir.c  = !digitalRead(PIN_IR_C);
  ir.d  = !digitalRead(PIN_IR_D);
  ir.ed = !digitalRead(PIN_IR_ED);
  return ir;
}

// ═════════════════════════════════════════════════════════════
//  BUZZER ATIVO
//  (buzzer passivo: substitua digitalWrite por tone/noTone)
// ═════════════════════════════════════════════════════════════

// Função auxiliar interna
static void _bip(int ms) {
  digitalWrite(PIN_BUZZER, HIGH);
  delay(ms);
  digitalWrite(PIN_BUZZER, LOW);
}

// 3 bipes curtos — sistema inicializado
void beepInicio() {
  for (int i = 0; i < 3; i++) {
    _bip(100);
    delay(100);
  }
}

// 1 bipe médio — obstáculo detectado
void beepAviso() {
  _bip(220);
}

// 2 bipes rápidos — obstáculo crítico
void beepAlerta() {
  _bip(80); delay(60);
  _bip(80);
}

// ═════════════════════════════════════════════════════════════
//  LOG SERIAL (debug — abre Monitor Serial a 115200 bps)
//
//  Formato: [ESTADO   ] Dist: XXX cm | IR: #.#.#
//    '#' = sensor ativo (detectou linha)
//    '.' = sensor inativo
// ═════════════════════════════════════════════════════════════

void logStatus() {
  static unsigned long tUlt = 0;
  if (millis() - tUlt < 250) return;   // ≤ 4 atualizações/s
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
 *  2. ÂNGULO DE SCAN / DESVIO
 *     T_SCAN  controla os ~20° de inclinação lateral.
 *     T_DESVIO controla os ~50° de curva final.
 *     Teste sobre a superfície real e ajuste até ficar
 *     satisfatório (a bateria influencia estes valores).
 *
 *  3. SENTIDO DOS MOTORES
 *     Se ao chamar moverFrente() o robô recuar, inverta
 *     HIGH e LOW nos pinos AIN1/AIN2 (ou BIN1/BIN2).
 *     Se ao chamar girarDireita() o robô girar à esquerda,
 *     troque os blocos de AIN e BIN dentro das funções.
 *
 *  4. SENSORES IR
 *     Verifique no Monitor Serial se '#' aparece quando o
 *     sensor está sobre uma linha preta. Se aparecer ao
 *     contrário, remova o "!" em lerSensoresIR().
 *
 *  5. GPIO12 (IR Direita)
 *     Se o robô não inicializar (boot loop), desligue o
 *     módulo IR do D12 e tente novamente. Adicione um
 *     resistor de 10 kΩ entre D12 e GND para forçar
 *     nível baixo durante o boot.
 *
 * ════════════════════════════════════════════════════════════
 *  PRÓXIMOS PASSOS — SEGUIDOR DE LINHA
 * ════════════════════════════════════════════════════════════
 *
 *  Para integrar o modo seguidor de linha futuramente:
 *
 *  1. Crie uma variável global:
 *       enum ModoRobo { MODO_OBSTACULO, MODO_LINHA };
 *       ModoRobo modoAtual = MODO_OBSTACULO;
 *
 *  2. No loop(), escolha o modo:
 *       if (modoAtual == MODO_LINHA) modoSeguidorLinha();
 *       else modoDesvioObstaculo();
 *
 *  3. Implemente modoSeguidorLinha() usando lerSensoresIR()
 *     com lógica PID ou proporcional simples.
 *
 *  4. Use o sensor ultrassom dentro do seguidor para
 *     pausar automaticamente ao encontrar um obstáculo.
 * ════════════════════════════════════════════════════════════
 */
