/*
 * ════════════════════════════════════════════════════════════════════════
 *  >>> VERSÃO ARDUINO IDE  —  COMO COMPILAR E ENVIAR  <<<
 * ════════════════════════════════════════════════════════════════════════
 *
 *  Este é o MESMO programa da versão PlatformIO, adaptado para a Arduino IDE.
 *  A pasta precisa se chamar "SeguidorPontoVerde" e o arquivo
 *  "SeguidorPontoVerde.ino" (a Arduino IDE exige pasta = nome do .ino).
 *
 *  1) SUPORTE A PLACA ESP32 (Boards Manager)
 *     - Arquivo > Preferencias > "URLs Adicionais de Gerenciadores de Placas":
 *         https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
 *     - Ferramentas > Placa > Gerenciador de Placas... > procure "esp32"
 *       e instale "esp32 by Espressif Systems" (3.x recomendado; compila no 2.x tambem).
 *     - Selecione: Ferramentas > Placa > ESP32 Arduino > "DOIT ESP32 DEVKIT V1".
 *
 *  2) BIBLIOTECAS (Ferramentas > Gerenciar Bibliotecas / Library Manager)
 *     Instale estas tres (as outras ja vem no core do ESP32):
 *       - "QTRSensors"         by Pololu           (sensores de linha)
 *       - "MPU6050"            by Electronic Cats  (giroscopio)
 *       - "Adafruit TCS34725"  by Adafruit         (sensores de cor)
 *           -> aceite instalar a dependencia "Adafruit BusIO" quando pedir.
 *     Ja incluidas no core do ESP32 (NAO precisa instalar):
 *       - Wire        (I2C - usa dois barramentos: Wire e Wire1)
 *       - Preferences (NVS - salva a calibracao na flash interna)
 *
 *  3) CONFIGURACAO DA IDE
 *     - Velocidade do Monitor Serial: 115200 baud.
 *     - Upload Speed: 921600 (use 115200 se der erro de gravacao).
 *     - Partition Scheme: padrao ("Default 4MB with spiffs") basta.
 *
 *  4) PRIMEIRO USO
 *     - Ao ligar: calibra o QTR (mova sobre a linha) e o giroscopio (parado
 *       nos 3 bips). Calibracao de QTR e cor ficam salvas na flash (NVS).
 *     - Calibre as cores 1x pelo Monitor Serial: coloque os DOIS sensores
 *       sobre cada cor e envie 'p' (preto), 'b' (branco), 'g' (verde).
 *       Confira com 'v'. Lista completa de comandos no cabecalho abaixo.
 *
 *  Comportamento IDENTICO a versao PlatformIO; muda so a instalacao.
 *  Pasta PlatformIO equivalente: ../../seguidor_ponto_verde_esp32/
 * ════════════════════════════════════════════════════════════════════════
 */

/*
 * ════════════════════════════════════════════════════════════════
 *  SEGUIDOR DE LINHA + PONTO VERDE — OBR 2026 Nível 2
 *  ESP32 / Arduino IDE
 *
 *  Baseado em:
 *   • seguidor_de_linha_PID_esp32  (QTRSensors analógico + PID)
 *   • RoboDesvio                   (MPU6050, LEDC, controle direto)
 *
 *  ┌─────────────────────────────────────────────────────────┐
 *  │                    MAPA DE PINOS                        │
 *  ├──────────────────────┬──────────────────────────────────┤
 *  │ PONTE H              │ AIN1=21 AIN2=22 PWMA=23          │
 *  │                      │ BIN1=25 BIN2=33 PWMB=32          │
 *  ├──────────────────────┼──────────────────────────────────┤
 *  │ QTR Analógico (5×)   │ EE=26  E=27  C=14  D=12(*) ED=13 │
 *  ├──────────────────────┼──────────────────────────────────┤
 *  │ TCS34725 ESQUERDO    │ Wire  SDA=16 SCL=17 (+ MPU6050)  │
 *  │ TCS34725 DIREITO     │ Wire1 SDA=4  SCL=5              │
 *  ├──────────────────────┼──────────────────────────────────┤
 *  │ Buzzer / LED         │ D15 / D2                        │
 *  └──────────────────────┴──────────────────────────────────┘
 *
 *  (*) GPIO12 = strapping pin (tensão de flash).
 *      O sensor IR normalmente mantém LOW no boot.
 *      Se houver boot loop, adicione 10 kΩ entre D12 e GND.
 *
 *  LÓGICA DE PONTO VERDE (OBR 2026 — marcador 2,5×2,5 cm):
 *   1. TCS detecta verde (N_HIST leituras consecutivas) → latch
 *   2. Avança devagar até TODOS os QTR ficarem pretos (interseção)
 *      OU esgotar T_AVANCO_MS → falso alarme → segue reto
 *   3. Interseção confirmada → volta T_VOLTA_MS (centraliza no crux)
 *   4. girarAngulo() via MPU6050:
 *        verde ESQ  → −90° (esquerda)
 *        verde DIR  → +90° (direita)
 *        ambos      →  180° (beco sem saída)
 *   5. Reencontra a linha avançando devagar → retoma PID
 *
 *  COMANDOS SERIAL (115200 baud):
 *   p  = captura RGB atual como PRETO     (sensor sobre linha preta)
 *   b  = captura RGB atual como BRANCO    (sensor sobre piso branco)
 *   g  = captura RGB atual como VERDE     (sensor sobre marcador verde)
 *   v  = imprime RGB lido + classificação (verde / não-verde)
 *   k  = recalibra o QTR ao vivo e salva
 *   z  = apaga toda a calibração da NVS
 *   c  = imprime calibração atual
 *   i  = imprime estado atual (debug)
 *
 *  CLASSIFICAÇÃO DE COR: vizinho mais próximo em CROMATICIDADE
 *   (RGB normalizado por R+G+B — independe do brilho). A cor lida é
 *   comparada com as 3 referências do sensor (preto/branco/verde) e
 *   vence a mais próxima; é verde quando a referência VERDE vence.
 *   Normalizar evita a falsa leitura de verde na transição branco→preto
 *   (onde os valores brutos passam pelo "médio" parecido com o verde).
 * ════════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <Wire.h>
#include <QTRSensors.h>
#include <MPU6050.h>
#include <Preferences.h>          // NVS — memória interna (calibração persistente)
#include "Adafruit_TCS34725.h"

// ─────────────────────────────────────────────────────────────
//  SEÇÃO 1 — PINAGEM
// ─────────────────────────────────────────────────────────────
#define PIN_AIN1   21
#define PIN_AIN2   22
#define PIN_PWMA   23
#define PIN_BIN1   25
#define PIN_BIN2   33
#define PIN_PWMB   32

#define PIN_BUZZER 15
#define PIN_LED     2

#define PIN_SDA    16   // I2C0: MPU6050 (0x68) + TCS ESQUERDO (0x29)
#define PIN_SCL    17
#define PIN_SDA2    4   // I2C1: TCS DIREITO (0x29) — barramento separado
#define PIN_SCL2    5

// ─────────────────────────────────────────────────────────────
//  SEÇÃO 2 — LEDC (Core 2.x / 3.x — igual ao RoboDesvio)
// ─────────────────────────────────────────────────────────────
#define PWM_FREQ 1000
#define PWM_BITS    8

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define LEDC_INIT() \
    ledcAttach(PIN_PWMA, PWM_FREQ, PWM_BITS); \
    ledcAttach(PIN_PWMB, PWM_FREQ, PWM_BITS)
  #define SET_PWMA(v)  ledcWrite(PIN_PWMA, (v))
  #define SET_PWMB(v)  ledcWrite(PIN_PWMB, (v))
#else
  #define LEDC_CH_A 0
  #define LEDC_CH_B 1
  #define LEDC_INIT() \
    ledcSetup(LEDC_CH_A, PWM_FREQ, PWM_BITS); \
    ledcAttachPin(PIN_PWMA, LEDC_CH_A); \
    ledcSetup(LEDC_CH_B, PWM_FREQ, PWM_BITS); \
    ledcAttachPin(PIN_PWMB, LEDC_CH_B)
  #define SET_PWMA(v)  ledcWrite(LEDC_CH_A, (v))
  #define SET_PWMB(v)  ledcWrite(LEDC_CH_B, (v))
#endif

// ─────────────────────────────────────────────────────────────
//  SEÇÃO 3 — PARÂMETROS TUNÁVEIS
// ─────────────────────────────────────────────────────────────

// --- PID de linha (mesmos nomes do seguidor_de_linha_PID_esp32) ---
float Kp = 0.30f;   // proporcional — aumente se oscilar pouco, diminua se oscilar demais
float Ki = 0.002f;  // integral     — deixe 0 até Kp/Kd estarem bons
float Kd = 1.20f;   // derivativo   — aumente para amortizar oscilação
int   lfspeed = 200; // velocidade base (0–255)

// --- Thresholds QTR pós-calibração (0–1000 por sensor) ---
const uint16_t QTR_PRETO  = 800; // sensor "em preto"  quando valor >= 800
const uint16_t QTR_BRANCO =  50; // sensor "em branco" quando valor <=  50

// --- Detecção de interseção ---
// OBR 2026 p.31: interseções podem ter 3 ou 4 ramificações.
// Quando o robô aborda pelo CABO do T: os 5 sensores cruzam a barra
//   perpendicular → todos ficam pretos (QTR_MIN_PRETOS = 5 funcionaria).
// Quando aborda pelo BRAÇO do T: os sensores são paralelos à barra;
//   apenas o ponto de junção com o ramo lateral (o "caule") fica preto
//   para 3-4 sensores — nunca todos os 5 simultaneamente.
// Solução: confirmar interseção quando >= QTR_MIN_PRETOS sensores pretos.
// Valor 3 captura ambos os casos sem disparar em curvas normais (1-2 pretos).
const uint8_t QTR_MIN_PRETOS = 3;

// --- Verde (TCS34725) ---
const uint8_t  N_HIST       = 3;    // leituras consecutivas p/ confirmar verde
const int      VEL_AVANCO   = 80;   // velocidade ao verificar interseção
const uint32_t T_AVANCO_MS  = 400;  // tempo máximo avançando; após → falso alarme
const int      VEL_VOLTA    = 80;   // velocidade de ré ao centralizar
const uint32_t T_VOLTA_MS   = 180;  // tempo de ré para centralizar no cruzamento
const int      VEL_REENCONTR= 60;   // velocidade ao buscar linha após giro

// --- Giro (MPU6050 — idêntico ao RoboDesvio) ---
const int   VEL_CURVA    = 180;  // velocidade das rodas durante o giro
const float COMP_ANGULO  = 12.0f; // para ANTES do alvo (inércia); tunar igual ao RoboDesvio

// --- Timeout reencontrar linha ---
const uint32_t T_REENCONTR_MAX = 2000;

// --- Calibração de cor TCS (RGB) — calibrar via Serial p/b/g ---
// Guardamos a referência RGB de cada cor e classificamos pela menor
// distância em cromaticidade (vizinho mais próximo). São "3 valores por cor"
// (R, G, B brutos do TCS34725).
//
// IMPORTANTE: os dois TCS têm escalas de leitura diferentes, então cada
// sensor tem seu PRÓPRIO conjunto de referências (sufixo E = esquerdo,
// D = direito). Sem isso, o preto de um sensor pode cair perto do verde
// calibrado no outro e ser classificado como verde.
struct RefCor { uint16_t r, g, b; };
//                          R     G     B
RefCor refPretoE  = {   30,   30,   30 };  // ESQ: linha/piso preto
RefCor refBrancoE = { 2000, 2000, 2000 };  // ESQ: piso branco
RefCor refVerdeE  = {  200,  600,  200 };  // ESQ: marcador verde
RefCor refPretoD  = {   30,   30,   30 };  // DIR: linha/piso preto
RefCor refBrancoD = { 2000, 2000, 2000 };  // DIR: piso branco
RefCor refVerdeD  = {  200,  600,  200 };  // DIR: marcador verde

// ─────────────────────────────────────────────────────────────
//  SEÇÃO 4 — OBJETOS GLOBAIS
// ─────────────────────────────────────────────────────────────
QTRSensors qtr;
const uint8_t NUM_SENS = 5;
uint16_t sensorValues[NUM_SENS];

MPU6050 mpu;

Adafruit_TCS34725 tcsE(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_TCS34725 tcsD(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
bool tcsEOk = false, tcsDOk = false;

// NVS — calibração persistente (QTR + cor). MPU NÃO é salvo (recalibra no boot).
Preferences prefs;
const char *NVS_NS = "calib";      // namespace na flash interna

// ─────────────────────────────────────────────────────────────
//  SEÇÃO 5 — VARIÁVEIS DE ESTADO
// ─────────────────────────────────────────────────────────────

// PID
int  previousError = 0;
long I_term        = 0;

// Verde
uint8_t contE = 0, contD = 0;
bool    latchE = false, latchD = false;

// FSM
enum Estado { SEGUINDO, VERDE_AVANCANDO, VERDE_VIRANDO, REENCONTRANDO };
Estado        estado  = SEGUINDO;
unsigned long tEstado = 0;

// ─────────────────────────────────────────────────────────────
//  SEÇÃO 6 — PROTÓTIPOS
// ─────────────────────────────────────────────────────────────
void   calibrarQTR();
void   calibrarQTRAoVivo();
void   aplicarCalibracaoQTR(const uint16_t *mins, const uint16_t *maxs);
bool   carregarCalibracaoQTR();
void   salvarCalibracaoQTR();
void   carregarCalibracaoCor();
void   salvarCalibracaoCor();
void   limparCalibracaoNVS();
void   calibrarIMU();
void   moverFrente(int vel);
void   moverRe(int vel);
void   pararMotores();
void   motorDrive(int esq, int dir);
void   girarAngulo(float graus, int vel, float comp = COMP_ANGULO);
static void aguardar(unsigned long ms);
void   pidLinha();
int    contarPretosSV();
bool   intersecaoDetectada();
struct LeituraCor { uint16_t r=0, g=0, b=0, c=0; };
LeituraCor lerCor(Adafruit_TCS34725 &tcs);
bool   ehVerde(const LeituraCor &L, const RefCor &rPreto, const RefCor &rBranco, const RefCor &rVerde);
void   fsmUpdate();
void   tratarSerial();
void   bip(int n, int ms_on=100, int ms_off=80);
void   imprimirTelemetria();

// ════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println(F("\n=== SEGUIDOR + PONTO VERDE OBR 2026 ==="));

    // Buzzer e LED
    pinMode(PIN_BUZZER, OUTPUT); digitalWrite(PIN_BUZZER, LOW);
    pinMode(PIN_LED,    OUTPUT); digitalWrite(PIN_LED,    LOW);

    // Motores
    pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
    LEDC_INIT();
    pararMotores();

    // I2C
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);
    Wire1.begin(PIN_SDA2, PIN_SCL2);
    Wire1.setClock(400000);

    // MPU6050 (mesmo barramento do TCS ESQ — endereços diferentes, OK)
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println(F("[!] MPU6050 nao encontrado!"));
    } else {
        Serial.println(F("[OK] MPU6050"));
        calibrarIMU();
    }

    // TCS34725
    tcsEOk = tcsE.begin(0x29, &Wire);
    tcsDOk = tcsD.begin(0x29, &Wire1);
    if (tcsEOk) { tcsE.setInterrupt(false); Serial.println(F("[OK] TCS ESQUERDO")); }
    else          Serial.println(F("[--] TCS ESQUERDO ausente"));
    if (tcsDOk) { tcsD.setInterrupt(false); Serial.println(F("[OK] TCS DIREITO")); }
    else          Serial.println(F("[--] TCS DIREITO ausente"));

    // Calibração de cor: carrega da NVS (se já calibrada antes)
    carregarCalibracaoCor();

    // QTR — carrega da NVS se existir; senão calibra (400 leituras) e salva
    calibrarQTR();

    bip(3);
    Serial.println(F("Pronto! Comandos: p=preto b=branco g=verde (capturam RGB)  v=ler RGB"));
    Serial.println(F("                  k=recalibrar QTR  z=limpar NVS  c=config i=debug"));
}

// ════════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════════
void loop() {
    tratarSerial();
    fsmUpdate();
    imprimirTelemetria();
}

// ════════════════════════════════════════════════════════════════
//  CALIBRAÇÃO
// ════════════════════════════════════════════════════════════════
void calibrarQTR() {
    // QTR analógico nos mesmos pinos do seguidor_de_linha_PID_esp32
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){26, 27, 14, 12, 13}, NUM_SENS);

    // Se já houver calibração salva na flash, usa ela e pula a calibração ao vivo.
    if (carregarCalibracaoQTR()) {
        Serial.println(F("[OK] Calibracao QTR carregada da NVS (envie 'k' p/ recalibrar)."));
        bip(1);
        return;
    }
    // Primeira vez (ou NVS limpa): calibra ao vivo e salva.
    calibrarQTRAoVivo();
}

// Rotina de calibração ao vivo (400 leituras) — usada na 1ª vez e no comando 'k'.
void calibrarQTRAoVivo() {
    Serial.println(F("Calibrando QTR (400 leituras)..."));
    Serial.println(F(">>> MOVA O ROBO SOBRE A LINHA PRETA AGORA <<<"));
    digitalWrite(PIN_LED, HIGH);

    for (uint16_t i = 0; i < 400; i++) {
        qtr.calibrate();
        delay(2);
    }

    digitalWrite(PIN_LED, LOW);
    Serial.println(F("Calibracao QTR concluida."));
    salvarCalibracaoQTR();
    bip(2);
}

// ── Persistência QTR (NVS) ──────────────────────────────────────
// Aplica min/max lidos da flash diretamente no objeto QTRSensors.
void aplicarCalibracaoQTR(const uint16_t *mins, const uint16_t *maxs) {
    // Uma chamada a calibrate() força a alocação interna dos arrays
    // calibrationOn.minimum/maximum (e marca initialized=true).
    qtr.calibrate();
    for (uint8_t i = 0; i < NUM_SENS; i++) {
        qtr.calibrationOn.minimum[i] = mins[i];
        qtr.calibrationOn.maximum[i] = maxs[i];
    }
}

bool carregarCalibracaoQTR() {
    prefs.begin(NVS_NS, true);                       // somente leitura
    bool ok = prefs.getBool("qtr_ok", false);
    if (ok) {
        uint16_t mins[NUM_SENS], maxs[NUM_SENS];
        prefs.getBytes("qtr_min", mins, sizeof(mins));
        prefs.getBytes("qtr_max", maxs, sizeof(maxs));
        prefs.end();
        aplicarCalibracaoQTR(mins, maxs);
        return true;
    }
    prefs.end();
    return false;
}

void salvarCalibracaoQTR() {
    if (!qtr.calibrationOn.initialized) return;
    prefs.begin(NVS_NS, false);                      // leitura/escrita
    prefs.putBytes("qtr_min", qtr.calibrationOn.minimum, sizeof(uint16_t) * NUM_SENS);
    prefs.putBytes("qtr_max", qtr.calibrationOn.maximum, sizeof(uint16_t) * NUM_SENS);
    prefs.putBool("qtr_ok", true);
    prefs.end();
    Serial.println(F("[NVS] Calibracao QTR salva."));
}

// ── Persistência de cor (NVS) ───────────────────────────────────
void carregarCalibracaoCor() {
    prefs.begin(NVS_NS, true);
    // "cor_ok2" = esquema por sensor (6 referências). Ignora dados antigos.
    if (prefs.getBool("cor_ok2", false)) {
        prefs.getBytes("refPE", &refPretoE,  sizeof(refPretoE));
        prefs.getBytes("refBE", &refBrancoE, sizeof(refBrancoE));
        prefs.getBytes("refVE", &refVerdeE,  sizeof(refVerdeE));
        prefs.getBytes("refPD", &refPretoD,  sizeof(refPretoD));
        prefs.getBytes("refBD", &refBrancoD, sizeof(refBrancoD));
        prefs.getBytes("refVD", &refVerdeD,  sizeof(refVerdeD));
        Serial.println(F("[OK] Calibracao de cor (RGB por sensor) carregada da NVS."));
    } else {
        Serial.println(F("[--] Sem calibracao de cor — calibre com p/b/g (usando padroes)."));
    }
    prefs.end();
}

void salvarCalibracaoCor() {
    prefs.begin(NVS_NS, false);
    prefs.putBytes("refPE", &refPretoE,  sizeof(refPretoE));
    prefs.putBytes("refBE", &refBrancoE, sizeof(refBrancoE));
    prefs.putBytes("refVE", &refVerdeE,  sizeof(refVerdeE));
    prefs.putBytes("refPD", &refPretoD,  sizeof(refPretoD));
    prefs.putBytes("refBD", &refBrancoD, sizeof(refBrancoD));
    prefs.putBytes("refVD", &refVerdeD,  sizeof(refVerdeD));
    prefs.putBool ("cor_ok2", true);
    prefs.end();
    Serial.println(F("[NVS] Calibracao de cor (RGB por sensor) salva."));
}

// Apaga toda a calibração da flash (QTR + cor). Recalibra no próximo boot.
void limparCalibracaoNVS() {
    prefs.begin(NVS_NS, false);
    prefs.clear();
    prefs.end();
    Serial.println(F("[NVS] Calibracao apagada. Reinicie para recalibrar."));
}

void calibrarIMU() {
    // Igual ao RoboDesvio: 500 amostras para calcular offset do gyro Z
    Serial.println(F("Calibrando IMU — mantenha o robo parado..."));
    long soma = 0;
    const int N = 500;
    for (int i = 0; i < N; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        soma += gz;
        delay(2);
    }
    // O offset é armazenado internamente pelo objeto MPU6050
    // Para girarAngulo(), o offset é recalculado localmente a cada chamada (como no RoboDesvio)
    Serial.println(F("IMU calibrado."));
}

// ════════════════════════════════════════════════════════════════
//  MOTORES (idênticos ao RoboDesvio)
// ════════════════════════════════════════════════════════════════
void moverFrente(int vel) {
    vel = constrain(vel, 0, 255);
    digitalWrite(PIN_AIN1, LOW);  digitalWrite(PIN_AIN2, HIGH); SET_PWMA(vel);
    digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW);  SET_PWMB(vel);
}

void moverRe(int vel) {
    vel = constrain(vel, 0, 255);
    digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW);  SET_PWMA(vel);
    digitalWrite(PIN_BIN1, LOW);  digitalWrite(PIN_BIN2, HIGH); SET_PWMB(vel);
}

void pararMotores() {
    digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, LOW); SET_PWMA(0);
    digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, LOW); SET_PWMB(0);
}

// motorDrive para o PID: esq>0=frente, dir>0=frente
void motorDrive(int esq, int dir) {
    esq = constrain(esq, 0, 255);  // clamped ≥ 0 (sem ré durante PID)
    dir = constrain(dir, 0, 255);
    digitalWrite(PIN_AIN1, LOW);  digitalWrite(PIN_AIN2, HIGH); SET_PWMA(esq);
    digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW);  SET_PWMB(dir);
}

// Giro controlado pelo gyro Z — IDÊNTICO ao RoboDesvio
// graus > 0 = direita | graus < 0 = esquerda
void girarAngulo(float graus, int vel, float comp) {
    int16_t ax, ay, az, gx, gy, gz;
    // Recalcula offset local (10 amostras × 15 ms = 150 ms)
    long offSum = 0;
    for (int i = 0; i < 10; i++) {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        offSum += gz;
        aguardar(15);
    }
    int16_t offsetLocal = (int16_t)(offSum / 10);

    bool paraDireita = graus > 0;
    float acumulado  = 0.0f;
    float alvo       = fmaxf(fabsf(graus) - comp, 0.0f);
    unsigned long tAnt = micros();

    while (acumulado < alvo) {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        unsigned long tAgora = micros();
        float dt = (tAgora - tAnt) * 1e-6f;
        tAnt = tAgora;
        acumulado += fabsf((gz - offsetLocal) / 131.0f) * dt;

        if (paraDireita) {
            // Esq frente, Dir ré → gira para direita
            digitalWrite(PIN_AIN1, LOW);  digitalWrite(PIN_AIN2, HIGH); SET_PWMA(vel);
            digitalWrite(PIN_BIN1, LOW);  digitalWrite(PIN_BIN2, HIGH); SET_PWMB(vel);
        } else {
            // Esq ré, Dir frente → gira para esquerda
            digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW);  SET_PWMA(vel);
            digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW);  SET_PWMB(vel);
        }
    }
    pararMotores();
}

static void aguardar(unsigned long ms) {
    unsigned long t0 = millis();
    while (millis() - t0 < ms);
}

// ════════════════════════════════════════════════════════════════
//  PID DE LINHA (baseado no seguidor_de_linha_PID_esp32)
//  DESATIVADO — mantido para reintegração futura do seguidor de linha.
//  Para reativar: chame pidLinha() no estado SEGUINDO da fsmUpdate().
// ════════════════════════════════════════════════════════════════
void pidLinha() {
    uint16_t position = qtr.readLineBlack(sensorValues);
    int error = 2000 - (int)position;  // centro = 2000; + = linha à esquerda

    int P = error;
    I_term += error;
    I_term = constrain(I_term, -5000L, 5000L);  // anti-windup
    int D = error - previousError;

    float PIDvalue = Kp * P + Ki * I_term + Kd * D;
    previousError = error;

    int lsp = lfspeed - (int)PIDvalue;
    int rsp = lfspeed + (int)PIDvalue;

    // Clamp [0, 255] — igual ao seguidor (sem ré durante seguimento normal)
    lsp = constrain(lsp, 0, 255);
    rsp = constrain(rsp, 0, 255);

    motorDrive(lsp, rsp);
}

// ════════════════════════════════════════════════════════════════
//  QTR — AUXILIARES
// ════════════════════════════════════════════════════════════════

// Conta quantos sensores estão sobre preto no array sensorValues atual
int contarPretosSV() {
    int n = 0;
    for (int i = 0; i < NUM_SENS; i++)
        if (sensorValues[i] >= QTR_PRETO) n++;
    return n;
}

// Retorna true quando há sensores pretos suficientes para indicar interseção.
// Cobre tanto o T abordado pelo cabo (5/5 pretos) quanto pelo braço (3-4/5):
//   +  interseção de 4 ramos: todos 5 ficam pretos               → 5 >= 3 ✓
//   T  abordado pelo cabo:    barra perpendicular cobre 5         → 5 >= 3 ✓
//   T  abordado pelo braço:   apenas o ponto de junção é extra    → 3-4 >= 3 ✓
//   curva normal 90°:         máximo 2 sensores simultâneos       → 2 < 3  ✗ (ok)
//   linha reta:               1-2 sensores                        → 1-2 < 3 ✗ (ok)
bool intersecaoDetectada() {
    return contarPretosSV() >= QTR_MIN_PRETOS;
}

// ════════════════════════════════════════════════════════════════
//  TCS34725 — LEITURA E CLASSIFICAÇÃO
// ════════════════════════════════════════════════════════════════
LeituraCor lerCor(Adafruit_TCS34725 &tcs) {
    LeituraCor L;
    // Lê diretamente os canais RGB brutos (+ clear) do TCS34725.
    tcs.getRawData(&L.r, &L.g, &L.b, &L.c);
    return L;
}

// Distância² em CROMATICIDADE (RGB normalizado por R+G+B), não em RGB bruto.
// Por quê: no RGB bruto o verde tem valores MÉDIOS, e na transição
// branco→preto os canais passam por valores médios também — então a
// transição fica perto do verde e gera leitura falsa. Normalizando,
// o brilho some: branco, preto e toda a transição ficam EQUILIBRADOS
// (~1/3 em cada canal), enquanto o verde tem G dominante. Assim só o
// verde de verdade se aproxima da referência verde.
static float distCromSq(const LeituraCor &L, const RefCor &R) {
    float sL = (float)L.r + L.g + L.b;
    float sR = (float)R.r + R.g + R.b;
    if (sL <= 0.0f || sR <= 0.0f) return 1e9f;     // sem luz: indefinido
    float dr = (float)L.r / sL - (float)R.r / sR;
    float dg = (float)L.g / sL - (float)R.g / sR;
    float db = (float)L.b / sL - (float)R.b / sR;
    return dr * dr + dg * dg + db * db;
}

// Classifica a leitura pelo vizinho mais próximo (em cromaticidade) entre
// as 3 referências DAQUELE sensor. É verde quando a referência VERDE é a
// mais próxima das três.
bool ehVerde(const LeituraCor &L, const RefCor &rPreto, const RefCor &rBranco, const RefCor &rVerde) {
    if (L.c == 0) return false;
    float dPreto  = distCromSq(L, rPreto);
    float dBranco = distCromSq(L, rBranco);
    float dVerde  = distCromSq(L, rVerde);
    return (dVerde < dPreto && dVerde < dBranco);
}

// ════════════════════════════════════════════════════════════════
//  FSM PRINCIPAL
// ════════════════════════════════════════════════════════════════
void fsmUpdate() {
    // Lê TCS e atualiza histerese (só em SEGUINDO)
    LeituraCor cE, cD;
    if (tcsEOk) cE = lerCor(tcsE);
    if (tcsDOk)  cD = lerCor(tcsD);

    if (estado == SEGUINDO) {
        contE = (tcsEOk && ehVerde(cE, refPretoE, refBrancoE, refVerdeE)) ? contE + 1 : 0;
        contD = (tcsDOk  && ehVerde(cD, refPretoD, refBrancoD, refVerdeD)) ? contD + 1 : 0;
        if (contE >= N_HIST) latchE = true;
        if (contD >= N_HIST) latchD = true;
    }

    // Indicador visual: LED acende quando verde detectado
    digitalWrite(PIN_LED, (latchE || latchD) ? HIGH : LOW);

    switch (estado) {

    // ──────────────────────────────────────────────
    case SEGUINDO:
        // Disparar verificação se algum verde travado
        if (latchE || latchD) {
            Serial.print(F("[VERDE] Detectado: ESQ="));
            Serial.print(latchE); Serial.print(F(" DIR=")); Serial.println(latchD);
            tEstado = millis();
            estado  = VERDE_AVANCANDO;
            break;
        }
        // SEGUIDOR DE LINHA REMOVIDO — anda reto com os dois motores na mesma
        // velocidade. Para reativar o PID, troque a linha abaixo por pidLinha()
        // (a função continua definida e pronta para reintegração).
        moverFrente(lfspeed);
        break;

    // ──────────────────────────────────────────────
    case VERDE_AVANCANDO:
        // Lê QTR para verificar interseção
        qtr.readLineBlack(sensorValues);

        if (intersecaoDetectada()) {
            // Interseção confirmada (>= QTR_MIN_PRETOS sensores pretos).
            Serial.print(F("[VERDE] Intersecao confirmada ("));
            Serial.print(contarPretosSV());
            Serial.println(F("/5 pretos)."));
            tEstado = millis();
            estado  = VERDE_VIRANDO;
            moverRe(VEL_VOLTA);
            break;
        }
        if (millis() - tEstado >= T_AVANCO_MS) {
            // Falso alarme: nenhuma interseção encontrada
            Serial.println(F("[VERDE] Falso alarme — sem intersecao."));
            latchE = latchD = false;
            contE  = contD  = 0;
            previousError = 0; I_term = 0;
            estado = SEGUINDO;
            break;
        }
        moverFrente(VEL_AVANCO);
        break;

    // ──────────────────────────────────────────────
    case VERDE_VIRANDO:
        // Fase 1: volta T_VOLTA_MS para centralizar o eixo no cruzamento
        if (millis() - tEstado < T_VOLTA_MS) {
            moverRe(VEL_VOLTA);
            break;
        }
        // Fase 2: para, determina ângulo, gira (BLOCKING — ~0,5–2 s)
        pararMotores();
        aguardar(100);
        {
            float angulo;
            if (latchE && latchD) {
                angulo = 180.0f;
                Serial.println(F("[VERDE] Beco sem saida -> 180 graus"));
                bip(3, 80, 60);
            } else if (latchE) {
                angulo = -90.0f;
                Serial.println(F("[VERDE] Esquerda -> -90 graus"));
                bip(1);
            } else {
                angulo = 90.0f;
                Serial.println(F("[VERDE] Direita -> +90 graus"));
                bip(2, 80, 60);
            }
            girarAngulo(angulo, VEL_CURVA);
        }
        aguardar(150);
        // Fase 3: buscar linha
        tEstado = millis();
        estado  = REENCONTRANDO;
        break;

    // ──────────────────────────────────────────────
    case REENCONTRANDO:
        // Avança devagar até o sensor CENTRO ver preto
        qtr.readLineBlack(sensorValues);
        if (sensorValues[2] >= QTR_PRETO) {
            pararMotores();
            aguardar(50);
            latchE = latchD = false;
            contE  = contD  = 0;
            previousError = 0; I_term = 0;
            estado = SEGUINDO;
            Serial.println(F("[VERDE] Linha reencontrada."));
            break;
        }
        if (millis() - tEstado > T_REENCONTR_MAX) {
            pararMotores();
            latchE = latchD = false;
            contE  = contD  = 0;
            previousError = 0; I_term = 0;
            estado = SEGUINDO;
            Serial.println(F("[VERDE] Timeout reencontrar — retomando PID."));
            break;
        }
        moverFrente(VEL_REENCONTR);
        break;
    }
}

// ════════════════════════════════════════════════════════════════
//  CALIBRAÇÃO VIA SERIAL
// ════════════════════════════════════════════════════════════════
void tratarSerial() {
    if (!Serial.available()) return;
    char cmd = Serial.read();

    // Comandos que não dependem do TCS
    if (cmd == 'k') {                 // recalibrar QTR ao vivo e salvar na NVS
        calibrarQTRAoVivo();
        return;
    }
    if (cmd == 'z') {                 // apagar toda a calibração da NVS
        limparCalibracaoNVS();
        return;
    }

    if (!tcsEOk && !tcsDOk) { Serial.println(F("[CAL] Nenhum TCS disponivel.")); return; }

    // Lê os dois sensores agora (ambos sobre a mesma cor durante a calibração)
    LeituraCor cE, cD;
    if (tcsEOk) cE = lerCor(tcsE);
    if (tcsDOk) cD = lerCor(tcsD);

    switch (cmd) {
        case 'p':
            if (tcsEOk) refPretoE = { cE.r, cE.g, cE.b };
            if (tcsDOk) refPretoD = { cD.r, cD.g, cD.b };
            salvarCalibracaoCor();
            Serial.print(F("[CAL] PRETO  ESQ="));
            Serial.print(cE.r); Serial.print(','); Serial.print(cE.g); Serial.print(','); Serial.print(cE.b);
            Serial.print(F("  DIR="));
            Serial.print(cD.r); Serial.print(','); Serial.print(cD.g); Serial.print(','); Serial.println(cD.b);
            break;
        case 'b':
            if (tcsEOk) refBrancoE = { cE.r, cE.g, cE.b };
            if (tcsDOk) refBrancoD = { cD.r, cD.g, cD.b };
            salvarCalibracaoCor();
            Serial.print(F("[CAL] BRANCO ESQ="));
            Serial.print(cE.r); Serial.print(','); Serial.print(cE.g); Serial.print(','); Serial.print(cE.b);
            Serial.print(F("  DIR="));
            Serial.print(cD.r); Serial.print(','); Serial.print(cD.g); Serial.print(','); Serial.println(cD.b);
            break;
        case 'g':
            if (tcsEOk) refVerdeE = { cE.r, cE.g, cE.b };
            if (tcsDOk) refVerdeD = { cD.r, cD.g, cD.b };
            salvarCalibracaoCor();
            Serial.print(F("[CAL] VERDE  ESQ="));
            Serial.print(cE.r); Serial.print(','); Serial.print(cE.g); Serial.print(','); Serial.print(cE.b);
            Serial.print(F("  DIR="));
            Serial.print(cD.r); Serial.print(','); Serial.print(cD.g); Serial.print(','); Serial.println(cD.b);
            break;
        case 'v':
            Serial.print(F("[CAL] ESQ="));
            Serial.print(cE.r); Serial.print(','); Serial.print(cE.g); Serial.print(','); Serial.print(cE.b);
            Serial.print(F(" C=")); Serial.print(cE.c);
            Serial.print(tcsEOk && ehVerde(cE, refPretoE, refBrancoE, refVerdeE) ? F("[VERDE]") : F("[-]"));
            Serial.print(F("   DIR="));
            Serial.print(cD.r); Serial.print(','); Serial.print(cD.g); Serial.print(','); Serial.print(cD.b);
            Serial.print(F(" C=")); Serial.print(cD.c);
            Serial.println(tcsDOk && ehVerde(cD, refPretoD, refBrancoD, refVerdeD) ? F("[VERDE]") : F("[-]"));
            break;
        case 'c':
            Serial.print(F("[CAL] ESQ P="));
            Serial.print(refPretoE.r); Serial.print(','); Serial.print(refPretoE.g); Serial.print(','); Serial.print(refPretoE.b);
            Serial.print(F(" B="));
            Serial.print(refBrancoE.r); Serial.print(','); Serial.print(refBrancoE.g); Serial.print(','); Serial.print(refBrancoE.b);
            Serial.print(F(" V="));
            Serial.print(refVerdeE.r); Serial.print(','); Serial.print(refVerdeE.g); Serial.print(','); Serial.println(refVerdeE.b);
            Serial.print(F("[CAL] DIR P="));
            Serial.print(refPretoD.r); Serial.print(','); Serial.print(refPretoD.g); Serial.print(','); Serial.print(refPretoD.b);
            Serial.print(F(" B="));
            Serial.print(refBrancoD.r); Serial.print(','); Serial.print(refBrancoD.g); Serial.print(','); Serial.print(refBrancoD.b);
            Serial.print(F(" V="));
            Serial.print(refVerdeD.r); Serial.print(','); Serial.print(refVerdeD.g); Serial.print(','); Serial.println(refVerdeD.b);
            Serial.print(F("[CAL] N_HIST=")); Serial.print(N_HIST);
            Serial.print(F("  Kp="));         Serial.print(Kp, 3);
            Serial.print(F("  Ki="));         Serial.print(Ki, 4);
            Serial.print(F("  Kd="));         Serial.print(Kd, 3);
            Serial.print(F("  lfspeed="));    Serial.println(lfspeed);
            break;
        case 'i':
            Serial.print(F("[DBG] estado="));   Serial.print(estado);
            Serial.print(F("  lE="));           Serial.print(latchE);
            Serial.print(F("  lD="));           Serial.print(latchD);
            Serial.print(F("  contE="));        Serial.print(contE);
            Serial.print(F("  contD="));        Serial.print(contD);
            Serial.print(F("  pretos="));       Serial.print(contarPretosSV());
            Serial.print(F("/5  minInters="));  Serial.println(QTR_MIN_PRETOS);
            break;
    }
}

// ════════════════════════════════════════════════════════════════
//  TELEMETRIA SERIAL (250 ms, não-bloqueante)
// ════════════════════════════════════════════════════════════════
void imprimirTelemetria() {
    static unsigned long tUlt = 0;
    if (millis() - tUlt < 250) return;
    tUlt = millis();

    // QTR: imprime ##### onde # = preto, . = branco
    Serial.print(F("QTR["));
    for (int i = 0; i < NUM_SENS; i++)
        Serial.print(sensorValues[i] >= QTR_PRETO ? '#' : '.');
    Serial.print(F("] "));

    // TCS esquerdo (RGB)
    if (tcsEOk) {
        LeituraCor cE = lerCor(tcsE);
        Serial.print(F("ESQ["));
        Serial.print(cE.r); Serial.print(','); Serial.print(cE.g); Serial.print(','); Serial.print(cE.b);
        Serial.print(ehVerde(cE, refPretoE, refBrancoE, refVerdeE) ? F(" VERDE]  ") : F(" -]  "));
    }

    // TCS direito (RGB)
    if (tcsDOk) {
        LeituraCor cD = lerCor(tcsD);
        Serial.print(F("DIR["));
        Serial.print(cD.r); Serial.print(','); Serial.print(cD.g); Serial.print(','); Serial.print(cD.b);
        Serial.print(ehVerde(cD, refPretoD, refBrancoD, refVerdeD) ? F(" VERDE]") : F(" -]"));
    }

    Serial.println();
}

// ════════════════════════════════════════════════════════════════
//  BUZZER
// ════════════════════════════════════════════════════════════════
void bip(int n, int ms_on, int ms_off) {
    for (int i = 0; i < n; i++) {
        digitalWrite(PIN_BUZZER, HIGH); aguardar(ms_on);
        digitalWrite(PIN_BUZZER, LOW);  aguardar(ms_off);
    }
}

/*
 * ════════════════════════════════════════════════════════════════
 *  TUTORIAL DE CALIBRAÇÃO — PASSO A PASSO
 * ════════════════════════════════════════════════════════════════
 *
 *  PASSO 1 — CALIBRAÇÃO QTR (automática no setup)
 *  ────────────────────────────────────────────────
 *  Quando ligar o robô, o LED (D2) acende e o Serial imprime:
 *  "Calibrando QTR — MOVA O ROBO SOBRE A LINHA PRETA AGORA"
 *
 *  Você tem 400 leituras × 2 ms = ~800 ms para:
 *    a) Mover o robô em arco sobre a fita preta E o piso branco
 *    b) Passar todos os 5 sensores por cima da linha várias vezes
 *  Quando o LED apagar = calibração QTR concluída (2 bips).
 *
 *  Se o robô virar errado na faixa:
 *    → Troque HIGH/LOW nos pinos AIN1/AIN2 (ou BIN1/BIN2) em
 *      moverFrente() e moverRe() até o robô andar pra frente.
 *
 *  PASSO 2 — CALIBRAÇÃO DE COR RGB (via Monitor Serial 115200)
 *  ──────────────────────────────────────────────────────────
 *  Faça isso com o robô PARADO e os sensores de cor a ~5 mm do piso.
 *  Cada comando captura os 3 canais RGB brutos e SALVA na flash (NVS),
 *  então você só precisa calibrar UMA vez.
 *
 *  a) Posicione o TCS sobre a LINHA PRETA  → envie 'p'
 *     Serial imprime: "[CAL] PRETO  RGB=r,g,b"
 *
 *  b) Posicione o TCS sobre o PISO BRANCO  → envie 'b'
 *     Serial imprime: "[CAL] BRANCO RGB=r,g,b"
 *
 *  c) Posicione o TCS sobre o MARCADOR VERDE → envie 'g'
 *     Serial imprime: "[CAL] VERDE  RGB=r,g,b"  (G deve dominar R e B)
 *
 *  d) Envie 'v' para ler o RGB atual e ver a classificação:
 *     "[CAL] Leitura RGB=r,g,b  C=XXXX  -> VERDE / nao-verde"
 *     Confirme sobre cada cor que a classificação está correta.
 *
 *  e) Envie 'c' para ver as 3 referências salvas.
 *
 *  A classificação usa o VIZINHO MAIS PRÓXIMO: a cor lida é comparada
 *  com as 3 referências e vence a de menor distância. Não há thresholds
 *  manuais — basta calibrar bem as 3 cores. Se houver confusão entre
 *  duas cores, recalibre a referência que estiver "puxando" errado.
 *
 *  Critérios de BOAS referências RGB:
 *    • PRETO  → R, G, B todos baixos (ex.: < 100)
 *    • BRANCO → R, G, B todos altos e equilibrados entre si
 *    • VERDE  → G claramente maior que R e B
 *
 *  PASSO 3 — CALIBRAÇÃO DO GIRO (MPU6050)
 *  ────────────────────────────────────────
 *  O robô calibra o gyro automaticamente no setup (500 amostras).
 *  NÃO mova o robô durante os 3 bips iniciais.
 *
 *  Para verificar se gira 90° correto:
 *    a) Marque uma posição no chão.
 *    b) Force um verde (cubra o TCS esquerdo com papel verde) e
 *       espere o robô virar.
 *    c) Meça o ângulo real com um transferidor ou marcas no piso.
 *    d) Se girou a MENOS: diminua COMP_ANGULO (ex.: de 12 → 8).
 *       Se girou a MAIS:  aumente COMP_ANGULO (ex.: de 12 → 16).
 *    e) Repita até o giro ficar em 90° ± 3°.
 *
 *  PASSO 4 — CALIBRAÇÃO DO PID
 *  ─────────────────────────────
 *  Velocidade base: comece com lfspeed = 150.
 *
 *  Método (igual ao seguidor_de_linha_PID_esp32):
 *    1. Ki = 0, Kd = 0. Aumente Kp até o robô oscilar na linha.
 *       Valor inicial: Kp = 0.3.
 *    2. Adicione Kd até a oscilação parar.
 *       Valor inicial: Kd = 1.2 (Kd ≈ 4× Kp é um ponto de partida).
 *    3. Se ainda houver erro acumulado em curvas longas, adicione Ki
 *       pequeno (0.001–0.005).
 *    4. Suba lfspeed gradualmente até o limite onde o robô ainda
 *       acompanha as curvas. Típico: 150–220.
 *
 *  PASSO 5 — TESTE DO PONTO VERDE
 *  ────────────────────────────────
 *  a) Coloque o robô sobre a linha antes de um cruzamento com verde.
 *  b) O Serial deve mostrar "[VERDE] Detectado: ESQ=1 DIR=0" quando
 *     o robô passar sobre o marcador.
 *  c) O robô deve avançar, ver a interseção confirmada (X/5 pretos),
 *     recuar brevemente, girar 90° e reencontrar a linha.
 *  d) Se não detectar verde: recalibre o VERDE ('g') sobre o marcador,
 *     ou aproxime o sensor do piso e refaça as 3 referências.
 *  e) Se detectar falsos positivos: recalibre PRETO/BRANCO ('p'/'b')
 *     para afastar essas referências do verde.
 *  f) Verifique o campo "pretos=X/5" via comando 'i' no Serial:
 *       • Interseção + (4 ramos), abordagem frontal:  espere 5/5
 *       • Interseção T, abordagem pelo CABO:          espere 5/5
 *       • Interseção T, abordagem pelo BRAÇO lateral: espere 3-4/5
 *     Se ver 3-4/5 mas o robô não girar: QTR_PRETO pode estar alto
 *     (rebaixe de 800 para 700) ou QTR_MIN_PRETOS pode estar alto
 *     (já está em 3, que é o mínimo recomendado).
 *  g) Se o robô girar em curvas normais (falso alarme): a interseção
 *     está gerando >= 3 pretos quando não deveria. Suba QTR_MIN_PRETOS
 *     para 4 e observe se T lateral ainda é detectado.
 *
 *  PASSO 6 — AJUSTE DE T_AVANCO_MS e T_VOLTA_MS
 *  ───────────────────────────────────────────────
 *  T_AVANCO_MS = tempo que o robô avança para verificar a interseção.
 *    → Se parar antes do cruzamento: aumente.
 *    → Se passar rápido demais e não ver todos pretos: diminua VEL_AVANCO.
 *
 *  T_VOLTA_MS = tempo de ré para centralizar o eixo de rodas no crux.
 *    → Se girar fora da interseção: aumente.
 *    → Se recuar demais e sair da interseção: diminua.
 *
 * ════════════════════════════════════════════════════════════════
 */
