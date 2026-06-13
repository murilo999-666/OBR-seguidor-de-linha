# OBR 2026 — Robô ESP32

Projetos de robótica para a **Olimpíada Brasileira de Robótica 2026**, desenvolvidos para ESP32 com PlatformIO.

---

## Hardware

| Componente | Modelo |
|---|---|
| Microcontrolador | ESP32 DevKit v1 |
| Motores | N20 100 RPM |
| Driver de motor | L298N (Ponte H dupla) |
| Sensores de linha | IR analógico (x5) |
| Sensor de distância | Ultrassônico HC-SR04 |
| Regulador de tensão | Step-down DC-DC |

---

## Pinagem

### Ponte H (L298N)

| Pino ESP32 | Função |
|---|---|
| D21 | AIN1 |
| D22 | AIN2 |
| D23 | PWMA |
| D25 | BIN1 |
| D33 | BIN2 |
| D32 | PWMB |

### Sensores IR (QTR Analógico)

| Pino ESP32 | Posição |
|---|---|
| D26 | Extrema-esquerda |
| D27 | Esquerda |
| D14 | Centro |
| D12 | Direita |
| D13 | Extrema-direita |

### Sensor Ultrassônico (HC-SR04)

| Pino ESP32 | Função |
|---|---|
| D18 | Echo |
| D19 | Trig |

### Buzzer

| Pino ESP32 | Função |
|---|---|
| D15 | Buzzer |

### I2C

| Pino ESP32 | Função |
|---|---|
| D16 | SDA |
| D17 | SCL |

### Portas Livres

| Pino | Observação |
|---|---|
| D4 | Digital, PWM — sensores, relé, LEDs |
| D5 | Digital, PWM, SPI |
| D2 | Digital, PWM, LED onboard — ⚠️ cuidado |
| D0 | Digital, botão de boot — ⚠️ cuidado |
| TX0 | Serial / digital — ⚠️ muito cuidado |
| RX0 | Serial / digital — ⚠️ muito cuidado |
| D34 | Somente entrada — evitar |
| D35 | Somente entrada — evitar |
| VN | Somente entrada — EVITAR |
| VP | Somente entrada — EVITAR |

---

## Projetos

### `seguidor_de_linha_pid_esp32/`
Seguidor de linha com controle PID. Lê 5 sensores IR analógicos e ajusta a velocidade dos dois motores proporcionalmente ao erro de posição.

- Controle PID (Kp, Ki, Kd ajustáveis)
- Calibração automática dos sensores na inicialização
- Recovery automático ao perder a linha (giro no sentido do último erro)
- LED interno indica centralização na linha

### `robo_desvio_obr_2026/`
Robô desviador de obstáculos. Usa sensor ultrassônico para detectar obstáculos e desviar.

### `seguidor_ponto_verde_esp32/`
Seguidor de ponto verde. Detecta e segue um marcador de cor verde.

---

## Estrutura dos Projetos

Todos os projetos usam **PlatformIO** com a plataforma `espressif32` e a placa `esp32doit-devkit-v1`.

```
projeto/
├── src/
│   └── main.cpp
└── platformio.ini
```
