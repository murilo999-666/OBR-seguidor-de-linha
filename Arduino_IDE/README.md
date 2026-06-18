# Projetos OBR — ESP32

Três projetos para ESP32 (DOIT ESP32 DEVKIT V1), disponíveis em duas formas:

- **PlatformIO** — projeto original (`src/main.cpp` + `platformio.ini`).
- **Arduino IDE** — esta pasta `Arduino_IDE/`, com cada sketch em uma pasta de mesmo nome do `.ino`.

## Placa e configuração
- **Placa:** ESP32 Dev Module (DOIT ESP32 DEVKIT V1)
- **Monitor serial:** 115200 baud
- No Arduino IDE, instale o core **esp32 by Espressif Systems** (Gerenciador de Placas).

---

## Bibliotecas necessárias por projeto

### 1. `seguidor_de_linha_PID_esp32` — Seguidor de linha com PID

| Biblioteca | PlatformIO (`lib_deps`) | Arduino IDE (Gerenciador de Bibliotecas) |
|------------|--------------------------|------------------------------------------|
| L298N      | `AndreaLombardo/L298N`   | **L298N** (por Andrea Lombardo)          |
| QTRSensors | `pololu/QTRSensors`      | **QTRSensors** (por Pololu)              |

### 2. `SeguidorPontoVerde` — Seguidor + ponto verde (OBR 2026)

| Biblioteca       | PlatformIO (`lib_deps`)          | Arduino IDE (Gerenciador de Bibliotecas) |
|------------------|----------------------------------|------------------------------------------|
| QTRSensors       | `pololu/QTRSensors`              | **QTRSensors** (por Pololu)              |
| MPU6050          | `electroniccats/MPU6050`        | **MPU6050** (por Electronic Cats)        |
| Adafruit TCS34725| `adafruit/Adafruit TCS34725`    | **Adafruit TCS34725** (por Adafruit)     |

> A **Adafruit TCS34725** depende de **Adafruit Unified Sensor** e **Adafruit BusIO**.
> No Arduino IDE, aceite a instalação das dependências quando solicitado.
> No PlatformIO elas são resolvidas automaticamente.

### 3. `RoboDesvio` — Robô desviador de obstáculos

| Biblioteca | PlatformIO (`lib_deps`)    | Arduino IDE (Gerenciador de Bibliotecas) |
|------------|----------------------------|------------------------------------------|
| MPU6050    | `electroniccats/MPU6050`  | **MPU6050** (por Electronic Cats)        |

> O sensor ultrassônico HC-SR04 usa apenas `pulseIn()` — **não precisa de biblioteca**.

---

## Resumo — todas as bibliotecas (instalação única cobrindo os 3 projetos)

### Arduino IDE — Gerenciador de Bibliotecas (Ferramentas → Gerenciar Bibliotecas)
- **L298N** — Andrea Lombardo
- **QTRSensors** — Pololu
- **MPU6050** — Electronic Cats
- **Adafruit TCS34725** — Adafruit
- *(dependências automáticas)* Adafruit Unified Sensor, Adafruit BusIO

### PlatformIO — `lib_deps`
```ini
lib_deps =
    AndreaLombardo/L298N
    pololu/QTRSensors
    electroniccats/MPU6050
    adafruit/Adafruit TCS34725
```

---

> Os `.ino` desta pasta têm código idêntico ao `src/main.cpp` de cada projeto PlatformIO original.
