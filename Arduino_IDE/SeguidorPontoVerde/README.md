# Seguidor de Linha + Ponto Verde — OBR 2026 (Arduino IDE)

Versão **Arduino IDE** do robô para a OBR 2026 (Nível 2), ESP32.
Mesmo código da versão PlatformIO em [`../../seguidor_ponto_verde_esp32/`](../../seguidor_ponto_verde_esp32/).

> Estado atual: **leitura de cor validada na bancada**. Falta testar em pista real.
> O seguimento de linha por PID está **desativado** (o robô anda reto com os dois
> motores na mesma velocidade), mas a função `pidLinha()` continua no código,
> pronta para reativar. Toda a lógica de ponto verde (detectar verde, achar a
> interseção, girar com o giroscópio e reencontrar a linha) está ativa.

---

## 1. Instalar o suporte ao ESP32

1. **Arquivo > Preferências** → em *URLs Adicionais de Gerenciadores de Placas*, adicione:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
2. **Ferramentas > Placa > Gerenciador de Placas…** → procure `esp32` → instale
   **"esp32 by Espressif Systems"** (3.x recomendado; também compila no 2.x).
3. Selecione **Ferramentas > Placa > ESP32 Arduino > "DOIT ESP32 DEVKIT V1"**.

## 2. Instalar as bibliotecas

**Ferramentas > Gerenciar Bibliotecas…** e instale:

| Biblioteca | Autor | Para quê |
|---|---|---|
| **QTRSensors** | Pololu | Sensores de linha (5× IR analógico) |
| **MPU6050** | Electronic Cats | Giroscópio (giro controlado por ângulo) |
| **Adafruit TCS34725** | Adafruit | Sensores de cor (detecção de verde) |

> Ao instalar a Adafruit TCS34725, aceite instalar a dependência **Adafruit BusIO**.

Já vêm no core do ESP32 (**não precisa instalar**): `Wire` (I2C, usa dois
barramentos: `Wire` e `Wire1`) e `Preferences` (NVS — salva calibração na flash).

## 3. Configuração da IDE

- **Monitor Serial:** 115200 baud
- **Upload Speed:** 921600 (use 115200 se der erro)
- **Partition Scheme:** padrão

## 4. Abrir e enviar

A pasta precisa se chamar `SeguidorPontoVerde` e o arquivo
`SeguidorPontoVerde.ino` (regra da Arduino IDE: pasta = nome do sketch).
Abra o `.ino`, conecte o ESP32, escolha a porta COM e clique em **Carregar**.

---

## Pinagem

| Bloco | Pinos |
|---|---|
| Ponte H | AIN1=21 AIN2=22 PWMA=23 / BIN1=25 BIN2=33 PWMB=32 |
| QTR analógico (5×) | EE=26 E=27 C=14 D=12(*) ED=13 |
| TCS34725 ESQUERDO | `Wire`  SDA=16 SCL=17 (junto com o MPU6050) |
| TCS34725 DIREITO | `Wire1` SDA=4 SCL=5 (barramento I2C separado) |
| Buzzer / LED | D15 / D2 |

(*) GPIO12 é *strapping pin*. Se houver boot loop, ligue 10 kΩ entre D12 e GND.

---

## Calibração (uma vez — fica salva na flash/NVS)

Tudo, menos o giroscópio, é salvo na memória interna (NVS) e recarregado no boot.

**Cores (Monitor Serial, com o robô parado, sensores a ~5 mm do piso):**
coloque os **dois** sensores sobre a mesma cor e envie:

| Cmd | Ação |
|---|---|
| `p` | captura RGB do **preto** (ambos os sensores) e salva |
| `b` | captura RGB do **branco** e salva |
| `g` | captura RGB do **verde** e salva |
| `v` | lê os dois sensores e mostra a classificação (VERDE / -) |
| `k` | recalibra o QTR ao vivo e salva |
| `z` | apaga toda a calibração da NVS |
| `c` | imprime as referências e os ganhos atuais |
| `i` | debug do estado da FSM |

**Como a cor é classificada:** vizinho mais próximo em **cromaticidade**
(RGB normalizado por R+G+B). Normalizar remove o brilho, então a transição
branco→preto (que no RGB bruto passa por valores "médios" parecidos com verde)
**não** é mais confundida com verde. Cada sensor usa as próprias referências
(os dois TCS têm escalas diferentes).

---

## Próximos passos

- [ ] Testar em pista real (seguimento + ponto verde).
- [ ] Calibrar `COMP_ANGULO` para o giro de 90° ficar exato.
- [ ] (Opcional) Reativar o PID de linha: trocar `moverFrente(lfspeed)` por
      `pidLinha()` no estado `SEGUINDO` da `fsmUpdate()`.

Detalhes de continuidade do projeto: ver [`CLAUDE.md`](CLAUDE.md).
