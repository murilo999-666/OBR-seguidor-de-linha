# CLAUDE.md — Seguidor de Linha + Ponto Verde (OBR 2026)

Guia para retomar este projeto em outra máquina. Leia antes de editar.

## O que é
Robô da OBR 2026 (Nível 2) em **ESP32**. Combina seguidor de linha (QTR + PID),
detecção de **ponto verde** (2× TCS34725) para decidir curvas em interseções, e
giro controlado por ângulo (MPU6050). Baseado em dois projetos do mesmo repo:
`seguidor_de_linha_PID_esp32` (PID) e `RoboDesvio` (MPU6050 + LEDC).

## Onde está o código (este repositório: `OBR-seguidor-de-linha`)
- **PlatformIO (fonte principal):** `seguidor_ponto_verde_esp32/`
  - `src/main.cpp` — todo o programa (arquivo único).
  - `platformio.ini` — placa `esp32doit-devkit-v1`, libs em `lib_deps`.
- **Arduino IDE (cópia para a IDE):** `Arduino_IDE/SeguidorPontoVerde/`
  - `SeguidorPontoVerde.ino` — MESMO código + cabeçalho explicando instalação.
  - `README.md` — passo a passo de bibliotecas/placa.
  - `CLAUDE.md` — este arquivo.

> As duas versões devem ficar em sincronia. O `.ino` é o `main.cpp` com um
> bloco de instruções no topo e o banner trocado para "Arduino IDE". Ao alterar
> o `main.cpp`, regenere o `.ino` (ver "Como sincronizar" no fim).

## Estado atual (jun/2026)
- **Leitura de cor: validada na bancada.** Falta testar em PISTA real.
- **Seguidor de linha (PID): DESATIVADO.** No estado `SEGUINDO` o robô anda reto
  com os dois motores na mesma velocidade (`moverFrente(lfspeed)`). A função
  `pidLinha()` continua pronta — para reativar, troque essa chamada por
  `pidLinha()`. Isso foi pedido de propósito (testar a parte de cor/giro primeiro).
- **Ponto verde: ativo** (detecção, interseção via QTR, giro, reencontro).
- **Calibração persistente em NVS** (flash interna), exceto o giroscópio.

## Arquitetura (FSM em `fsmUpdate()`)
Estados: `SEGUINDO → VERDE_AVANCANDO → VERDE_VIRANDO → REENCONTRANDO → SEGUINDO`.
1. `SEGUINDO`: anda reto; se um TCS confirmar verde (histerese `N_HIST`), trava
   `latchE`/`latchD` e vai para `VERDE_AVANCANDO`.
2. `VERDE_AVANCANDO`: avança devagar até `intersecaoDetectada()` (>= `QTR_MIN_PRETOS`
   sensores pretos) ou `T_AVANCO_MS` (falso alarme → volta a `SEGUINDO`).
3. `VERDE_VIRANDO`: recua `T_VOLTA_MS` para centralizar, depois `girarAngulo()`
   (−90 esq, +90 dir, 180 se ambos) via MPU6050.
4. `REENCONTRANDO`: avança até o sensor central ver preto (ou timeout) → `SEGUINDO`.

## Detecção de cor (parte mais delicada — histórico importante)
Evoluiu em 3 passos, NÃO regredir:
1. Começou com razões `g/c` + thresholds (`GC_MIN`, etc.). **Removido.**
2. Virou **RGB bruto, vizinho mais próximo** de 3 referências (preto/branco/verde).
   - Bug 1: calibrava só 1 sensor e usava para os 2 → preto de um virava verde.
     **Correção:** referências POR SENSOR (`refPretoE/BrancoE/VerdeE` e `...D`).
   - Bug 2: na transição branco→preto os valores brutos passam pelo "médio",
     perto do verde → falso verde.
3. **Solução atual: vizinho mais próximo em CROMATICIDADE** (RGB normalizado por
   R+G+B), em `distCromSq()`/`ehVerde()`. Normalizar tira o brilho: branco, preto
   e a transição ficam equilibrados (~1/3 por canal) e só o verde (G dominante)
   se aproxima da ref verde. **Validado na bancada.**

Referências guardadas como **RGB bruto** (display de calibração intuitivo); a
normalização é feita só na comparação. 6 referências (3 cores × 2 sensores).

## Persistência (NVS via `Preferences`, namespace `"calib"`)
- QTR: `qtr_ok`, `qtr_min`, `qtr_max` (carrega na 1ª; senão calibra ao vivo e salva).
- Cor: flag `cor_ok2` + `refPE/refBE/refVE/refPD/refBD/refVD` (bytes da struct `RefCor`).
- `limparCalibracaoNVS()` apaga tudo (comando `z`).
- MPU NÃO é salvo: recalibra a cada boot (500 amostras, robô parado).
- Se mudar o esquema das structs salvas, **suba a versão da flag** (ex.: `cor_ok3`)
  para invalidar dados antigos automaticamente.

## Pinagem
Ponte H: AIN1=21 AIN2=22 PWMA=23 / BIN1=25 BIN2=33 PWMB=32.
QTR (5×): 26,27,14,12,13. (GPIO12 é strapping — 10kΩ p/ GND se boot loop.)
TCS ESQ: `Wire` SDA=16 SCL=17 (junto do MPU6050 0x68). TCS DIR: `Wire1` SDA=4 SCL=5.
Buzzer=15, LED=2. Ambos TCS no endereço 0x29, em barramentos I2C separados.

## Comandos Serial (115200)
`p`/`b`/`g` = capturar RGB de preto/branco/verde (os DOIS sensores) e salvar.
`v` = ler e classificar os dois lados. `k` = recalibrar QTR. `z` = limpar NVS.
`c` = imprimir referências/ganhos. `i` = debug da FSM.

## Parâmetros tunáveis (topo do arquivo)
PID: `Kp=0.30 Ki=0.002 Kd=1.20 lfspeed=200`. Giro: `VEL_CURVA=180`,
`COMP_ANGULO=12` (compensa inércia — calibrar na pista). Verde/interseção:
`N_HIST=3`, `QTR_MIN_PRETOS=3`, `T_AVANCO_MS`, `T_VOLTA_MS`, etc.

## Como compilar
- **PlatformIO:** `pio run` em `seguidor_ponto_verde_esp32/` (libs baixam sozinhas).
- **Arduino IDE:** ver `README.md` (instalar core ESP32 + 3 libs + Adafruit BusIO).

## Como sincronizar .ino ↔ main.cpp
A partir da raiz do repo:
```
SRC=seguidor_ponto_verde_esp32/src/main.cpp
INO=Arduino_IDE/SeguidorPontoVerde/SeguidorPontoVerde.ino
# (manter o bloco de cabeçalho do .ino) e reanexar o código com o banner trocado:
sed 's@ESP32 / PlatformIO@ESP32 / Arduino IDE@' "$SRC"   # cole após o cabeçalho
```
O cabeçalho de instruções do `.ino` vai do início até a linha
`*/ ` logo antes de `/* ... SEGUIDOR DE LINHA + PONTO VERDE ...`.

## Próximos passos
1. Testar em pista: seguimento (reto por enquanto) + ponto verde de ponta a ponta.
2. Calibrar `COMP_ANGULO` até o giro de 90° ficar exato.
3. Reativar PID (`pidLinha()` no `SEGUINDO`) quando a cor/giro estiverem ok.
4. Se a luz do ambiente variar muito, a cromaticidade já ajuda; reavaliar só se preciso.
