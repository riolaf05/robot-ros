# Schema Visuale Dettagliato - Wiring Diagram

Questo documento fornisce schemi visuali dettagliati per il collegamento di tutti i componenti del robot.

## Indice

1. [Vista d'Insieme Sistema Completo](#vista-dinsieme-sistema-completo)
2. [Arduino Nano - Pinout e Collegamenti](#arduino-nano-pinout-e-collegamenti)
3. [L298N Motor Driver - Collegamenti](#l298n-motor-driver-collegamenti)
4. [Encoder - Collegamenti Dettagliati](#encoder-collegamenti-dettagliati)
5. [Sistema di Alimentazione](#sistema-di-alimentazione)
6. [RPLIDAR - Setup](#rplidar-setup)
7. [Checklist Collegamento Passo-Passo](#checklist-collegamento-passo-passo)

---

## Vista d'Insieme Sistema Completo

```
╔═══════════════════════════════════════════════════════════════════════╗
║                      ARCHITETTURA COMPLETA ROBOT                      ║
╚═══════════════════════════════════════════════════════════════════════╝

                            ┌─────────────────┐
                            │  RASPBERRY PI 4 │
                            │  (Main Brain)   │
                            │  Ubuntu 22.04   │
                            │  ROS2 Humble    │
                            └────┬─────┬──┬───┘
                                 │     │  │
                    ┌────────────┘     │  └─────────┐
                    │                  │            │
                    ▼                  ▼            ▼
            ┌──────────────┐   ┌──────────┐  ┌──────────┐
            │ ARDUINO NANO │   │ RPLIDAR  │  │  CAMERA  │
            │ (HW Control) │   │   A1/A2  │  │   USB    │
            └───────┬──────┘   └──────────┘  └──────────┘
                    │
                    │ Digital I/O
                    │ (PWM + Encoder)
                    │
            ┌───────┴───────┐
            │               │
            ▼               ▼
    ┌──────────────┐   ┌──────────┐
    │   L298N      │   │ ENCODER  │
    │ Motor Driver │   │  Left    │
    └──────┬───────┘   └────┬─────┘
           │                │
           ▼                ▼
    ┌────────────┐   ┌────────────┐
    │  MOTOR L   │   │  ENCODER   │
    │  + Encoder │   │   Right    │
    └────────────┘   └────────────┘

           │                │
           ▼                ▼
    ┌────────────┐   ┌────────────┐
    │  MOTOR R   │   │            │
    │  + Encoder │   │            │
    └────────────┘   └────────────┘


                ┌─────────────┐
                │  BATTERIA   │
                │   12V LiPo  │
                │ 2200-5000mAh│
                └──┬────────┬─┘
                   │        │
                   ▼        ▼
              ┌────────┐ ┌──────────┐
              │ L298N  │ │Step-Down │
              │  12V   │ │ 12V→5V   │
              └────────┘ └────┬─────┘
                              │
                              ▼
                        Raspberry Pi
```

---

## Arduino Nano - Pinout e Collegamenti

### Vista Superiore Arduino Nano

```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃                        ARDUINO NANO                               ┃
┃                                                                    ┃
┃    ┌───────────────────────────────────────────────────────┐     ┃
┃    │                                                         │     ┃
┃    │  [USB Mini-B]                                   [RESET]│     ┃
┃    │                                                         │     ┃
┃    │  TX  RX  RST  GND  D2  D3  D4  D5  D6  D7  D8  D9  D10│     ┃
┃    │  ○   ○   ○    ○    ●   ●   ●   ●   ●   ●   ●   ●   ● │     ┃
┃    │                    ┊   ┊   ┊   ┊   ┊   ┊   ┊   ┊   ┊  │     ┃
┃    │  ○   ○   ○    ○    ○   ○   ○   ○   ○   ○   ○   ○   ○ │     ┃
┃    │  30  A0  A1   A2   A3  A4  A5  A6  A7  5V  RST GND VIN│     ┃
┃    │                                    ●        ○       ●  │     ┃
┃    └───────────────────────────────────────────────────────┘     ┃
┃                                                                    ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛

Legenda:
● = Pin utilizzati in questo progetto
○ = Pin disponibili (non usati)
┊ = Connessione attiva (vedere tabella sotto)
```

### Tabella Dettagliata Collegamenti Arduino

| Pin | Nome  | Tipo      | Collegamento         | Destinazione          | Note                    |
|-----|-------|-----------|----------------------|-----------------------|-------------------------|
| D2  | INT0  | Input     | Encoder L - Ch A     | Encoder Sinistro      | Interrupt hardware      |
| D3  | INT1  | Input     | Encoder R - Ch A     | Encoder Destro        | Interrupt hardware      |
| D4  | I/O   | Input     | Encoder L - Ch B     | Encoder Sinistro      | Lettura direzione       |
| D5  | PWM   | Output    | Motor L - Enable     | L298N ENA             | PWM 0-255               |
| D6  | I/O   | Output    | Motor L - IN1        | L298N IN1             | Direzione HIGH/LOW      |
| D7  | I/O   | Output    | Motor L - IN2        | L298N IN2             | Direzione HIGH/LOW      |
| D8  | I/O   | Input     | Encoder R - Ch B     | Encoder Destro        | Lettura direzione       |
| D9  | PWM   | Output    | Motor R - Enable     | L298N ENB             | PWM 0-255               |
| D10 | I/O   | Output    | Motor R - IN1        | L298N IN3             | Direzione HIGH/LOW      |
| D11 | PWM   | Output    | Motor R - IN2        | L298N IN4             | Direzione HIGH/LOW      |
| 5V  | Power | Output    | VCC Encoder (x2)     | Encoder L + R         | Alimentazione 5V        |
| GND | Ground| Ground    | GND Encoder (x2)     | Encoder L + R         | Ground comune           |

---

## L298N Motor Driver - Collegamenti

### Vista Frontale L298N

```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃                       L298N MOTOR DRIVER                          ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛

     ┌──────────────────────────────────────────────────────┐
     │                                                        │
     │  ┌────────┐        L298N         ┌────────┐          │
     │  │ Heat   │                       │ Heat   │          │
     │  │ Sink   │                       │ Sink   │          │
     │  └────────┘                       └────────┘          │
     │                                                        │
     │  OUT1 OUT2        [JUMPER]        OUT3 OUT4           │
     │   ●    ●         [5V REG]          ●    ●            │
     │   ┃    ┃            ▼              ┃    ┃            │
     │   ┃    ┃       (RIMUOVERE!)        ┃    ┃            │
     │   ┃    ┃                            ┃    ┃            │
     │   ┗━━━━╋━━━━━━━━━━━━━━━━━━━━━━━━━━╋━━━━┛            │
     │        ┃                            ┃                 │
     │        ┃  Motor L                   ┃  Motor R        │
     │        ┃                            ┃                 │
     │                                                        │
     │  ENA IN1 IN2 IN3 IN4 ENB    +12V GND  +5V            │
     │   ●   ●   ●   ●   ●   ●      ●   ●    ○              │
     │   ┃   ┃   ┃   ┃   ┃   ┃      ┃   ┃                   │
     │   ┃   ┃   ┃   ┃   ┃   ┃      ┃   ┃                   │
     │   ┃   ┃   ┃   ┃   ┃   ┃      ┃   ┃                   │
     │   D5  D6  D7 D10 D11  D9    12V  GND                 │
     │   ┃   ┃   ┃   ┃   ┃   ┃     ┃   ┃                    │
     │   ┗━━━┻━━━┻━━━┻━━━┻━━━┛     ┃   ┃                    │
     │   Arduino Nano               ┃   ┃                    │
     │                              ┃   ┃                    │
     │                              ┗━━━┻━━━━━━━━━           │
     │                             Batteria 12V              │
     └──────────────────────────────────────────────────────┘

⚠️  IMPORTANTE: Rimuovere il jumper del regolatore 5V!
    (Evita sovraccarico se alimentazione esterna è presente)
```

### Tabella Collegamenti L298N

| Pin L298N | Tipo      | Collegamento            | Note                           |
|-----------|-----------|-------------------------|--------------------------------|
| ENA       | PWM Input | Arduino D5              | Enable Motor L (PWM)           |
| IN1       | Input     | Arduino D6              | Motor L - Direzione bit 1      |
| IN2       | Input     | Arduino D7              | Motor L - Direzione bit 2      |
| IN3       | Input     | Arduino D10             | Motor R - Direzione bit 1      |
| IN4       | Input     | Arduino D11             | Motor R - Direzione bit 2      |
| ENB       | PWM Input | Arduino D9              | Enable Motor R (PWM)           |
| +12V      | Power In  | Batteria (+)            | Alimentazione motori           |
| GND       | Ground    | Batteria (-) + Arduino  | Ground comune                  |
| OUT1/OUT2 | Output    | Motore Sinistro (+/-)   | Invertire se gira al contrario |
| OUT3/OUT4 | Output    | Motore Destro (+/-)     | Invertire se gira al contrario |
| +5V       | Power Out | NON USARE               | Rimuovi jumper!                |

---

## Encoder - Collegamenti Dettagliati

### Encoder Incrementale con 2 Canali

```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃                    ENCODER MOTORE (Tipico)                        ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛

              ┌─────────────────────┐
              │   ENCODER HOUSING   │
              │                     │
              │   ┌───────────┐     │
              │   │  Optical  │     │
              │   │  Sensor   │     │
              │   └───────────┘     │
              │                     │
              │   Motor Shaft       │
              │        ↓            │
              └────────┼────────────┘
                       │
                       │
              ┌────────┴────────┐
              │  CAVI ENCODER   │
              │                 │
              │  ┌───┬───┬───┬───┐
              │  │VCC│GND│ChA│ChB│
              │  └─┬─┴─┬─┴─┬─┴─┬─┘
              │    │   │   │   │
              │    │   │   │   │
              └────┼───┼───┼───┼────
                   │   │   │   │
                   ▼   ▼   ▼   ▼

     Encoder Sinistro (LEFT):
     VCC  → Arduino 5V
     GND  → Arduino GND
     ChA  → Arduino D2 (INT0)
     ChB  → Arduino D4

     Encoder Destro (RIGHT):
     VCC  → Arduino 5V
     GND  → Arduino GND
     ChA  → Arduino D3 (INT1)
     ChB  → Arduino D8
```

### Schema Collegamento Encoder Completo

```
                    ENCODER SINISTRO              ENCODER DESTRO
                          (LEFT)                      (RIGHT)

                    ┌──────────────┐            ┌──────────────┐
                    │              │            │              │
                    │    VCC (+)   │            │    VCC (+)   │
                    │     │        │            │     │        │
                    └─────┼────────┘            └─────┼────────┘
                          │                           │
                          └─────┬─────────────────────┘
                                │
                                │ +5V
                                │
                    ┌───────────┴────────────┐
                    │    ARDUINO NANO 5V     │
                    └────────────────────────┘

                    ┌──────────────┐            ┌──────────────┐
                    │              │            │              │
                    │   GND (-)    │            │   GND (-)    │
                    │     │        │            │     │        │
                    └─────┼────────┘            └─────┼────────┘
                          │                           │
                          └─────┬─────────────────────┘
                                │
                                │ GND
                                │
                    ┌───────────┴────────────┐
                    │   ARDUINO NANO GND     │
                    └────────────────────────┘

                    ┌──────────────┐            ┌──────────────┐
                    │  Channel A   │            │  Channel A   │
                    │     │        │            │     │        │
                    └─────┼────────┘            └─────┼────────┘
                          │                           │
                          │ D2 (INT0)                 │ D3 (INT1)
                          │                           │
                    ┌─────┴────────┐            ┌─────┴────────┐
                    │ ARDUINO D2   │            │ ARDUINO D3   │
                    └──────────────┘            └──────────────┘

                    ┌──────────────┐            ┌──────────────┐
                    │  Channel B   │            │  Channel B   │
                    │     │        │            │     │        │
                    └─────┼────────┘            └─────┼────────┘
                          │                           │
                          │ D4                        │ D8
                          │                           │
                    ┌─────┴────────┐            ┌─────┴────────┐
                    │ ARDUINO D4   │            │ ARDUINO D8   │
                    └──────────────┘            └──────────────┘
```

---

## Sistema di Alimentazione

### Schema Completo Alimentazione

```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃                    SISTEMA DI ALIMENTAZIONE                       ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛

                         ┌─────────────────┐
                         │   BATTERIA      │
                         │   LiPo 3S       │
                         │   11.1V - 12.6V │
                         │   2200-5000mAh  │
                         └─────┬──────┬────┘
                               │      │
                        (+) 12V│      │GND (-)
                               │      │
                ┌──────────────┘      └──────────────┐
                │                                     │
                ▼                                     ▼
        ┌───────────────┐                     ┌──────────────┐
        │   L298N       │                     │  STEP-DOWN   │
        │  Motor Driver │                     │  CONVERTER   │
        │               │                     │  12V → 5V    │
        │  +12V    GND  │                     │  (3A min)    │
        └───┬───────┬───┘                     └──────┬───────┘
            │       │                                │
            │       │                                │ +5V (3A)
            │       │                                │
            ▼       ▼                                ▼
      ┌─────────────────┐                  ┌─────────────────┐
      │  Motore SX + DX │                  │  RASPBERRY PI 4 │
      │  (12V, 2-3A)    │                  │  (5V, 2.5-3A)   │
      └─────────────────┘                  └─────────────────┘
                                                    │
                                                    │ USB 5V
                                                    │
                                           ┌────────┴─────────┐
                                           │                  │
                                           ▼                  ▼
                                    ┌─────────────┐   ┌─────────────┐
                                    │ Arduino Nano│   │   RPLIDAR   │
                                    │ (via USB)   │   │  (via USB)  │
                                    └─────────────┘   └─────────────┘

⚠️  NOTE IMPORTANTI:
    1. Tutti i GND devono essere collegati insieme (ground comune)
    2. Usa fili adeguati: 18AWG per motori, 20AWG per alimentazione 5V
    3. Verifica polarità prima di alimentare
    4. Step-Down deve fornire almeno 3A stabile
    5. Aggiungi condensatori (100µF) vicino al L298N per stabilità
```

### Specifiche Alimentazione

| Componente      | Voltage | Corrente  | Note                              |
|-----------------|---------|-----------|-----------------------------------|
| Batteria LiPo   | 11.1V   | 2200-5000mAh | 3S (3 celle)               |
| L298N Input     | 12V     | -         | Alimentazione motori              |
| Motori DC       | 12V     | 1-2A each | Peak 2-3A per motore              |
| Step-Down Input | 12V     | -         | Regolatore buck                   |
| Step-Down Output| 5V      | 3A        | Per Raspberry Pi                  |
| Raspberry Pi 4  | 5V      | 2.5-3A    | Via USB-C (raccomandato)          |
| Arduino Nano    | 5V      | 200mA     | Alimentato da Raspberry via USB   |
| RPLIDAR         | 5V      | 400-500mA | Alimentato da Raspberry via USB   |
| Encoder (x2)    | 5V      | 50mA each | Alimentati da Arduino 5V pin      |

---

## RPLIDAR - Setup

### Collegamenti RPLIDAR

```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃                         RPLIDAR A1/A2                             ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛

                         ┌─────────────┐
                         │             │
                      ┌──┴──┐       ┌──┴──┐
                      │  O  │       │  O  │  ← Scanning Head
                      │  O  │       │  O  │     (Rotates 360°)
                      └──┬──┘       └──┬──┘
                         │             │
                         └──────┬──────┘
                                │
                                │ Cable
                                │
                         ┌──────┴──────┐
                         │             │
                         │  ADAPTER    │  ← USB Adapter Board
                         │             │     (Powers + Serial)
                         └──────┬──────┘
                                │
                                │ USB Cable (Type A)
                                │
                         ┌──────┴──────┐
                         │             │
                         │ RASPBERRY   │
                         │   PI USB    │
                         │             │
                         └─────────────┘

    Collegamento:
    1. Connetti il cavo LIDAR all'adapter board
    2. Connetti l'adapter USB al Raspberry Pi
    3. LIDAR apparirà come /dev/ttyUSB1 (se Arduino è /dev/ttyUSB0)
    4. Non serve alimentazione esterna (alimentato via USB)
```

---

## Checklist Collegamento Passo-Passo

### Fase 1: Preparazione Componenti

- [ ] Verifica di avere tutti i componenti
- [ ] Prepara cavi dupont maschio-femmina
- [ ] Prepara cavi per alimentazione (sezione adeguata)
- [ ] Verifica polarità batteria
- [ ] Testa step-down converter (imposta 5V output)

### Fase 2: Collegamenti Arduino → L298N

- [ ] GND Arduino → GND L298N
- [ ] D5 → ENA (Enable Motor L)
- [ ] D6 → IN1 (Motor L - Dir 1)
- [ ] D7 → IN2 (Motor L - Dir 2)
- [ ] D9 → ENB (Enable Motor R)
- [ ] D10 → IN3 (Motor R - Dir 1)
- [ ] D11 → IN4 (Motor R - Dir 2)

### Fase 3: Collegamenti Encoder → Arduino

**Encoder Sinistro:**
- [ ] VCC Encoder L → Arduino 5V
- [ ] GND Encoder L → Arduino GND
- [ ] ChA Encoder L → Arduino D2
- [ ] ChB Encoder L → Arduino D4

**Encoder Destro:**
- [ ] VCC Encoder R → Arduino 5V
- [ ] GND Encoder R → Arduino GND
- [ ] ChA Encoder R → Arduino D3
- [ ] ChB Encoder R → Arduino D8

### Fase 4: Collegamenti Motori

- [ ] Motor L (+) → L298N OUT1
- [ ] Motor L (-) → L298N OUT2
- [ ] Motor R (+) → L298N OUT3
- [ ] Motor R (-) → L298N OUT4
- [ ] Encoder L montato su albero Motor L
- [ ] Encoder R montato su albero Motor R

### Fase 5: Alimentazione

- [ ] Batteria (+) → L298N +12V
- [ ] Batteria (-) → L298N GND
- [ ] Batteria (+) → Step-Down Input (+)
- [ ] Batteria (-) → Step-Down Input (-)
- [ ] Step-Down Output (+5V) → Raspberry Pi (USB-C o GPIO Pin 4)
- [ ] Step-Down Output (GND) → Raspberry Pi (GPIO Pin 6)
- [ ] **Verifica che tutti i GND siano collegati insieme**

### Fase 6: Collegamenti USB

- [ ] Arduino Nano USB → Raspberry Pi USB (apparirà come /dev/ttyUSB0)
- [ ] RPLIDAR USB → Raspberry Pi USB (apparirà come /dev/ttyUSB1)
- [ ] Camera USB → Raspberry Pi USB (apparirà come /dev/video0)

### Fase 7: Verifica Finale

- [ ] Verifica visiva di tutti i collegamenti
- [ ] Controlla polarità batteria
- [ ] Controlla polarità step-down
- [ ] Verifica che jumper 5V regulator su L298N sia RIMOSSO
- [ ] Verifica che encoder siano montati correttamente
- [ ] Verifica che motori ruotino liberamente

### Fase 8: Test Prima dell'Accensione

- [ ] Misura output step-down con multimetro (deve essere 5V)
- [ ] Controlla continuità dei GND con multimetro
- [ ] Controlla che non ci siano cortocircuiti
- [ ] Prepara piano di emergenza (interruttore, estintore)

### Fase 9: Primo Accensione

- [ ] Collega batteria
- [ ] Verifica LED su Raspberry Pi (si accende?)
- [ ] Verifica LED su Arduino (si accende via USB?)
- [ ] Verifica RPLIDAR (motore gira?)
- [ ] **Tieni il robot sollevato (ruote non toccano terra)**

### Fase 10: Test Software

- [ ] Test comunicazione Arduino: `ls -l /dev/ttyUSB0`
- [ ] Test comunicazione LIDAR: `ls -l /dev/ttyUSB1`
- [ ] Test encoder con Serial Monitor Arduino
- [ ] Test movimento motori (con robot sollevato)
- [ ] Test LIDAR: `ros2 launch robot_ros lidar.launch.py`

---

## Suggerimenti di Sicurezza

⚠️ **PRIMA DI ALIMENTARE:**

1. **Verifica Polarità:** Controlla 3 volte polarità batteria e step-down
2. **Ground Comune:** Tutti i GND devono essere collegati insieme
3. **Nessun Cortocircuito:** Usa multimetro per verificare
4. **Cavi Adeguati:** Usa cavi di sezione corretta (18AWG minimo per motori)
5. **Robot Sollevato:** Primi test sempre con robot sollevato
6. **Batteria Carica:** Usa batteria carica al 50-100%
7. **Ventilazione:** L298N può scaldarsi, assicura ventilazione
8. **Interruttore:** Aggiungi interruttore sulla batteria per spegnimento rapido

⚠️ **DURANTE I TEST:**

1. **Mai lasciare incustodito** con batteria collegata
2. **Temperatura L298N:** Controlla che non scaldi troppo
3. **Movimento inaspettato:** Pronto a scollegare batteria
4. **Timeout Arduino:** Verifica che funzioni (motori si fermano dopo 1s)

---

## Troubleshooting Visivo

### LED di Stato

| LED             | Posizione      | Stato Normale        | Se Spento                    |
|-----------------|----------------|----------------------|------------------------------|
| Raspberry PWR   | Raspberry Pi   | Verde fisso          | Verifica alimentazione 5V    |
| Raspberry ACT   | Raspberry Pi   | Verde lampeggiante   | Sistema non boot             |
| Arduino PWR     | Arduino Nano   | Verde fisso          | Verifica USB da Raspberry    |
| L298N PWR       | L298N          | LED rosso acceso     | Verifica alimentazione 12V   |
| RPLIDAR Motor   | RPLIDAR        | Girare continuo      | Verifica USB + Driver        |

### Verifiche con Multimetro

| Punto di Test              | Valore Atteso | Se Diverso                    |
|----------------------------|---------------|-------------------------------|
| Batteria voltage           | 11.1-12.6V    | Ricarica batteria             |
| Step-Down output           | 5.0V ±0.1V    | Regola trimmer step-down      |
| L298N +12V pin             | 11.1-12.6V    | Verifica connessione batteria |
| Arduino 5V pin             | 5.0V          | Problema USB Raspberry        |
| Encoder VCC                | 5.0V          | Verifica collegamento Arduino |
| Ground continuity          | 0Ω            | Controlla collegamenti GND    |

---

**Documento creato:** 2026-02-16
**Versione:** 1.0
**Testato su:** ROS2 Humble, Ubuntu 22.04
