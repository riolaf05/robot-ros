# Wiring Diagram - Robot Mobile ROS2

Questa guida descrive le connessioni hardware per il robot mobile con ROS2 Control.

## Componenti Hardware Richiesti

### Componenti Principali
- **Raspberry Pi 4** (minimo 2GB RAM, consigliato 4GB)
- **Arduino Nano** (ATmega328P)
- **L298N Motor Driver Module**
- **2x Motori DC con encoder** (12V raccomandato)
- **2x Ruote** (diametro ~10cm)
- **RPLIDAR A1/A2** (o compatibile)
- **Camera USB** (opzionale, per visione)
- **Batteria LiPo/Li-ion** (11.1V o 12V, capacità consigliata 2200-5000mAh)
- **Step-down converter** (12V → 5V, per alimentare Raspberry e Arduino)
- **Caster wheel** (ruota pazza per supporto anteriore)

### Componenti Aggiuntivi
- Cavi dupont maschio-femmina
- Cavi per alimentazione
- Breadboard (opzionale, per prototipazione)
- Connettori XT60 o simili per batteria

## Schema Collegamenti

### 1. Arduino Nano ↔ L298N Motor Driver

#### Motore Sinistro
```
Arduino Nano          L298N
----------------------------------
Pin D5 (PWM)    →    ENA (Enable A)
Pin D6          →    IN1 (Input 1)
Pin D7          →    IN2 (Input 2)
```

#### Motore Destro
```
Arduino Nano          L298N
----------------------------------
Pin D9 (PWM)    →    ENB (Enable B)
Pin D10         →    IN3 (Input 3)
Pin D11         →    IN4 (Input 4)
```

### 2. Arduino Nano ↔ Encoder Motori

#### Encoder Motore Sinistro
```
Arduino Nano          Encoder
----------------------------------
Pin D2 (INT0)   →    Channel A
Pin D4          →    Channel B
GND             →    GND
5V              →    VCC
```

#### Encoder Motore Destro
```
Arduino Nano          Encoder
----------------------------------
Pin D3 (INT1)   →    Channel A
Pin D8          →    Channel B
GND             →    GND
5V              →    VCC
```

**Nota:** I pin D2 e D3 sono pin di interrupt hardware, necessari per lettura veloce degli encoder.

### 3. L298N ↔ Motori DC

```
L298N                 Motore
----------------------------------
OUT1, OUT2      →    Motore Sinistro (+/-)
OUT3, OUT4      →    Motore Destro (+/-)
```

**Nota:** Se il motore gira al contrario, inverti i collegamenti al motore.

### 4. Arduino Nano ↔ Raspberry Pi

```
Arduino Nano          Raspberry Pi
----------------------------------
USB Port        →    USB Port (qualsiasi porta USB)
```

L'Arduino viene alimentato tramite USB dal Raspberry Pi e comunica via seriale (57600 baud).
Apparirà come `/dev/ttyUSB0` sul Raspberry Pi.

### 5. RPLIDAR ↔ Raspberry Pi

```
RPLIDAR               Raspberry Pi
----------------------------------
USB Port        →    USB Port
```

Il LIDAR apparirà come `/dev/ttyUSB1` sul Raspberry Pi (o altro se l'Arduino non è collegato).

### 6. Camera USB ↔ Raspberry Pi

```
Camera USB            Raspberry Pi
----------------------------------
USB Port        →    USB Port
```

La camera apparirà come `/dev/video0` sul Raspberry Pi.

### 7. Alimentazione

#### Schema Alimentazione Generale
```
Batteria 12V
    │
    ├──→ L298N (12V)
    │       │
    │       └──→ Motori DC (12V)
    │
    └──→ Step-Down Converter (12V → 5V)
            │
            ├──→ Raspberry Pi 4 (5V, GPIO o USB-C)
            └──→ RPLIDAR (5V, se necessario)
```

#### Dettagli Alimentazione L298N
```
L298N
----------------------------------
+12V          →    Batteria 12V (+)
GND           →    Batteria 12V (-)
+5V (jumper)  →    NON USARE (rimuovi jumper se presente)
```

**IMPORTANTE:**
- Rimuovi il jumper del regolatore 5V sul L298N per evitare sovraccarichi
- Il L298N necessita di una batteria capace di fornire almeno 2-3A
- Usa un step-down di qualità per il Raspberry (almeno 3A di capacità)

#### Alimentazione Raspberry Pi
Hai due opzioni per alimentare il Raspberry Pi:

**Opzione 1: USB-C (Raccomandato)**
```
Step-Down 5V → Cavo USB-C → Raspberry Pi
```

**Opzione 2: GPIO Pins (Attenzione: Nessuna protezione)**
```
Step-Down 5V+ → Pin 4 (5V)
Step-Down GND → Pin 6 (GND)
```

⚠️ **Attenzione Opzione 2:** Alimentando via GPIO bypassa tutte le protezioni. Assicurati che il voltage sia esattamente 5V.

## Schema Visuale Collegamenti

```
                    RASPBERRY PI 4
                    ┌─────────────┐
                    │             │
                    │   USB  USB  │ USB  USB
                    └──┬────┬─────┴───┬────┬────┘
                       │    │         │    │
          Arduino ◄────┘    │         │    └────► Camera
          Nano              │         │
                            │         └─────────► LIDAR
                            │
                    ┌───────▼──────┐
                    │   L298N      │
                    │ Motor Driver │
                    └──┬─────┬─────┘
                       │     │
            Motor L ◄──┘     └──► Motor R
            + Encoder           + Encoder


                    POWER SUPPLY
                    ┌────────────┐
                    │  12V LiPo  │
                    │  Battery   │
                    └──┬─────┬───┘
                       │     │
                   L298N  Step-Down
                             5V
                             │
                          Rasp Pi
```

## Tabella Pin Arduino Completa

| Pin Arduino | Funzione          | Collegamento        |
|-------------|-------------------|---------------------|
| D2          | Encoder L - Ch A  | Encoder Sinistro A  |
| D3          | Encoder R - Ch A  | Encoder Destro A    |
| D4          | Encoder L - Ch B  | Encoder Sinistro B  |
| D5          | Motor L Enable    | L298N ENA           |
| D6          | Motor L IN1       | L298N IN1           |
| D7          | Motor L IN2       | L298N IN2           |
| D8          | Encoder R - Ch B  | Encoder Destro B    |
| D9          | Motor R Enable    | L298N ENB           |
| D10         | Motor R IN1       | L298N IN3           |
| D11         | Motor R IN2       | L298N IN4           |
| 5V          | Power Encoder     | Encoder VCC (x2)    |
| GND         | Ground            | Encoder GND (x2)    |

## Note Importanti

### Encoder
- **Counts per Revolution (CPR):** Lo sketch Arduino è configurato per 374 CPR. Questo valore dipende dal tuo encoder specifico. Verifica le specifiche del tuo encoder e aggiorna il valore `ENCODER_CPR` nello sketch se necessario.
- Se il tuo encoder ha un riduttore (gearbox), moltiplica i CPR base per il rapporto di riduzione
  - Esempio: Encoder 11 CPR × Riduttore 34:1 = 374 CPR totali

### Direzione Motori
- Se un motore gira al contrario rispetto all'altro:
  1. Inverti fisicamente i cavi del motore (OUT1 ↔ OUT2 o OUT3 ↔ OUT4)
  2. OPPURE modifica lo sketch Arduino invertendo la logica di HIGH/LOW

### Porte Seriali
- Arduino: `/dev/ttyUSB0`
- LIDAR: `/dev/ttyUSB1`
- Se le porte cambiano, aggiorna il parametro `device` nel file `description/ros2_control.xacro`

Per identificare le porte:
```bash
ls -l /dev/ttyUSB*
# o
dmesg | grep tty
```

### Alimentazione
- **Mai condividere ground tra alimentazione motori e Raspberry direttamente** (usa convertitori isolati se possibile)
- Ground comune tra tutti i componenti è OK se tutto deriva dalla stessa batteria
- Usa cavi di sezione adeguata per i motori (almeno 18AWG)

### Test Sicurezza
Prima di assemblare il robot completo:
1. Testa i motori senza le ruote montate
2. Verifica la direzione degli encoder
3. Assicurati che il timeout dello sketch Arduino funzioni (i motori si fermano se non ricevono comandi)

## Troubleshooting

### Motori non si muovono
- Verifica alimentazione L298N (12V)
- Controlla i collegamenti IN1-IN4
- Misura il voltage sul pin Enable (dovrebbe essere PWM)

### Encoder non conta
- Verifica alimentazione encoder (5V)
- Testa con Serial Monitor Arduino (sketch apposito)
- Controlla i pull-up resistor (se non integrati nell'encoder)

### Arduino non comunica
- Verifica porta seriale: `ls -l /dev/ttyUSB*`
- Controlla permessi: `sudo usermod -a -G dialout $USER` (riavvio richiesto)
- Testa con: `sudo minicom -D /dev/ttyUSB0 -b 57600`

### LIDAR non funziona
- Verifica porta seriale
- Controlla il package installato: `ros2 pkg list | grep rplidar`
- Testa standalone: `ros2 launch robot_ros lidar.launch.py`
