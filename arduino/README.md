# Arduino Sketch per Hardware Interface ROS2

Questo sketch Arduino fornisce l'hardware interface per comunicare con ROS2 Control tramite il plugin DiffDriveArduino.

## Hardware Supportato

- **Arduino Nano** (ATmega328P)
- **L298N Motor Driver**
- **2x Motori DC con encoder**

## Funzionalità

- ✅ Controllo PWM di 2 motori DC tramite L298N
- ✅ Lettura encoder per odometria
- ✅ Comunicazione seriale con ROS2 (57600 baud)
- ✅ Timeout di sicurezza (ferma i motori se non riceve comandi)
- ✅ Compatibile con DiffDriveArduino plugin

## Collegamenti Hardware

Vedi [docs/WIRING.md](../docs/WIRING.md) per lo schema completo.

### Pin Arduino Utilizzati

| Pin | Funzione          | Collegamento        |
|-----|-------------------|---------------------|
| D2  | Encoder L - Ch A  | Interrupt           |
| D3  | Encoder R - Ch A  | Interrupt           |
| D4  | Encoder L - Ch B  | Digital Input       |
| D5  | Motor L Enable    | PWM Output          |
| D6  | Motor L IN1       | Digital Output      |
| D7  | Motor L IN2       | Digital Output      |
| D8  | Encoder R - Ch B  | Digital Input       |
| D9  | Motor R Enable    | PWM Output          |
| D10 | Motor R IN1       | Digital Output      |
| D11 | Motor R IN2       | Digital Output      |

## Configurazione

### 1. Encoder CPR (Counts Per Revolution)

**IMPORTANTE:** Prima di caricare lo sketch, devi configurare il valore `ENCODER_CPR` secondo il tuo encoder specifico.

Trova questa riga nello sketch (linea ~30):
```cpp
const int ENCODER_CPR = 374;  // <-- MODIFICA QUESTO VALORE
```

Come calcolare il CPR corretto:
- Se l'encoder è **senza riduttore**: Usa il valore CPR dalle specifiche (es. 11, 20, 400, ecc.)
- Se l'encoder ha un **riduttore integrato**: Moltiplica CPR base × rapporto riduzione
  - Esempio: Encoder 11 CPR con riduttore 34:1 → 11 × 34 = **374 CPR**

### 2. Direzione Motori

Se un motore gira al contrario:
- **Hardware:** Inverti i cavi del motore sul L298N (scambia OUT1 con OUT2 o OUT3 con OUT4)
- **Software:** Modifica le funzioni `updateMotors()` invertendo HIGH/LOW

## Upload dello Sketch

### 1. Installazione Arduino IDE

Scarica da: https://www.arduino.cc/en/software

### 2. Configurazione

1. Apri `diffdrive_arduino.ino` in Arduino IDE
2. Seleziona:
   - **Board:** Arduino Nano
   - **Processor:** ATmega328P (Old Bootloader)
   - **Port:** La porta USB corretta
3. **Modifica `ENCODER_CPR`** se necessario
4. Clicca "Verify" per compilare
5. Clicca "Upload" per caricare

### 3. Test

Apri Serial Monitor (57600 baud) e invia:
```
<e>
```

Dovresti ricevere:
```
L:0,R:0
```

Ruota manualmente le ruote e verifica che i conteggi cambino.

## Protocollo Comunicazione

Lo sketch comunica con ROS2 tramite seriale usando un protocollo semplice:

### Comandi in Ingresso (da ROS2 → Arduino)

Formato: `<COMANDO>`

Esempi:
- `<L:1.5,R:1.5>` - Imposta velocità left=1.5 rad/s, right=1.5 rad/s
- `<L:-0.5,R:0.5>` - Ruota a sinistra
- `<e>` - Richiesta encoder (per debug)

### Feedback in Uscita (da Arduino → ROS2)

Formato: `L:count,R:count\n`

Esempio:
```
L:1234,R:1200
```

Questo feedback viene inviato automaticamente a 10Hz e viene letto da DiffDriveArduino per calcolare l'odometria.

## Funzionamento

### 1. Conversione Velocità → PWM

Lo sketch converte la velocità angolare (rad/s) in valori PWM (0-255):
```cpp
PWM = abs(velocity) * 51  // Assumendo vel_max = 5 rad/s
```

Limiti:
- Velocità massima: ±5 rad/s
- PWM range: 0-255
- Deadzone: ±0.05 rad/s (per evitare jitter)

### 2. Timeout Sicurezza

Se non riceve comandi per più di 1 secondo:
```cpp
const unsigned long COMMAND_TIMEOUT = 1000; // ms
```
I motori vengono automaticamente fermati per sicurezza.

### 3. Lettura Encoder

Gli encoder utilizzano interrupt hardware (pin D2 e D3) per lettura veloce e precisa:
```cpp
attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, RISING);
attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, RISING);
```

## Troubleshooting

### Problema: Encoder non conta

**Possibili cause:**
1. Cavi encoder invertiti o scollegati
2. Encoder non alimentato (verifica 5V)
3. CPR configurato male
4. Interrupt non funzionanti

**Test:**
```cpp
// Aggiungi nel loop() per debug
Serial.print("Left: ");
Serial.print(leftEncoderCount);
Serial.print(" Right: ");
Serial.println(rightEncoderCount);
delay(100);
```

Ruota manualmente le ruote e verifica che i conteggi aumentino.

### Problema: Motori non girano

**Possibili cause:**
1. L298N non alimentato (verifica 12V)
2. Pin L298N collegati male
3. PWM troppo basso (aumenta il moltiplicatore)

**Test:**
```cpp
// Aggiungi nel setup() per test statico
digitalWrite(LEFT_MOTOR_IN1, HIGH);
digitalWrite(LEFT_MOTOR_IN2, LOW);
analogWrite(LEFT_MOTOR_EN, 200);  // PWM fisso
delay(2000);
```

### Problema: Motori girano al contrario

**Soluzione rapida:**
Inverti fisicamente i cavi sul L298N (OUT1 ↔ OUT2)

**Soluzione software:**
Modifica la funzione `updateMotors()`, invertendo HIGH/LOW per quel motore.

### Problema: Comunicazione seriale non funziona

**Verifica:**
1. Baud rate corretto (57600)
2. Arduino riconosciuto: `ls -l /dev/ttyUSB*` (su Linux)
3. Permessi seriale: `sudo usermod -a -G dialout $USER`

**Test:**
```bash
# Linux
minicom -D /dev/ttyUSB0 -b 57600

# Invia: <e>
# Attendi: L:0,R:0
```

## Modifiche Avanzate

### Cambio Pin

Se devi usare pin diversi, modifica le define:
```cpp
#define LEFT_MOTOR_EN   5   // <- Cambia qui
#define LEFT_MOTOR_IN1  6
// ...
```

**Nota:** I pin encoder devono rimanere su D2/D3 (interrupt hardware).

### PID Control

Lo sketch attuale usa controllo open-loop (senza feedback). Per aggiungere PID:
1. Calcola velocità attuale da encoder
2. Implementa controller PID
3. Aggiusta PWM in base all'errore

Esempio libreria: [Arduino PID Library](https://github.com/br3ttb/Arduino-PID-Library)

### Cambio Velocità Massima

Modifica il fattore di conversione in `updateMotors()`:
```cpp
int leftPWM = constrain(abs(leftWheelSetpoint) * 51, 0, 255);
//                                                  ^^ modifica questo
```

Formula: `255 / velocità_max_desiderata`
- Vel max 5 rad/s → 255/5 = 51
- Vel max 10 rad/s → 255/10 = 25.5

## Integrazione con ROS2

Questo sketch è compatibile con:
- **DiffDriveArduino** plugin per ros2_control
- Configurato in `description/ros2_control.xacro`
- Usa porta seriale `/dev/ttyUSB0` a 57600 baud

ROS2 si aspetta:
- Feedback encoder formato: `L:count,R:count\n`
- Comandi formato: `<L:vel,R:vel>`
- CPR deve corrispondere al valore nel file xacro

## Risorse

- [DiffDriveArduino GitHub](https://github.com/joshnewans/diffdrive_arduino)
- [ros2_control Documentation](https://control.ros.org/)
- [Arduino Interrupts](https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/)
- [L298N Datasheet](https://www.sparkfun.com/datasheets/Robotics/L298_H_Bridge.pdf)

## Licenza

Questo sketch è open source. Sentiti libero di modificarlo per le tue esigenze.
