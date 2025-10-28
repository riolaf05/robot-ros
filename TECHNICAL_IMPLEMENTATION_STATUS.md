# ROBOT-ROS - Stato Implementazione Tecnica
**Data ultimo aggiornamento**: 28 Ottobre 2025  
**Branch**: ros2-control-test  
**ROS2 Version**: Humble  

---

## 🎯 EXECUTIVE SUMMARY

### Punto di Partenza
- **Obiettivo iniziale**: Creare hardware interface ROS2 per robot differenziale con Arduino
- **Firmware target**: ROSArduinoBridge (https://github.com/riolaf05/ros_arduino_bridge)
- **Hardware**: Arduino Uno + L298N Motor Driver + motori DC
- **Framework**: ROS2 Humble + ros2_control

### Punto di Arrivo Attuale
- ✅ **Hardware Interface**: Completamente implementato e funzionante
- ✅ **Comunicazione Seriale**: Robusta e corretta (risolti problemi di flush/buffering)
- ✅ **Protocollo Arduino**: ROSArduinoBridge protocol implementato correttamente
- ✅ **Controller Stack**: diff_drive_controller e joint_state_broadcaster attivi
- ✅ **Sistema Launch**: Script automatico e launch files funzionanti
- ❌ **PROBLEMA ATTUALE**: Motori non rispondono ai comandi teleop (segnali arrivano ma PWM=0)
- ❌ **MANCANTE**: Lettura odometria da Arduino (non ancora implementata)

### Stato Debug
- Arduino riceve comandi seriali correttamente (verificato con echo diretto)
- Sistema ROS2 funziona (controller attivi, topic pubblicano)
- Hardware interface riceve comandi da teleop ma genera sempre PWM=0
- Necessaria investigazione sulla catena: teleop → controller → hardware_interface

---

## 🔧 ARCHITETTURA SISTEMA

### Stack Software
```
teleop_twist_keyboard (input utente)
         ↓ 
    /diff_drive_controller/cmd_vel_unstamped (topic)
         ↓
    diff_drive_controller (ros2_control)
         ↓  
    ArduinoHardwareInterface (custom)
         ↓
    Comunicazione Seriale (/dev/ttyACM0, 57600 baud)
         ↓
    Arduino ROSArduinoBridge Firmware
         ↓
    L298N Motor Driver → Motori DC
```

### Componenti Implementati

#### 1. Hardware Interface (`arduino_hardware_interface.cpp`)
**Localizzazione**: `robot_ros/hardware_interface/`

**Funzionalità Implementate**:
- ✅ Connessione seriale robusta con termios configurazione completa
- ✅ Protocollo ROSArduinoBridge ("o left_pwm right_pwm" format)  
- ✅ Retry logic per gestire "Partial write" errors
- ✅ Flush corretto con `fsync()` per invio immediato comandi
- ✅ Conversione velocity→PWM tramite `velocity_to_pwm()`
- ✅ Debug logging per diagnostica
- ❌ Lettura odometria (non implementata)

**Parametri Chiave**:
```cpp
device_: "/dev/ttyACM0"
baud_rate_: 57600
max_velocity: 5.0 rad/s  // per conversione PWM
wheel_radius_: 0.065m
wheel_separation_: 0.17m
```

#### 2. Controller Configuration (`arduino_controllers_fixed.yaml`)
**Localizzazione**: `config/`

**Controller Attivi**:
- `diff_drive_controller`: Conversione Twist→wheel velocities
- `joint_state_broadcaster`: Pubblicazione stato joint

**Parametri Critici**:
```yaml
update_rate: 10  # Hz
wheel_separation: 0.17  # m
wheel_radius: 0.0325   # m (NOTA: diverso da hardware interface!)
linear.x.max_velocity: 1.0   # m/s
angular.z.max_velocity: 1.0  # rad/s
```

#### 3. Launch System
**Script Principale**: `run_robot_test.sh`
- Avvio automatico completo del sistema
- Sourcing ambiente
- Launch sequenziale con attesa controller
- Istruzioni teleop integrate

**Launch File**: `arduino_persistent.launch.py`
- Configurazione completa nodi ROS2
- Loading controller manager e hardware interface
- Spawn automatico controller

---

## 🐛 PROBLEMI RISOLTI

### 1. Problema: "Partial Write" Errori Seriali
**Sintomi**: Comandi seriali non inviati completamente, errori di comunicazione
**Causa Root**: Configurazione termios insufficiente, buffer management scorretto
**Soluzione Implementata**:
```cpp
// PRIMA (scorretto)
tcflush(serial_fd_, TCOFLUSH);  // Scartava dati invece di inviarli

// DOPO (corretto) 
fsync(serial_fd_);  // Forza invio immediato buffer
```

### 2. Problema: Ritardo Comandi Arduino "Parecchi Secondi"  
**Sintomi**: Arduino riceveva comandi con 5-10 secondi di ritardo
**Causa Root**: Buffer seriale non flushato correttamente
**Soluzione**: Sostituzione `tcflush()` con `fsync()` + configurazione termios robusta

### 3. Problema: Protocollo Arduino Scorretto
**Sintomi**: Comandi non riconosciuti da ROSArduinoBridge
**Causa Root**: Formato comando errato 
**Soluzione**: Implementato protocollo corretto `"o left_pwm right_pwm"`

### 4. Problema: Configurazione Seriale Instabile
**Sintomi**: Connessione seriale intermittente, lock del sistema
**Causa Root**: Flag O_NDELAY e configurazione termios incompleta
**Soluzione**: Configurazione termios completa e robusta

---

## ❌ PROBLEMI ATTUALI (NON RISOLTI)

### 🔥 CRITICO: Motori Non Rispondono a Teleop

**Sintomi Osservati**:
- ✅ teleop_twist_keyboard riceve input utente correttamente
- ✅ Arduino riceve comandi seriali (verificato con echo diretto: `echo "o 100 100" > /dev/ttyACM0`)
- ✅ Hardware interface log mostra: "Motor PWM command sent: o 0 0"
- ❌ **PROBLEMA**: PWM values sempre = 0 anche con input teleop attivo

**Log Diagnostici Attesi**:
```bash
# Se teleop funziona, dovremmo vedere:
[ArduinoHardwareInterface]: RECEIVED VELOCITIES: Left=0.XXX rad/s, Right=0.XXX rad/s

# Invece vediamo sempre:
[ArduinoHardwareInterface]: RECEIVED VELOCITIES: Left=0.000 rad/s, Right=0.000 rad/s
```

**Possibili Cause da Investigare**:

1. **Topic Remapping Issue**: 
   - Teleop pubblica su topic diverso da quello che controller ascolta
   - Verifica: `ros2 topic list | grep cmd_vel`
   - Test: `ros2 topic echo /diff_drive_controller/cmd_vel_unstamped`

2. **Controller Non Riceve Comandi**:
   - diff_drive_controller non configurato correttamente
   - Joint names mismatch tra URDF e controller config
   - Verifica: `ros2 control list_controllers`

3. **Hardware Interface Command Interface**:
   - `hw_commands_` array non popolato da controller
   - Command interface names incorretti
   - Export command interfaces configuration issue

4. **Conversione Velocity→PWM**:
   - `max_velocity = 5.0 rad/s` troppo alta (comandi reali < soglia)
   - Funzione `velocity_to_pwm()` logica errata
   - Scaling factor incorretto

5. **URDF Joint Configuration**:
   - Nomi joint non corrispondenti tra URDF e controller
   - Joint type o limits incorretti

**Action Items per Debug**:
```bash
# 1. Verifica pubblicazione teleop
ros2 topic echo /diff_drive_controller/cmd_vel_unstamped --once

# 2. Test comando diretto controller  
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.2}}" -1

# 3. Verifica controller attivi
ros2 control list_controllers

# 4. Debug hardware interface con verbose logging
ros2 launch robot_ros arduino_persistent.launch.py --ros-args --log-level DEBUG

# 5. Verifica joint names URDF vs controller config
grep -r "wheel_joint" config/ description/
```

### 🔶 MANCANTE: Lettura Odometria Arduino

**Stato**: Non implementato
**Necessità**: Feedback posizione per navigation stack
**Implementazione Richiesta**:

1. **Firmware Arduino**: Lettura encoder e invio dati posizione
2. **Hardware Interface**: Parsing risposta Arduino per odometria  
3. **Read Function**: Aggiornamento `hw_positions_[]` e `hw_velocities_[]` con dati reali

**Protocollo ROSArduinoBridge per Odometria**:
```bash
# Comando richiesta encoder
echo "e" > /dev/ttyACM0

# Risposta attesa (esempio)  
"1234 5678"  # left_encoder_count right_encoder_count
```

---

## 🔍 CONFIGURAZIONI TECNICHE DETTAGLIATE

### Hardware Interface - Parametri Critici

**File**: `arduino_hardware_interface.cpp`
```cpp
class ArduinoHardwareInterface {
    // Comunicazione seriale
    std::string device_ = "/dev/ttyACM0";
    int baud_rate_ = 57600;
    double timeout_ = 1.0;  // seconds
    
    // Parametri robot  
    double wheel_radius_ = 0.065;      // 65mm
    double wheel_separation_ = 0.17;   // 170mm
    
    // Conversione velocity→PWM
    const double max_velocity = 5.0;   // rad/s
    
    // Command/State arrays (size=2, left/right wheels)
    std::vector<double> hw_commands_;   // Input da controller
    std::vector<double> hw_positions_;  // Output per stato
    std::vector<double> hw_velocities_; // Output per stato
};
```

### Controller Configuration Details

**File**: `config/arduino_controllers_fixed.yaml`
```yaml
# NOTA CRITICA: Discrepanza parametri ruote!
robot_ros:
  hardware_interface:
    wheel_radius_: 0.065      # Hardware interface
    wheel_separation_: 0.17   # Hardware interface

diff_drive_controller:
  ros__parameters:
    wheel_radius: 0.0325      # Controller (DIVERSO!)
    wheel_separation: 0.17    # Controller (uguale)
    
    # Joint mapping (CRITICO per funzionamento)
    left_wheel_names: ["base_left_wheel_joint"]
    right_wheel_names: ["base_right_wheel_joint"]
```

**⚠️ DISCREPANZA IDENTIFICATA**: wheel_radius diverso tra hardware interface (0.065) e controller (0.0325)!

### URDF Robot Description

**File**: `description/robot.urdf.xacro`
**Joint Names Critici**:
```xml
<joint name="base_left_wheel_joint" type="continuous">
<joint name="base_right_wheel_joint" type="continuous">
```

**Verifica Compatibilità**: I nomi joint URDF devono matchare esattamente con controller config.

### Serial Communication Protocol

**ROSArduinoBridge Commands**:
```bash
# Motor control (implementato)
"o LEFT_PWM RIGHT_PWM"  # Esempio: "o 100 -150"

# Encoder request (da implementare)  
"e"                     # Request encoder values

# Baudrate test (implementato)
"b"                     # Responds with baudrate

# Reset encoders (implementato)
"r"                     # Reset encoder counters to 0
```

---

## 📋 NEXT STEPS - AZIONI IMMEDIATE

### Priority 1: RISOLVI PROBLEMA MOTORI (CRITICO)

1. **Diagnosi Command Flow**:
   ```bash
   # Traccia completa comando da teleop a Arduino
   ros2 topic echo /diff_drive_controller/cmd_vel_unstamped  # Verifica topic
   ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.5}}" -1  # Test diretto
   ```

2. **Debug Hardware Interface**:
   - Verifica che `hw_commands_[0]` e `hw_commands_[1]` ricevono valori != 0
   - Add temporary logging in `write()` function per vedere valori raw
   - Test conversione `velocity_to_pwm()` con valori noti

3. **Risolvi Discrepanza Parametri**:
   - Uniforma `wheel_radius` tra hardware interface e controller
   - Verifica impatto su calcoli di conversione

4. **Verifica Joint Configuration**:
   - Conferma match perfetto nomi joint URDF ↔ controller config ↔ hardware interface
   - Test controller configuration con `ros2 control` commands

### Priority 2: IMPLEMENTA ODOMETRIA

1. **Extend Firmware Arduino**:
   - Aggiungi lettura encoder e comando "e" response
   
2. **Hardware Interface Read Function**:
   - Implementa parsing encoder data da Arduino
   - Calcola posizione e velocity da encoder counts
   - Update `hw_positions_[]` e `hw_velocities_[]`

### Priority 3: TESTING E VALIDATION

1. **End-to-end Testing**:
   - Teleop → movement → odometry feedback loop
   
2. **Performance Tuning**:
   - Ottimizza update rate (attuale: 10Hz)
   - Tuning PID controller se necessario

---

## 📁 FILES MODIFICATI/CREATI

### Core Implementation Files
- `robot_ros/hardware_interface/arduino_hardware_interface.hpp`
- `robot_ros/hardware_interface/arduino_hardware_interface.cpp` ⭐ MAIN
- `config/arduino_controllers_fixed.yaml` ⭐ MAIN
- `launch/arduino_persistent.launch.py`
- `run_robot_test.sh` ⭐ ENTRY POINT

### Configuration Files  
- `description/robot.urdf.xacro` (joint names)
- `package.xml` (dependencies)
- `CMakeLists.txt` (build configuration)

### Documentation
- `README.md` (updated with complete setup guide)
- `TECHNICAL_IMPLEMENTATION_STATUS.md` (questo file)

---

## 🔧 ENVIRONMENT E DEPENDENCIES

### ROS2 Packages Required
```xml
<depend>rclcpp</depend>
<depend>hardware_interface</depend>
<depend>controller_manager</depend>  
<depend>diff_drive_controller</depend>
<depend>joint_state_broadcaster</depend>
<depend>teleop_twist_keyboard</depend>
<depend>robot_state_publisher</depend>
```

### System Dependencies
- Ubuntu 22.04 LTS
- ROS2 Humble
- Arduino IDE (firmware upload)
- Serial communication tools (`echo`, `stty`)

### Hardware Requirements  
- Arduino Uno/Mega
- L298N Motor Driver o equivalente
- 2x Motori DC con encoder (opzionali per ora)
- USB cable per comunicazione seriale
- Power supply per motori (separate da Arduino)

---

## 💡 CONTEXT PER FUTURE AI SESSIONS

**Se stai riprendendo questo progetto**:

1. **Current Status**: Sistema quasi completo, hardware interface funziona ma motori non rispondono a teleop
2. **Main Issue**: Debug perché `hw_commands_[]` array riceve sempre 0.0 values nonostante teleop input
3. **Files to check first**: `arduino_hardware_interface.cpp` (funzione write), `arduino_controllers_fixed.yaml` (parametri controller)
4. **Known working**: Arduino serial communication, controller spawning, topic publishing
5. **Priority**: Fix command flow teleop→controller→hardware interface, poi implementare odometria

**Debug commands ready to use**:
```bash
cd /home/rosario/robot-ros
./run_robot_test.sh  # Avvia tutto
ros2 topic echo /diff_drive_controller/cmd_vel_unstamped  # Verifica comandi
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.5}}" -1  # Test diretto
echo "o 100 100" > /dev/ttyACM0  # Test Arduino diretto
```

---

**ULTIMO UPDATE**: 28 Ottobre 2025 - Sistema pronto per debug finale command flow issue