# robot-ros
Repository con il codice per la creazione di un robot differenziale tramite framework ROS2 (Humble) con Arduino e ROSArduinoBridge.

## 🎯 Panoramica del Progetto

Questo progetto implementa un **robot differenziale completo** controllato tramite ROS2, utilizzando:

- **ROS2 Humble**: Framework principale per il controllo robotico
- **ros2_control**: Sistema di controllo hardware/software
- **Arduino**: Microcontrollore per il controllo motori 
- **ROSArduinoBridge**: Firmware Arduino per comunicazione seriale
- **Navigation2**: Stack di navigazione autonoma
- **Hardware Interface**: Interfaccia custom per Arduino

### 🚀 Funzionalità Implementate

✅ **Controllo Differenziale**: Movimento omnidirezionale con due ruote motrici  
✅ **Hardware Interface**: Comunicazione seriale robusta con Arduino  
✅ **Teleoperazione**: Controllo remoto via tastiera (`teleop_twist_keyboard`)  
✅ **Simulazione**: Test completi in Gazebo e RViz  
✅ **Navigazione**: Integration con Nav2 stack per navigazione autonoma  
✅ **Mapping**: Generazione mappe tramite SLAM  
✅ **Web Interface**: Controllo via browser (ROSBridge)  

## 🔧 Setup Hardware Arduino

### Prerequisiti Hardware

- **Arduino Uno/Mega**: Microcontrollore principale
- **Motor Driver**: L298N o equivalente per controllo motori DC
- **Encoder** (opzionali): Per feedback di posizione
- **Sensori**: IMU, LiDAR, camera (opzionali)
- **Alimentazione**: Batteria per motori e Arduino

### 1. Installazione ROSArduinoBridge

Il firmware Arduino è basato sul progetto **ROSArduinoBridge** ottimizzato per ROS2:

```bash
# Clone del repository firmware
git clone https://github.com/riolaf05/ros_arduino_bridge.git
cd ros_arduino_bridge
```

### 2. Configurazione Arduino IDE

1. **Installare Arduino IDE** (versione 1.8.x o 2.x)
2. **Installare librerie necessarie**:
   ```
   - PID Library (Brett Beauregard)
   - Encoder Library (Paul Stoffregen) 
   ```
3. **Aprire il firmware**: `ros_arduino_bridge/ros_arduino_firmware/ros_arduino_firmware.ino`

### 3. Configurazione Hardware nel Firmware

Modifica il file `ROSArduinoBridge.h` per il tuo setup:

```cpp
// Configurazione motori
#define USE_BASE      // Abilita controllo base mobile
#define BAUDRATE     57600

// Pin motori (L298N)
#define RIGHT_MOTOR_BACKWARD 5
#define LEFT_MOTOR_BACKWARD  6  
#define RIGHT_MOTOR_FORWARD  9
#define LEFT_MOTOR_FORWARD   10

// Pin encoder (opzionali)
#define LEFT_ENC_PIN_A  2
#define LEFT_ENC_PIN_B  3
#define RIGHT_ENC_PIN_A 18
#define RIGHT_ENC_PIN_B 19

// Parametri robot
#define WHEEL_DIAMETER      0.065  // metri
#define WHEEL_TRACK         0.17   // metri 
#define ENCODER_CPR         360    // Count per rivoluzione
```

### 4. Upload Firmware

1. **Collegare Arduino** via USB al computer
2. **Selezionare porta** in Arduino IDE (Tools → Port → `/dev/ttyACM0`)
3. **Upload firmware** → Compile and Upload
4. **Verificare connessione**:
   ```bash
   # Test comunicazione seriale
   echo "b" > /dev/ttyACM0  # Dovrebbe rispondere con baudrate
   ```

### 5. Protocollo di Comunicazione

Il firmware utilizza il **protocollo ROSArduinoBridge** con questi comandi principali:

```bash
# Controllo motori PWM
echo "o 100 150" > /dev/ttyACM0  # LEFT=100, RIGHT=150 PWM

# Reset encoder  
echo "r" > /dev/ttyACM0

# Richiesta baudrate
echo "b" > /dev/ttyACM0

# Stop motori
echo "o 0 0" > /dev/ttyACM0
```

## 💻 Installazione Software ROS2

### Wiring Schema

```
Arduino Uno + L298N Motor Driver
================================

Arduino  →  L298N    →  Motore
------------------------------
Pin 5    →  IN1      →  Motor B (Right)
Pin 6    →  IN2      →  Motor B (Right) 
Pin 9    →  IN3      →  Motor A (Left)
Pin 10   →  IN4      →  Motor A (Left)
5V       →  VCC      →  Logic Power
GND      →  GND      →  Ground
         →  12V      →  Motor Power (batteria)

Encoder (opzionali):
Pin 2    →  Left Encoder A
Pin 3    →  Left Encoder B  
Pin 18   →  Right Encoder A
Pin 19   →  Right Encoder B
```

## ROS2 Humble Setup su Raspberry Pi

### Prerequisiti: Ubuntu Server 22.04

Prima di tutto è necessario installare Ubuntu Server 22.04 sul Raspberry Pi 4. Utilizziamo Ubuntu Server perché è il sistema operativo più adatto per ROS2 su Raspberry Pi e ROS2 Humble è compatibile specificamente con Ubuntu 22.04.

Usare [Rasbperry PI Imager](https://www.raspberrypi.com/software/) per installare Ubuntu 22.04.

### Setup OpenSSH Server

Dopo l'installazione di Ubuntu Server, è essenziale configurare SSH per l'accesso remoto al Raspberry Pi:

#### 1. Installazione OpenSSH Server

```console
sudo apt update
sudo apt install openssh-server
```

#### 2. Abilitazione e avvio del servizio SSH

```console
# Abilita SSH all'avvio del sistema
sudo systemctl enable ssh

# Avvia il servizio SSH
sudo systemctl start ssh

# Verifica lo stato del servizio
sudo systemctl status ssh
```

#### 3. Configurazione del firewall (opzionale)

Se hai il firewall abilitato, permettere le connessioni SSH:

```console
sudo ufw allow ssh
sudo ufw enable
```

#### 4. Configurazione SSH (opzionale)

Per maggiore sicurezza, puoi modificare il file di configurazione SSH:

```console
sudo nano /etc/ssh/sshd_config
```

Configurazioni consigliate:
- Cambiare la porta predefinita (22) se necessario
- Disabilitare l'accesso root: `PermitRootLogin no`
- Abilitare l'autenticazione con chiavi pubbliche: `PubkeyAuthentication yes`

Dopo le modifiche, riavviare il servizio:
```console
sudo systemctl restart ssh
```

#### 5. Connessione SSH

Dal computer di sviluppo:
```console
ssh username@raspberry_pi_ip_address
```

### Installazione ROS2 Humble

#### 1. Setup locale

```console
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

#### 2. Setup sources

Abilitare il repository Ubuntu Universe:
```console
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Aggiungere le nuove sources:
```console
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

#### 3. Installazione pacchetti core ROS2

```console
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-ros-base
```

**Nota**: Installiamo `ros-humble-ros-base` invece di `ros-humble-desktop` perché non include strumenti GUI, mantenendo l'installazione leggera per il Raspberry Pi.

#### 4. Installazione colcon (build tool)

```console
sudo apt install python3-colcon-common-extensions
```

#### 5. Installazione dipendenze robot

Pacchetti necessari per il funzionamento del robot:
```console
sudo apt install ros-humble-xacro ros-humble-robot-state-publisher ros-humble-slam-toolbox ros-humble-ros2-control ros-humble-ros2-controllers
```

#### 6. Setup dell'ambiente

Per configurare automaticamente l'ambiente ROS2 ad ogni sessione:
```console
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Note sull'uso di ROS2 su Raspberry Pi

- Il Raspberry Pi 4 con almeno 2GB di RAM è raccomandato per un'esperienza ottimale
- Per strumenti di visualizzazione come Rviz o Gazebo, considera un setup multi-macchina con il Pi che controlla l'hardware e un computer più potente per la visualizzazione
- Per installare pacchetti aggiuntivi ROS2, usa il formato: `sudo apt install ros-humble-<nome-pacchetto>`

### Setup del Progetto Robot-ROS

#### 1. Installazione Git

```console
sudo apt install git
```

#### 2. Clone del Repository

```console
cd ~
git clone https://github.com/riolaf05/robot-ros
```

#### 3. Compilazione del Workspace

```console
cd ~/robot-ros
colcon build
```

#### 4. Setup Automatico del Workspace

Per configurare automaticamente il workspace ad ogni sessione:
```console
echo "source ~/robot-ros/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Nota**: Ora ad ogni nuovo terminale, il workspace robot-ros sarà automaticamente disponibile e potrai utilizzare direttamente i comandi `ros2 launch robot_ros ...`

### Risoluzione Problemi Comuni

#### Errore: "No module named 'xacro'"
Se ricevi questo errore quando lanci i file launch:
```console
sudo apt install ros-humble-xacro ros-humble-robot-state-publisher
```

#### Errore: "Package 'robot_ros' not found"
Assicurati di aver compilato e fatto il source del workspace:
```console
cd ~/robot-ros
colcon build
source install/setup.bash
```

#### Installazione completa dipendenze
Per installare tutti i pacchetti necessari in una volta:
```console
sudo apt install -y ros-humble-xacro ros-humble-robot-state-publisher \
ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup \
ros-humble-teleop-twist-keyboard ros-humble-serial python3-serial \
ros-humble-ros2-control ros-humble-ros2-controllers
```

#### Verifica installazione
Controlla che tutti i pacchetti siano installati:
```console
ros2 pkg list | grep -E "(robot_ros|slam_toolbox|teleop|nav2)"
```

## Descrizione del Progetto

### Architettura del Sistema

Questo progetto implementa un robot mobile differenziale utilizzando ROS2 Humble. Il sistema è progettato per funzionare sia in simulazione (Gazebo) che su hardware reale (Raspberry Pi), con capacità di navigazione autonoma e controllo remoto tramite interfaccia web.

### Struttura del Pacchetto (setup.py)

Il file `setup.py` definisce l'installazione del pacchetto ROS2 e include:

#### Data Files
- **launch/**: File di launch per avvio di nodi e configurazioni
- **description/**: File URDF/Xacro per la descrizione del robot
- **worlds/**: Mondi Gazebo per la simulazione
- **rviz/**: Configurazioni per la visualizzazione in Rviz
- **maps/**: Mappe per la navigazione
- **config/**: File di configurazione per Nav2 e controller
- **bt/**: Behavior trees per la navigazione

### Definizione del Robot (URDF/Xacro)

Il robot è definito utilizzando una struttura modulare di file Xacro:

#### File Principali
- **`robot.urdf.xacro`**: File principale che include tutti i componenti
- **`robot_core.xacro`**: Definisce la struttura base del robot (chassis, ruote)
- **`lidar.xacro`**: Definisce il sensore LiDAR
- **`camera.xacro`**: Definisce la camera
- **`gazebo_control.xacro`**: Plugin per il controllo in simulazione Gazebo
- **`ros2_control.xacro`**: Interfaccia hardware per il robot reale

#### Caratteristiche del Robot
- **Tipo**: Robot mobile differenziale
- **Ruote**: Due ruote motrici posteriori (raggio: 0.14m, larghezza: 0.06m)
- **Sensori**: LiDAR per navigazione, camera per visione
- **Frame di riferimento**: `base_footprint` → `base_link` → componenti specifici

### Robot State Publisher

Il `robot_state_publisher` è un nodo fondamentale che:
- **Legge la descrizione URDF del robot** dal parametro `robot_description`
- **Pubblica le trasformazioni statiche** tra i vari frame del robot
- **Mantiene l'albero delle trasformazioni TF2** aggiornato
- **Permette la visualizzazione corretta** in Rviz e altri strumenti

Viene lanciato attraverso il file `rsp.launch.py` che:
1. Processa il file Xacro principale (`robot.urdf.xacro`)
2. Converte il risultato in URDF
3. Passa l'URDF come parametro al robot_state_publisher
4. Configura l'uso del tempo di simulazione quando necessario

### Launch Files

#### `rsp.launch.py`
- Lancia il robot_state_publisher
- Processa i file Xacro e genera l'URDF
- **Include SLAM Toolbox** per mappatura automatica
- **Include ROS2 Control** con controller manager e differential drive controller
- Utilizzato come base per altri launch file

#### `launch_robot_cus.launch.py` (Robot Reale con Nodi Custom)
- **Robot State Publisher**: Per le trasformazioni del robot
- **Hardware Interface**: Nodo personalizzato per controllo motori (`cmdVel_to_pwm_node`)
- **Camera Node**: Interfaccia con camera USB via v4l2
- **Rosbridge**: Server WebSocket per interfaccia web remota
- **Nodi Accessori**: Include tutti i nodi custom sviluppati e accessori

#### `test_mobile_robot_1.launch.py` (Simulazione)
- Launch file per simulazione completa in Gazebo
- Include spawning del robot nel mondo virtuale
- Configurazione per uso con Nav2 e SLAM

#### Altri Launch Files
- **`launch_sim.launch.py`**: Launch file alternativo per simulazione
- **`surveillance_bot.launch.py`**: Configurazione per modalità sorveglianza
- **`lidar.launch.py`**: Launch dedicato per il sensore LiDAR

## Comandi di Lancio

### Prerequisiti
Prima di lanciare qualsiasi comando, assicurati di:
1. Avere compilato il workspace: `colcon build`
2. Aver fatto il source del workspace: `source install/setup.bash`

### Robot Reale (Raspberry Pi)

#### Lancio Completo del Robot con Nodi Custom
```console
ros2 launch robot_ros launch_robot_cus.launch.py
```
Questo comando lancia:
- Robot State Publisher + SLAM Toolbox
- Hardware interface per controllo motori
- Nodo camera (v4l2)
- Rosbridge server per interfaccia web
- Tutti i nodi custom e accessori

#### Robot State Publisher + SLAM + ROS2 Control
```console
ros2 launch robot_ros rsp.launch.py
```
Lancia robot_state_publisher + SLAM Toolbox + ROS2 Control per trasformazioni, mappatura automatica e controllo hardware.

#### Con Parametri Personalizzati
```console
ros2 launch robot_ros launch_robot_cus.launch.py use_sim_time:=false
```

### Simulazione (Gazebo)

#### Simulazione Base
```console
ros2 launch robot_ros test_mobile_robot_1.launch.py
```
Lancia la simulazione completa con Gazebo, Rviz e robot_state_publisher.

#### Simulazione Alternativa
```console
ros2 launch robot_ros launch_sim.launch.py
```

#### Solo LiDAR
```console
ros2 launch robot_ros lidar.launch.py
```

### Modalità Sorveglianza
```console
ros2 launch robot_ros surveillance_bot.launch.py
```

### Controllo Remoto

#### Interfaccia Web
1. Lancia il robot con rosbridge: `ros2 launch robot_ros launch_robot_cus.launch.py`
2. Apri `webserver/index.html` nel browser (aggiorna l'IP del Raspberry Pi)

#### Controllo da Tastiera
```console
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Nodi Individuali

#### Controllo Motori
```console
ros2 run robot_ros cmdVel_to_pwm_node
```

#### Publisher Odometria (deprecato)
```console
ros2 run robot_ros odom_publisher_node
```

#### Nodo di Test
```console
ros2 run robot_ros talker
```

#### Nodi Custom
- `cmdVel_to_pwm_node`: Nodo custom che converte i comandi di velocità in segnali PWM per i motori, collegati tramite L298N. Non prevede l'utilizzo di motor encoder quindi **è deprecato**. 
- `motor_controller_custom_node`: Controller personalizzato per i motori. Tentativo di pubblicare manualmente odom e altrei frame, quindi **è deprecato**.
- `odom_publisher_node`: Pubblica dati di odometria. Tentativo di pubblicare manualmente odom e altrei frame, quindi **è deprecato**.
- `nav_behaviour_tree`: Implementazione di behavior tree per navigazione
- `talker`: Nodo di esempio per pubblicazione messaggi

## Installazione pacchetti aggiuntivi

### Setup pacchetti per la navigazione

Per navigazione completa:
```console
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox
```

Per controllo da tastiera:
```console
sudo apt install -y ros-humble-teleop-twist-keyboard
```

Per simulazione con Turtlebot:
```console
sudo apt install -y ros-humble-turtlebot3*
```

Pacchetti per comunicazione seriale (Arduino):
```console
sudo apt install -y ros-humble-serial python3-serial
```

### Dipendenze per Simulazione

Per la simulazione con Gazebo sono necessari i seguenti pacchetti:
```console
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
```

La simulazione include:
1. Robot State Publisher
2. Gazebo (mondo virtuale e fisica)
3. Rviz (visualizzazione)
4. Plugin di controllo differenziale


## Navigazione e SLAM

### Nav2 (Navigazione Autonoma)
Nav2 richiede `/map` e `/odom`. L'odometria (`/odom`) è generata dal plugin `diff_drive` in Gazebo per la simulazione, o da sensori/encoder reali sul robot fisico.

```console
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True map:=maps/my_map_1
```

### SLAM (Mapping)
SLAM Toolbox genera la mappa (`/map`) dell'ambiente:

```console
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```

### Salvataggio Mappe
```console
ros2 launch robot_ros launch_robot_cus.launch.py
```

## Debug e Monitoring

### Visualizzazione Sistema
```console
# Grafo di nodi e topic
rqt_graph

# Albero delle trasformazioni TF2
ros2 run tf2_tools view_frames.py

# Monitor dei topic
ros2 topic list
ros2 topic echo /cmd_vel

# Informazioni sui nodi
ros2 node list
ros2 node info /robot_state_publisher
```

### ROS Bridge (Standalone)
Se non lanciato tramite launch file:
```console
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### Salva le mappe
```console
ros2 run nav2_map_server map_saver_cli -f maps/my_map_1
```

### Pubblicazione manuale comandi velocità
```console
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'
```

## 🚀 Quick Start - Avvio Rapido

### 1. Compilazione del Workspace

```bash
cd /path/to/robot-ros
colcon build --packages-select robot_ros
source install/setup.bash
```

### 2. Test Sistema Completo

**Metodo 1: Script Automatico (CONSIGLIATO)**
```bash
# Avvio sistema completo con teleop
./run_robot_test.sh

# In un nuovo terminale, avvia il teleop
cd /path/to/robot-ros
source install/setup.bash  
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

**Metodo 2: Avvio Manuale**
```bash
# Terminale 1: Hardware Interface + Controllers
ros2 launch robot_ros arduino_persistent.launch.py

# Terminale 2: Teleoperazione  
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

### 3. Controlli Teleop

Dopo l'avvio del teleop keyboard:

```
Controlli di movimento:
   i    - avanti
   k    - stop  
   j    - sinistra
   l    - destra
   u,o  - curve avanti sinistra/destra
   m,.  - curve indietro sinistra/destra

Velocità:
   q/z  - aumenta/diminuisci velocità lineare  
   w/x  - aumenta/diminuisci velocità angolare

CTRL+C - esci
```

### 4. Test e Diagnostica

**Verifica connessione Arduino:**
```bash
# Test comunicazione seriale diretta
echo "o 100 100" > /dev/ttyACM0  # Motori a PWM 100
echo "o 0 0" > /dev/ttyACM0      # Stop motori

# Verifica porta seriale
ls -la /dev/ttyACM*
sudo chmod 666 /dev/ttyACM0  # Se necessario
```

**Monitoraggio ROS2:**
```bash
# Lista topic attivi
ros2 topic list | grep cmd_vel

# Monitoraggio comandi velocità
ros2 topic echo /diff_drive_controller/cmd_vel_unstamped

# Stato controller
ros2 control list_controllers

# Test comando manuale
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" -r 1
```

**Log debugging:**
```bash
# Verifica log hardware interface
ros2 launch robot_ros arduino_persistent.launch.py --ros-args --log-level DEBUG

# Cerca messaggi debug specifici
# - "RECEIVED VELOCITIES: Left=X rad/s, Right=Y rad/s"
# - "Motor PWM command sent: o X Y" 
# - "Arduino Hardware Interface activated"
```

## 🔧 Configurazione Avanzata

### Parametri Hardware Interface

File: `config/arduino_controllers_fixed.yaml`

```yaml
controller_manager:
  ros__parameters:
    update_rate: 10  # Hz - frequenza aggiornamento

robot_ros:
  hardware_interface:
    - name: "RealRobot"  
      type: "arduino_hardware_interface/ArduinoHardwareInterface"
      parameters:
        device: "/dev/ttyACM0"  # Porta Arduino
        baud_rate: 57600        # Baudrate comunicazione
        timeout: 1000          # Timeout ms
        
diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["base_left_wheel_joint"]
    right_wheel_names: ["base_right_wheel_joint"] 
    wheel_separation: 0.17     # Distanza ruote (m)
    wheel_radius: 0.0325       # Raggio ruote (m)
    
    # Limiti velocità
    linear.x.has_velocity_limits: true
    linear.x.max_velocity: 1.0        # m/s
    angular.z.has_velocity_limits: true  
    angular.z.max_velocity: 1.0       # rad/s
```

### Troubleshooting Comuni

**Problema: Arduino non risponde**
```bash
# Verifica connessione
sudo dmesg | grep tty
lsusb | grep Arduino

# Reset permessi
sudo usermod -a -G dialout $USER
newgrp dialout
```

**Problema: Comandi con ritardo**  
✅ **RISOLTO**: Implementato `fsync()` per flush immediato buffer seriale

**Problema: Robot non si muove con teleop**
```bash
# 1. Verifica topic pubblicazione
ros2 topic echo /diff_drive_controller/cmd_vel_unstamped

# 2. Verifica controller attivo  
ros2 control list_controllers

# 3. Test comando diretto Arduino
echo "o 100 100" > /dev/ttyACM0

# 4. Verifica log hardware interface
# Cerca: "RECEIVED VELOCITIES" nei log
```

**Problema: Valori PWM sempre a 0**
- Verifica parametri `max_velocity` nella conversione `velocity_to_pwm()`
- Default: `max_velocity = 5.0 rad/s` 
- PWM = `(velocity / max_velocity) * 255`

### Compilazione e Build
```console
# Compilazione completa
colcon build

# Compilazione singolo pacchetto  
colcon build --packages-select robot_ros

# Source del workspace
source install/setup.bash

# Compilazione con debug verbose
colcon build --packages-select robot_ros --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

## Gestione Versioni e Release

### GitHub Actions Workflow

Il progetto include un workflow automatico (`.github/workflows/release.yml`) che:

#### Trigger
- Si attiva automaticamente quando viene fatto un push su `master` o `main`

#### Funzionalità
1. **Creazione Tag Automatica**: Genera un tag con formato `vYYYY.MM.DD`
2. **Gestione Conflitti**: Se esiste già un tag per la data corrente, aggiunge un suffisso incrementale (es. `v2025.10.27.1`)
3. **Changelog Automatico**: Genera un changelog con tutti i commit dalla release precedente
4. **Release GitHub**: Crea una release completa con descrizione e informazioni
5. **Aggiornamento Versione**: Aggiorna automaticamente il file `package.xml` con la nuova versione

#### Esempio di Tag Generati
- `v2025.10.27` - Prima release del giorno
- `v2025.10.27.1` - Seconda release dello stesso giorno
- `v2025.10.28` - Prima release del giorno successivo

#### Informazioni nella Release
- Data e ora di creazione
- Commit hash
- Lista delle modifiche dalla release precedente
- Artifact con informazioni di release

#### Permessi Richiesti
Il workflow richiede i permessi `contents: write` per creare tag e release.

### Release Manuali
Per creare release manuali:
```console
# Crea un tag locale
git tag -a v1.0.0 -m "Release v1.0.0"

# Push del tag
git push origin v1.0.0
```

# References 

## Ros Navigation

* [Corso Udemy](https://www.udemy.com/course/ros2-nav2-stack/)

## ROS-bridge

* [ROS web tutorial part 1 - rosbridge server and roslibjs](https://msadowski.github.io/ros-web-tutorial-pt1/)

* [How to visualise ROS images in html?](https://parkerrobert.medium.com/how-to-visualise-ros-images-in-html-c6b88e37e985)

## Batteria Raspberry

* [PiSugar](https://www.amazon.it/dp/B09QRPNDHB?psc=1&ref=ppx_yo2ov_dt_b_product_details)
