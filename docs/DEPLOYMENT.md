# Guida Deployment - Robot Mobile ROS2

Questa guida completa ti accompagna passo-passo nel deploy del robot mobile su Raspberry Pi con Ubuntu 22.04 e ROS2 Humble.

## Indice

1. [Prerequisiti Hardware](#prerequisiti-hardware)
2. [Setup Raspberry Pi](#setup-raspberry-pi)
3. [Installazione ROS2 Humble](#installazione-ros2-humble)
4. [Installazione Pacchetti ROS2](#installazione-pacchetti-ros2)
5. [Setup Arduino](#setup-arduino)
6. [Clone e Build del Progetto](#clone-e-build-del-progetto)
7. [Configurazione Hardware](#configurazione-hardware)
8. [Test e Verifica](#test-e-verifica)
9. [Avvio del Robot](#avvio-del-robot)
10. [Troubleshooting](#troubleshooting)

---

## Prerequisiti Hardware

Prima di iniziare, assicurati di avere:

- ✅ Raspberry Pi 4 (minimo 2GB RAM, consigliato 4GB)
- ✅ MicroSD card (minimo 32GB, classe 10 o superiore)
- ✅ Arduino Nano
- ✅ L298N Motor Driver
- ✅ 2x Motori DC con encoder
- ✅ RPLIDAR (A1, A2 o compatibile)
- ✅ Batteria 12V (LiPo/Li-ion)
- ✅ Step-down converter (12V → 5V)
- ✅ Tutti i cavi necessari (vedi [WIRING.md](WIRING.md))

---

## Setup Raspberry Pi

### 1. Installazione Ubuntu Server 22.04

1. Scarica [Raspberry Pi Imager](https://www.raspberrypi.com/software/)
2. Avvia Raspberry Pi Imager
3. Seleziona:
   - **OS:** Ubuntu Server 22.04 LTS (64-bit)
   - **Storage:** La tua microSD card
4. Clicca su ⚙️ (Settings) e configura:
   - **Hostname:** `robot-pi` (o nome preferito)
   - **Username e Password:** Imposta le tue credenziali
   - **WiFi:** Configura la tua rete WiFi (se necessario)
   - **SSH:** Abilita SSH
5. Clicca "Write" e attendi il completamento

### 2. Primo Avvio e Connessione

1. Inserisci la microSD nel Raspberry Pi
2. Collega alimentazione ed ethernet (o attendi connessione WiFi)
3. Attendi 1-2 minuti per il primo boot
4. Trova l'IP del Raspberry:
   ```bash
   # Da un altro computer sulla stessa rete
   nmap -sn 192.168.1.0/24
   # oppure controlla il router
   ```
5. Connettiti via SSH:
   ```bash
   ssh username@<IP_RASPBERRY>
   ```

### 3. Setup OpenSSH Server (se necessario)

```bash
# Aggiorna il sistema
sudo apt update && sudo apt upgrade -y

# Installa OpenSSH Server (dovrebbe essere già installato)
sudo apt install openssh-server -y

# Abilita e avvia SSH
sudo systemctl enable ssh
sudo systemctl start ssh

# (Opzionale) Configura firewall
sudo ufw allow ssh
sudo ufw enable
```

---

## Installazione ROS2 Humble

### 1. Setup Locale

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 2. Setup Sources

```bash
# Abilita repository Universe
sudo apt install software-properties-common
sudo add-apt-repository universe

# Aggiungi repository ROS2
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 3. Installazione ROS2 Base

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-ros-base -y
```

**Nota:** Installiamo `ros-humble-ros-base` (senza GUI) per risparmiare spazio e risorse sul Raspberry Pi.

### 4. Setup Ambiente ROS2

```bash
# Aggiungi al .bashrc per caricamento automatico
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verifica installazione
ros2 --version
```

---

## Installazione Pacchetti ROS2

### 1. Colcon (Build Tool)

```bash
sudo apt install python3-colcon-common-extensions -y
```

### 2. Pacchetti Essenziali Robot

```bash
sudo apt install -y \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup
```

### 3. ROS2 Control e Controller

```bash
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager
```

### 4. DiffDriveArduino Hardware Interface

```bash
cd ~
git clone https://github.com/joshnewans/diffdrive_arduino.git
cd diffdrive_arduino
colcon build
echo "source ~/diffdrive_arduino/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Importante:** Questo pacchetto fornisce l'hardware interface per comunicare con l'Arduino.

### 5. RPLIDAR Driver

```bash
sudo apt install -y ros-humble-rplidar-ros
```

### 6. Camera e Comunicazione

```bash
sudo apt install -y \
  ros-humble-v4l2-camera \
  ros-humble-rosbridge-server \
  ros-humble-rosbridge-suite
```

### 7. Utilità e Strumenti

```bash
sudo apt install -y \
  ros-humble-teleop-twist-keyboard \
  python3-serial \
  git \
  minicom \
  screen
```

---

## Setup Arduino

### 1. Installazione Arduino IDE (su PC di sviluppo)

Se non hai Arduino IDE sul tuo computer di sviluppo:

```bash
# Su Ubuntu/Debian
sudo apt install arduino -y

# Oppure scarica da: https://www.arduino.cc/en/software
```

### 2. Upload Sketch Arduino

1. Apri Arduino IDE sul tuo PC
2. Collega Arduino Nano via USB al PC
3. Seleziona:
   - **Board:** Arduino Nano
   - **Processor:** ATmega328P (Old Bootloader) - prova entrambe le opzioni se una non funziona
   - **Port:** La porta seriale corretta (es. /dev/ttyUSB0 o COM3)
4. Apri il file `arduino/diffdrive_arduino/diffdrive_arduino.ino` dal progetto
5. **IMPORTANTE:** Prima di caricare, modifica il valore `ENCODER_CPR` (linea 30) secondo le specifiche del tuo encoder:
   ```cpp
   const int ENCODER_CPR = 374;  // <-- MODIFICA QUESTO VALORE
   ```
   Calcolo: `CPR base encoder × rapporto riduttore`
   - Esempio: 11 CPR × 34:1 = 374 CPR
6. Clicca "Upload" (Carica)
7. Verifica nel Serial Monitor (115200 baud) che non ci siano errori

### 3. Test Sketch Arduino

Apri il Serial Monitor (57600 baud) e invia:
```
<e>
```
Dovresti ricevere qualcosa come:
```
L:0,R:0
```

Ruota manualmente le ruote e verifica che i conteggi cambino.

### 4. Trasferisci Arduino al Raspberry Pi

Scollega l'Arduino dal PC e collegalo al Raspberry Pi via USB.

---

## Clone e Build del Progetto

### 1. Clone Repository

```bash
cd ~
git clone https://github.com/riolaf05/robot-ros.git
cd robot-ros
```

### 2. Verifica Dipendenze

```bash
rosdep install --from-paths . --ignore-src -r -y
```

### 3. Build del Workspace

```bash
colcon build --symlink-install
```

**Nota:** `--symlink-install` permette di modificare i file Python senza ricompilare.

### 4. Setup Automatico Workspace

```bash
echo "source ~/robot-ros/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Configurazione Hardware

### 1. Identificazione Porte Seriali

```bash
# Lista tutte le porte USB
ls -l /dev/ttyUSB*

# Output esempio:
# /dev/ttyUSB0 -> Arduino
# /dev/ttyUSB1 -> LIDAR
```

Se le porte sono diverse, aggiorna:
- Arduino: `description/ros2_control.xacro` (parametro `device`)
- LIDAR: `launch/launch_robot_cus.launch.py` (parametro `serial_port`)

### 2. Permessi Seriale

```bash
# Aggiungi utente al gruppo dialout
sudo usermod -a -G dialout $USER

# Logout e login per applicare
# oppure riavvia
sudo reboot
```

### 3. Test Comunicazione Arduino

```bash
# Test con minicom
minicom -D /dev/ttyUSB0 -b 57600

# Invia: <e>
# Aspetta risposta: L:0,R:0

# Esci: Ctrl+A poi X
```

### 4. Test LIDAR

```bash
# Avvia solo il LIDAR
ros2 launch robot_ros lidar.launch.py

# In un altro terminale, verifica il topic /scan
ros2 topic echo /scan
```

### 5. Verifica Dimensioni Ruote e Encoder

Misura le dimensioni reali delle tue ruote e verifica nel file `config/my_controllers.yaml`:

```yaml
wheel_separation: 0.35  # Distanza tra ruote (metri)
wheel_radius: 0.05      # Raggio ruota (metri)
```

Aggiorna se necessario.

---

## Test e Verifica

### 1. Test Robot State Publisher

```bash
# Avvia solo robot state publisher e SLAM
ros2 launch robot_ros rsp.launch.py

# Verifica che i nodi siano attivi
ros2 node list
```

Dovresti vedere:
- `/robot_state_publisher`
- `/controller_manager`
- `/slam_toolbox`

### 2. Test Controller Manager

```bash
# Lista controller attivi
ros2 control list_controllers

# Output atteso:
# diff_cont[diff_drive_controller/DiffDriveController] active
# joint_broad[joint_state_broadcaster/JointStateBroadcaster] active
```

### 3. Test Movimento Manuale

```bash
# In un terminale, avvia il robot
ros2 launch robot_ros launch_robot_cus.launch.py

# In un altro terminale, invia comandi
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --once

# Il robot dovrebbe muoversi in avanti
```

**⚠️ Attenzione:** Tieni il robot sollevato per i primi test!

### 4. Test Odometria

```bash
# Verifica topic odometria
ros2 topic echo /odom

# Ruota le ruote manualmente e verifica che i valori cambino
```

### 5. Test SLAM

```bash
# Con robot avviato, muovi il robot nell'ambiente
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# In un altro terminale, salva la mappa
ros2 run nav2_map_server map_saver_cli -f ~/mia_mappa
```

---

## Avvio del Robot

### Avvio Completo

```bash
# Avvia tutto il sistema
ros2 launch robot_ros launch_robot_cus.launch.py
```

Questo avvia:
- ✅ Robot State Publisher
- ✅ Controller Manager e Controller
- ✅ SLAM Toolbox
- ✅ LIDAR
- ✅ Camera
- ✅ Rosbridge (per interfaccia web)

### Controllo da Tastiera

```bash
# In un nuovo terminale
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Usa i tasti:
- `i`: Avanti
- `k`: Stop
- `,`: Indietro
- `j`: Sinistra
- `l`: Destra

### Interfaccia Web (Opzionale)

1. Assicurati che rosbridge sia attivo
2. Apri `webserver/index.html` nel browser
3. Aggiorna l'IP del Raspberry Pi nel file
4. Controlla il robot dal browser

### Salvataggio Mappe

```bash
# Dopo aver mappato l'ambiente
ros2 run nav2_map_server map_saver_cli -f maps/nome_mappa
```

---

## Troubleshooting

### Problema: Controller Manager non si avvia

**Sintomi:** Errore "Failed to load controller"

**Soluzione:**
```bash
# Verifica che diffdrive_arduino sia installato
ros2 pkg list | grep diffdrive_arduino

# Se non c'è, reinstalla
cd ~/diffdrive_arduino
colcon build
source install/setup.bash
```

### Problema: Arduino non comunica

**Sintomi:** "Failed to connect to /dev/ttyUSB0"

**Soluzione:**
```bash
# Verifica porta
ls -l /dev/ttyUSB*

# Verifica permessi
groups  # Deve contenere 'dialout'

# Se non c'è, aggiungi e riavvia
sudo usermod -a -G dialout $USER
sudo reboot
```

### Problema: Encoder non funziona

**Sintomi:** Odometria sempre a 0, robot non si muove correttamente

**Soluzione:**
1. Verifica collegamenti encoder (vedi [WIRING.md](WIRING.md))
2. Testa encoder con Serial Monitor Arduino
3. Verifica valore `ENCODER_CPR` nello sketch Arduino
4. Controlla alimentazione encoder (5V)

### Problema: Motori girano al contrario

**Sintomi:** Robot va indietro quando comando avanti

**Soluzione:**
- **Opzione 1:** Inverti fisicamente i cavi del motore sul L298N
- **Opzione 2:** Modifica lo sketch Arduino invertendo HIGH/LOW

### Problema: LIDAR non funziona

**Sintomi:** "Failed to connect to LIDAR"

**Soluzione:**
```bash
# Verifica porta LIDAR
ls -l /dev/ttyUSB*

# Testa standalone
ros2 launch rplidar_ros rplidar_a1_launch.py

# Verifica parametri in launch_robot_cus.launch.py
```

### Problema: SLAM non crea mappa

**Sintomi:** Mappa vuota o errata

**Soluzione:**
1. Verifica che `/scan` pubblichi dati: `ros2 topic echo /scan`
2. Verifica che `/odom` pubblichi dati: `ros2 topic echo /odom`
3. Controlla frame in SLAM config: `base_footprint`, `odom`, `map`
4. Muovi il robot lentamente per permettere a SLAM di processare

### Problema: Robot si muove in modo irregolare

**Sintomi:** Velocità non corretta, movimenti a scatti

**Soluzione:**
1. Verifica `wheel_radius` e `wheel_separation` in `my_controllers.yaml`
2. Verifica `ENCODER_CPR` in Arduino sketch
3. Controlla batteria (voltage basso causa problemi)
4. Verifica che i motori ricevano PWM corretto

### Problema: Performance Raspberry Pi basse

**Sintomi:** Sistema lento, lag

**Soluzione:**
```bash
# Monitora CPU e RAM
htop

# Disabilita servizi non necessari
sudo systemctl disable bluetooth
sudo systemctl disable avahi-daemon

# Usa modello Haiku per nodi meno critici
# Riduci frequenza pubblicazione in my_controllers.yaml
```

---

## Comando Utili

### Verifica Sistema

```bash
# Lista tutti i nodi attivi
ros2 node list

# Lista tutti i topic
ros2 topic list

# Info su un topic
ros2 topic info /cmd_vel

# Echo di un topic
ros2 topic echo /odom

# Lista controller
ros2 control list_controllers

# Info su un controller
ros2 control list_hardware_interfaces
```

### Debug

```bash
# Log completi
ros2 launch robot_ros launch_robot_cus.launch.py --log-level DEBUG

# TF tree
ros2 run tf2_tools view_frames

# RQT graph
rqt_graph
```

### Manutenzione

```bash
# Ricompila singolo pacchetto
colcon build --packages-select robot_ros

# Pulisci build
rm -rf build install log
colcon build

# Aggiorna tutto
cd ~/robot-ros
git pull
colcon build
```

---

## Autostart al Boot (Opzionale)

Per avviare automaticamente il robot al boot:

```bash
# Crea servizio systemd
sudo nano /etc/systemd/system/robot-ros.service
```

Inserisci:
```ini
[Unit]
Description=ROS2 Robot Service
After=network.target

[Service]
Type=simple
User=YOUR_USERNAME
Environment="ROS_DOMAIN_ID=0"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/YOUR_USERNAME/diffdrive_arduino/install/setup.bash && source /home/YOUR_USERNAME/robot-ros/install/setup.bash && ros2 launch robot_ros launch_robot_cus.launch.py"
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

Sostituisci `YOUR_USERNAME` con il tuo username.

Abilita il servizio:
```bash
sudo systemctl daemon-reload
sudo systemctl enable robot-ros.service
sudo systemctl start robot-ros.service

# Verifica stato
sudo systemctl status robot-ros.service
```

---

## Risorse Aggiuntive

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ros2_control Documentation](https://control.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2 Documentation](https://navigation.ros.org/)
- [Wiring Diagram](WIRING.md)

---

## Supporto

Per problemi o domande:
- Apri un issue su GitHub: https://github.com/riolaf05/robot-ros/issues
- Controlla la documentazione ROS2
- Verifica i log con `ros2 launch --log-level DEBUG`

---

**Congratulazioni! Il tuo robot ROS2 è ora pronto per l'uso! 🤖**
