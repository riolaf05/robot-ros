# Quick Start Guide - Robot ROS2

Guida rapida per far partire il robot in meno di 30 minuti (assumendo hardware già assemblato).

## Prerequisiti

- ✅ Hardware assemblato (vedi [docs/WIRING.md](docs/WIRING.md))
- ✅ Raspberry Pi 4 con Ubuntu 22.04
- ✅ Arduino Nano programmato
- ✅ Batteria carica

## 1. Setup Raspberry Pi (10 minuti)

```bash
# Connetti via SSH
ssh user@raspberry_pi_ip

# Aggiorna sistema
sudo apt update && sudo apt upgrade -y

# Installa ROS2 Humble (rapido)
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-ros-base -y

# Source ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 2. Installa Pacchetti (5 minuti)

```bash
# Colcon
sudo apt install python3-colcon-common-extensions -y

# Pacchetti essenziali (tutto insieme)
sudo apt install -y \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-slam-toolbox \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-rplidar-ros \
  ros-humble-v4l2-camera \
  ros-humble-rosbridge-server \
  ros-humble-teleop-twist-keyboard \
  python3-serial \
  git
```

## 3. Installa DiffDriveArduino (5 minuti)

```bash
cd ~
git clone https://github.com/joshnewans/diffdrive_arduino.git
cd diffdrive_arduino
colcon build
echo "source ~/diffdrive_arduino/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 4. Clone e Build Progetto (5 minuti)

```bash
cd ~
git clone https://github.com/riolaf05/robot-ros.git
cd robot-ros
colcon build --symlink-install
echo "source ~/robot-ros/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 5. Configura Permessi (1 minuto)

```bash
# Permessi seriale
sudo usermod -a -G dialout $USER

# Logout e login (o riavvia)
sudo reboot
```

## 6. Verifica Hardware (2 minuti)

```bash
# Dopo riavvio, riconnetti via SSH

# Verifica porte seriali
ls -l /dev/ttyUSB*
# Dovrebbe mostrare:
# /dev/ttyUSB0 -> Arduino
# /dev/ttyUSB1 -> LIDAR

# Se diverse, aggiorna config:
# Arduino: description/ros2_control.xacro
# LIDAR: launch/launch_robot_cus.launch.py
```

## 7. Test Arduino (1 minuto)

```bash
# Test comunicazione
sudo apt install minicom -y
minicom -D /dev/ttyUSB0 -b 57600

# Digita: <e>
# Aspetta: L:0,R:0

# Ruota le ruote manualmente
# I numeri dovrebbero cambiare

# Esci: Ctrl+A poi X
```

## 8. Avvio Robot (1 minuto)

```bash
# Avvia sistema completo
ros2 launch robot_ros launch_robot_cus.launch.py
```

Dovresti vedere:
```
[INFO] [robot_state_publisher]: ...
[INFO] [controller_manager]: ...
[INFO] [slam_toolbox]: ...
[INFO] [rplidar_composition]: ...
```

## 9. Test Movimento (1 minuto)

```bash
# In un NUOVO terminale SSH
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}" --once

# Il robot dovrebbe muoversi in avanti!
```

## 10. Controllo da Tastiera (FINALE!)

```bash
# In un nuovo terminale
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Usa i tasti:
# i = Avanti
# k = Stop
# , = Indietro
# j = Sinistra
# l = Destra
```

## Comandi Utili

### Verifica Sistema

```bash
# Nodi attivi
ros2 node list

# Topic attivi
ros2 topic list

# Controller attivi
ros2 control list_controllers
```

### Debug

```bash
# Visualizza /odom (odometria)
ros2 topic echo /odom

# Visualizza /scan (LIDAR)
ros2 topic echo /scan

# Log controller_manager
ros2 launch robot_ros launch_robot_cus.launch.py --log-level DEBUG
```

### Salva Mappa

```bash
# Dopo aver mappato l'ambiente
ros2 run nav2_map_server map_saver_cli -f ~/mia_mappa
```

## Troubleshooting Rapido

### Problema: Controller non si avvia

```bash
# Reinstalla diffdrive_arduino
cd ~/diffdrive_arduino
rm -rf build install log
colcon build
source install/setup.bash
```

### Problema: Arduino non risponde

```bash
# Verifica porta
ls -l /dev/ttyUSB*

# Verifica permessi
groups  # Deve contenere 'dialout'
```

### Problema: Encoder non funziona

1. Verifica collegamenti encoder (docs/WIRING.md)
2. Controlla valore CPR in Arduino sketch (deve essere 374 o il tuo valore)
3. Testa con Serial Monitor

### Problema: LIDAR non funziona

```bash
# Test standalone
ros2 launch rplidar_ros rplidar_a1_launch.py

# Se funziona, aggiorna porta in launch file
```

## Prossimi Passi

Ora che il robot funziona:

1. **Crea una mappa:**
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   # Muovi il robot nell'ambiente
   ros2 run nav2_map_server map_saver_cli -f maps/casa
   ```

2. **Esplora Nav2 per navigazione autonoma**

3. **Configura interfaccia web**
   - Apri `webserver/index.html`
   - Aggiorna IP Raspberry Pi
   - Controlla da browser

4. **Leggi documentazione completa:**
   - [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md) - Setup dettagliato
   - [docs/WIRING.md](docs/WIRING.md) - Schema hardware
   - [docs/SUMMARY.md](docs/SUMMARY.md) - Modifiche progetto

## Note Importanti

- ⚠️ Testa sempre con robot sollevato prima di metterlo per terra
- 🔋 Monitora livello batteria
- 🛑 I motori si fermano automaticamente dopo 1s senza comandi (timeout sicurezza)
- 📏 Verifica che `wheel_radius` e `wheel_separation` in `config/my_controllers.yaml` corrispondano alle tue ruote

## Supporto

Problemi? Controlla:
1. [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md) - Sezione Troubleshooting
2. [arduino/README.md](arduino/README.md) - Problemi Arduino
3. GitHub Issues: https://github.com/riolaf05/robot-ros/issues

---

**Congratulazioni! Il tuo robot ROS2 è operativo! 🤖🎉**

Tempo totale stimato: **~30 minuti**
