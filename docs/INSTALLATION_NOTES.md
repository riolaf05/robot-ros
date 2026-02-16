# Note Importanti sull'Installazione

## Serial e DiffDriveArduino - Workspace Unificato

### Problema Comune

Quando si tenta di compilare `diffdrive_arduino` da solo, si ottiene questo errore:

```
CMake Error at CMakeLists.txt:15 (find_package):
  By not providing "Findserial.cmake" in CMAKE_MODULE_PATH this project has
  asked CMake to find a package configuration file provided by "serial", but
  CMake did not find one.
```

### Causa

`diffdrive_arduino` dipende dalla libreria `serial` (comunicazione seriale). CMake non riesce a trovare questa dipendenza se i due pacchetti sono in workspace separati.

### Soluzione Corretta ✅

**Compilare entrambi i pacchetti nello stesso workspace:**

```bash
# Crea workspace unificato
cd ~
mkdir -p robot_ws/src
cd robot_ws/src

# Clona serial (dipendenza)
git clone https://github.com/RoverRobotics-forks/serial-ros2.git serial

# Clona diffdrive_arduino
git clone https://github.com/joshnewans/diffdrive_arduino.git

# Compila tutto insieme
cd ~/robot_ws
colcon build --symlink-install

# Source del workspace
source install/setup.bash
echo "source ~/robot_ws/install/setup.bash" >> ~/.bashrc
```

### Soluzioni Errate ❌

#### Tentativo 1: Installare serial separatamente
```bash
# NON FUNZIONA
cd ~
git clone https://github.com/wjwwood/serial.git  # Versione ROS1!
cd serial
colcon build  # Errore: cerca catkin (ROS1)
```

**Problema:** Questo è il repository ROS1 che usa catkin invece di ament.

#### Tentativo 2: Workspace separati
```bash
# NON FUNZIONA
cd ~
git clone https://github.com/RoverRobotics-forks/serial-ros2.git serial
cd serial
colcon build
source install/setup.bash

cd ~
git clone https://github.com/joshnewans/diffdrive_arduino.git
cd diffdrive_arduino
colcon build  # Errore: CMake non trova serial
```

**Problema:** CMake non cerca dipendenze in altri workspace, anche se hai fatto source.

#### Tentativo 3: Installare da apt
```bash
# NON FUNZIONA
sudo apt install ros-humble-serial-driver
```

**Problema:** `ros-humble-serial-driver` è un pacchetto diverso e non fornisce la libreria `serial` richiesta da diffdrive_arduino.

## Perché Funziona la Soluzione Corretta

Quando `serial` e `diffdrive_arduino` sono nello **stesso workspace**:

1. Colcon analizza tutte le dipendenze in `src/`
2. Compila prima `serial` (nessuna dipendenza)
3. Poi compila `diffdrive_arduino` e trova `serial` già compilato nello stesso workspace
4. CMake può risolvere correttamente il path a `serialConfig.cmake`

## Struttura File Finale

```
~/robot_ws/                    # Workspace unificato per dipendenze
  ├── src/
  │   ├── serial/              # Libreria comunicazione seriale
  │   └── diffdrive_arduino/   # Hardware interface
  ├── build/
  ├── install/
  └── log/

~/robot-ros/                   # Progetto robot principale
  ├── src/
  ├── launch/
  ├── description/
  ├── config/
  ├── build/
  ├── install/
  └── log/
```

## Ordine di Source nel .bashrc

```bash
# Ordine corretto
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash       # Serial + DiffDriveArduino
source ~/robot-ros/install/setup.bash      # Progetto robot
```

## Verifica Installazione

Dopo aver compilato, verifica che entrambi i pacchetti siano disponibili:

```bash
# Verifica serial
ros2 pkg list | grep serial
# Output atteso: serial

# Verifica diffdrive_arduino
ros2 pkg list | grep diffdrive_arduino
# Output atteso: diffdrive_arduino

# Verifica plugin ros2_control
ros2 control list_hardware_interfaces
# Dovrebbe caricare senza errori quando ros2_control_node è in esecuzione
```

## Repository Serial per ROS2

Ci sono vari fork della libreria serial per ROS2. Questi sono i principali:

1. **RoverRobotics-forks/serial-ros2** ✅ (RACCOMANDATO)
   - Fork mantenuto e testato
   - URL: https://github.com/RoverRobotics-forks/serial-ros2.git
   - Branch: main

2. **tylerjw/serial** ⚠️ (Alternativo)
   - Branch ROS2: `ros2`
   - URL: https://github.com/tylerjw/serial.git -b ros2
   - Meno aggiornato ma funziona

3. **wjwwood/serial** ❌ (NON USARE)
   - Versione originale ROS1
   - Usa catkin invece di ament
   - NON compatibile con ROS2

## Troubleshooting

### Errore: "Could not find package 'serial'"

**Causa:** I pacchetti sono in workspace separati

**Soluzione:**
```bash
# Rimuovi installazioni separate
cd ~
rm -rf serial diffdrive_arduino

# Ricrea workspace unificato (vedi sopra)
```

### Errore: "package 'catkin' not found"

**Causa:** Hai clonato la versione ROS1 di serial

**Soluzione:**
```bash
cd ~
rm -rf serial
git clone https://github.com/RoverRobotics-forks/serial-ros2.git serial
```

### Errore: Controller manager non carica plugin

**Causa:** diffdrive_arduino non è nel ROS_PACKAGE_PATH

**Soluzione:**
```bash
# Verifica source
source ~/robot_ws/install/setup.bash

# Verifica pacchetto
ros2 pkg list | grep diffdrive_arduino

# Se non c'è, ricompila
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash
```

## Link Utili

- [DiffDriveArduino GitHub](https://github.com/joshnewans/diffdrive_arduino)
- [Serial-ROS2 GitHub](https://github.com/RoverRobotics-forks/serial-ros2)
- [Articulated Robotics Tutorial](https://articulatedrobotics.xyz/mobile-robot-13-ros2-control-real/)

---

**Data:** 2026-02-15
**Status:** Testato e funzionante su ROS2 Humble
