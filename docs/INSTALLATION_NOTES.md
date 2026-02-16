# Note Importanti sull'Installazione

## Dipendenze di Sistema - libserial-dev

### Requisito Critico

**Prima di compilare il workspace**, devi installare la libreria di sistema `libserial-dev`:

```bash
sudo apt install -y libserial-dev python3-serial
```

**Errore se manca:**
```
fatal error: serial/serial.h: No such file or directory
```

`libserial-dev` fornisce le librerie C++ per la comunicazione seriale richieste dal package `serial-ros2`.

## Serial, DiffDriveArduino e Robot Package - Workspace Unificato

### Problema Comune

Quando si tenta di compilare `diffdrive_arduino` o `robot_ros` separatamente, si ottiene questo errore:

```
CMake Error at CMakeLists.txt:15 (find_package):
  By not providing "Findserial.cmake" in CMAKE_MODULE_PATH this project has
  asked CMake to find a package configuration file provided by "serial", but
  CMake did not find one.
```

### Causa

- `diffdrive_arduino` dipende dalla libreria `serial` (comunicazione seriale)
- `robot_ros` dipende da `diffdrive_arduino`
- CMake non riesce a trovare queste dipendenze se i pacchetti sono in workspace separati

### Soluzione Corretta ✅

**Compilare TUTTI i pacchetti nello stesso workspace:**

```bash
# Installa dipendenza di sistema (PRIMA del build!)
sudo apt install -y libserial-dev python3-serial

# Crea workspace unificato
cd ~
mkdir -p robot_ws/src
cd robot_ws/src

# 1. Clona serial (dipendenza base)
git clone https://github.com/RoverRobotics-forks/serial-ros2.git serial

# 2. Clona diffdrive_arduino (dipende da serial)
git clone https://github.com/joshnewans/diffdrive_arduino.git

# 3. Clona robot_ros (dipende da diffdrive_arduino)
git clone https://github.com/riolaf05/robot-ros.git

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
~/robot_ws/                    # Workspace unificato per TUTTO
  ├── src/
  │   ├── serial/              # 1. Libreria comunicazione seriale
  │   ├── diffdrive_arduino/   # 2. Hardware interface (dipende da serial)
  │   └── robot-ros/           # 3. Progetto robot (dipende da diffdrive_arduino)
  │       ├── launch/
  │       ├── description/
  │       ├── config/
  │       └── robot_ros/       # Python package
  ├── build/
  ├── install/
  └── log/
```

## Ordine di Source nel .bashrc

```bash
# Source ROS2 base
source /opt/ros/humble/setup.bash

# Source workspace unificato (contiene tutto)
source ~/robot_ws/install/setup.bash
```

**Nota:** Non servono più workspace separati! Tutto è in `~/robot_ws`.

## Verifica Installazione

Dopo aver compilato, verifica che tutti i pacchetti siano disponibili:

```bash
# Verifica tutti i package
ros2 pkg list | grep -E "(serial|diffdrive|robot_ros)"

# Output atteso:
# diffdrive_arduino
# robot_ros
# serial

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

### Errore: "serial/serial.h: No such file or directory"

**Causa:** Libreria di sistema `libserial-dev` non installata

**Soluzione:**
```bash
sudo apt install -y libserial-dev python3-serial
cd ~/robot_ws
rm -rf build install log
colcon build --symlink-install
```

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
