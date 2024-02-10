# robot-ros
Repository con il codice per la creazione di un robot tramite framework ROS2 (Foxy)

## Install

```console

sudo apt install libraspberrypi-bin v4l-utils ros-${ROS_DISTRO}-v4l2-camera ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-camera-calibration-parsers ros-${ROS_DISTRO}-camera-info-manager ros-${ROS_DISTRO}-launch-testing-ament-cmake
sudo apt install ros-${ROS_DISTRO}-rosbridge-server ros-${ROS_DISTRO}-rosbridge-suite
sudo apt install ros-${ROS_DISTRO}-slam-toolbox

sudo apt install python3-pip -y
pip install RPi.GPIO xacro
sudo apt install rpi.gpio-common ros-${ROS_DISTRO}-gazebo-ros-pkgs ros-${ROS_DISTRO}-rviz2 python3-roslaunch ros-${ROS_DISTRO}-rqt-image-view
cd ~
git clone https://github.com/riolaf05/robot-ros

cd robot-ros
colcon build
source install/setup.bash
```

### Simulazione

1. robot state publisher
2. Gazebo
3. Rviz

```console
ros2 launch robot_ros test_mobile_robot_1.launch.py 
```

4. **Nav2** (serve per settare punti di arrivo e partenza e generare il percorso, necessita di `/map` e `/odom`), `/odom` è generato dal nodo `diff_drive` (simulato in Gazebo) che serve per fare l'odometria, cioè la stima della posizione del robot dati i comandi di movimento che gli sono stati dati (su `/cmd_vel`)

```console
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```
TODO: integrarlo nel launch file

5. **Slam Toolbox** (è uno dei tool per generare lo slam, fornisce `/map`)

```console
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```
TODO: integrarlo nel launch file


### Robot su raspberry

1. robot state publisher
2. controller hardware custom fatto in python (`robot_ros/cmd_to_pwm_driver.py`) 
3. ROS bridge per web server

```console
ros2 launch robot_ros rsp.launch.py
```

Lanciare `webserver/index.html` (aggiornando l'indirizzo IP) per la console web.

## Altri comandi

Lanciare ROS bridge
```console
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Salva le mappe
```console
ros2 run nav2_map_server map_saver_cli -f maps/my_map_1
```

Visualizza il grafo di nodi e topic
```console
rqt_graph
```

Visualizza l'albero delle trasformazioni (vedi `frames.pdf`)
```console
ros2 run tf2_tools view_frames.py
```

Tastiera da console
```console
ros2 run turtlebot3_teleop teleop_keyboard
```

# References 

### Ros Navigation

* [Corso Udemy](https://www.udemy.com/course/ros2-nav2-stack/)

### ROS-bridge

* [ROS web tutorial part 1 - rosbridge server and roslibjs](https://msadowski.github.io/ros-web-tutorial-pt1/)

* [How to visualise ROS images in html?](https://parkerrobert.medium.com/how-to-visualise-ros-images-in-html-c6b88e37e985)

