#!/bin/bash

# Script completo per test del robot Arduino con teleop
# Autore: GitHub Copilot
# Data: 28/10/2024

echo "🤖 Avvio sistema completo robot Arduino con teleop..."

# Verifica che siamo nella directory corretta
cd /home/rosario/robot-ros

# Source dell'ambiente ROS2
echo "📦 Sourcing ambiente ROS2..."
source install/setup.bash

echo "🔧 Avvio sistema ROS2 e Arduino Hardware Interface..."

# Avvia il sistema in background in un nuovo terminale
gnome-terminal --title="Robot System" -- bash -c "
    cd /home/rosario/robot-ros && 
    source install/setup.bash && 
    ros2 launch robot_ros arduino_direct.launch.py; 
    exec bash"

# Attesa per l'avvio del sistema
echo "⏳ Attesa avvio sistema (10 secondi)..."
sleep 10

echo "🎮 Verifica controller attivi..."
ros2 control list_controllers

echo ""
echo "🎯 Avvio Teleop Twist Keyboard!"
echo ""
echo "   Usa i seguenti tasti per controllare il robot:"
echo "   🔺 i = avanti"
echo "   🔻 k = stop"  
echo "   ◀️ j = sinistra"
echo "   ▶️ l = destra"
echo "   🔄 u o = curve"
echo ""
echo "   📈 q/z = aumenta/diminuisci velocità"
echo "   🔄 w/x = aumenta/diminuisci velocità angolare"
echo ""
echo "   ❌ Premi Ctrl+C per uscire"
echo ""

# Avvia teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diff_drive_controller/cmd_vel_unstamped