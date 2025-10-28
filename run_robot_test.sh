#!/bin/bash

# Script completo per test del robot Arduino con teleop
# Test finale del sistema ROS2 control + Arduino

echo "🤖 === TEST COMPLETO ROBOT ARDUINO CON TELEOP ==="
echo ""

# Naviga nella directory corretta
cd /home/rosario/robot-ros

# Source dell'ambiente ROS2
echo "📦 Sourcing ambiente ROS2..."
source install/setup.bash

echo ""
echo "🚀 Avvio sistema ROS2 control + Arduino..."
echo ""
echo "  ✅ Hardware interface Arduino"
echo "  ✅ Joint state broadcaster" 
echo "  ✅ Diff drive controller"
echo ""
echo "Attendere i messaggi di successo:"
echo "  - 'Arduino Hardware Interface activated'"
echo "  - 'Configured and activated joint_state_broadcaster'"
echo "  - 'Configured and activated diff_drive_controller'"
echo ""
echo "DOPO che vedi questi messaggi, apri UN NUOVO TERMINALE e esegui:"
echo ""
echo "  cd /home/rosario/robot-ros"
echo "  source install/setup.bash"
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diff_drive_controller/cmd_vel_unstamped"
echo ""
echo "Controlli teleop:"
echo "  🔺 i = avanti    🔻 k = stop"
echo "  ◀️ j = sinistra  ▶️ l = destra"
echo "  🔄 u,o = curve   ❌ Ctrl+C = exit"
echo ""
echo "================================"
echo ""

# Avvia il sistema
ros2 launch robot_ros arduino_persistent.launch.py