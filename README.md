# robot-ros
Repository con il codice per la creazione di un robot tramite framework ROS

## Lanciare per il robot su Raspberry:

```console
ros2 launch robot_ros launch_robot.launch.py
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=False
```

Per vedere l'albero dei frames:

```console
ros2 run tf2_tools view_frames
```