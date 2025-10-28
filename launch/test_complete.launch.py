#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import Node, TimerAction, DeclareLaunchArgument, GroupAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node as RosNode
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get the path to the package
    package_name = 'robot_ros'
    
    # Get paths to the robot description and configuration files
    urdf_file = os.path.join(get_package_share_directory(package_name), 'description', 'robot.urdf.xacro')
    controller_config = os.path.join(get_package_share_directory(package_name), 'config', 'arduino_controllers_fixed.yaml')
    
    # Read the URDF file
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    # Robot State Publisher
    robot_state_publisher = RosNode(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False
        }],
        output='screen'
    )

    # Controller Manager Node (ros2_control node)
    controller_manager = RosNode(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            controller_config
        ],
        output='screen'
    )

    # Spawn Joint State Broadcaster
    joint_state_broadcaster_spawner = RosNode(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Spawn Diff Drive Controller
    diff_drive_controller_spawner = RosNode(
        package='controller_manager',
        executable='spawner', 
        arguments=['diff_drive_controller'],
        output='screen'
    )

    return LaunchDescription([
        # Start core nodes
        robot_state_publisher,
        controller_manager,
        
        # Delay spawning controllers to give hardware time to initialize
        TimerAction(
            period=3.0,
            actions=[joint_state_broadcaster_spawner]
        ),
        
        TimerAction(
            period=4.0,
            actions=[diff_drive_controller_spawner]
        )
    ])