#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get the path to the package
    package_name = 'robot_ros'
    
    # Get paths to the robot description and configuration files
    urdf_file = os.path.join(get_package_share_directory(package_name), 'description', 'robot.urdf.xacro')
    controller_config = os.path.join(get_package_share_directory(package_name), 'config', 'arduino_controllers_fixed.yaml')
    
    # Process XACRO to get URDF content
    from xacro import process_file
    robot_description_content = process_file(urdf_file).toxml()

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False
        }],
        output='screen'
    )

    # Controller Manager Node (ros2_control node) - QUESTO RIMANE ATTIVO!
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            controller_config
        ],
        output='screen'
    )

    # Spawn Joint State Broadcaster (temporaneo)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Spawn Diff Drive Controller (temporaneo)
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner', 
        arguments=['diff_drive_controller'],
        output='screen'
    )

    return LaunchDescription([
        # Start persistent nodes (questi rimangono attivi)
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