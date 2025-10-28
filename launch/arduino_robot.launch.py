#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Package directories
    pkg_robot_ros = get_package_share_directory('robot_ros')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/ttyACM0',
        description='Arduino serial device'
    )
    
    # Robot description
    robot_description_file = PathJoinSubstitution([
        FindPackageShare('robot_ros'),
        'description',
        'robot.urdf.xacro'
    ])
    
    robot_description = Command([
        'xacro ', robot_description_file,
        ' sim_mode:=false'
    ])
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Controller manager
    controller_params = PathJoinSubstitution([
        FindPackageShare('robot_ros'),
        'config',
        'arduino_controllers_fixed.yaml'
    ])
    
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            controller_params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                name='joint_state_broadcaster_spawner',
                arguments=['joint_state_broadcaster'],
                output='screen'
            )
        ]
    )
    
    # Diff drive controller spawner
    diff_drive_controller_spawner = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                name='diff_drive_controller_spawner',
                arguments=['diff_drive_controller'],
                output='screen'
            )
        ]
    )
    
    # Teleop twist keyboard
    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        remappings=[
            ('cmd_vel', 'diff_drive_controller/cmd_vel_unstamped')
        ],
        prefix='xterm -e'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        device_arg,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        teleop_keyboard
    ])