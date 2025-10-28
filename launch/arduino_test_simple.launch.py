#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    
    # Controller manager con solo hardware interface
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {
                'controller_manager': {
                    'ros__parameters': {
                        'update_rate': 30,
                        'joint_state_broadcaster': {
                            'type': 'joint_state_broadcaster/JointStateBroadcaster'
                        }
                    }
                }
            }
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        device_arg,
        robot_state_publisher,
        controller_manager
    ])