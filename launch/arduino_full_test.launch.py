#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("robot_ros"), "description", "robot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Controller parameters directly in the launch file
    controller_config = {
        "controller_manager": {
            "ros__parameters": {
                "update_rate": 100,
                "diff_drive_controller": {
                    "type": "diff_drive_controller/DiffDriveController"
                },
                "joint_state_broadcaster": {
                    "type": "joint_state_broadcaster/JointStateBroadcaster"
                }
            }
        },
        "diff_drive_controller": {
            "ros__parameters": {
                "left_wheel_names": ["base_left_wheel_joint"],
                "right_wheel_names": ["base_right_wheel_joint"],
                "wheel_separation": 0.287,
                "wheel_radius": 0.033,
                "use_stamped_vel": False,
                "publish_limited_velocity": True,
                "velocity_rolling_window_size": 10,
                "publish_wheel_data": True,
                "publish_odom": True,
                "publish_odom_tf": True,
                "open_loop": False,
                "enable_odom_tf": True,
                "odom_frame_id": "odom",
                "base_frame_id": "base_link",
                "pose_covariance_diagonal": [0.001, 0.001, 0.0, 0.0, 0.0, 0.01],
                "twist_covariance_diagonal": [0.001, 0.0, 0.0, 0.0, 0.0, 0.01],
                "cmd_vel_timeout": 0.25
            }
        },
        "joint_state_broadcaster": {
            "ros__parameters": {}
        }
    }

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        output="both",
    )

    # Spawn controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay controller spawning
    delay_joint_state_spawner_after_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[joint_state_broadcaster_spawner],
                )
            ],
        )
    )

    delay_diff_drive_spawner_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[diff_drive_controller_spawner],
                )
            ],
        )
    )

    return LaunchDescription([
        robot_state_pub_node,
        control_node,
        delay_joint_state_spawner_after_control_node,
        delay_diff_drive_spawner_after_joint_state,
    ])