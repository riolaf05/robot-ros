import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():

    package_name='robot_ros' 
    use_sim_time='false'
  
    #“Include” our own rsp.launch.py, from our package, and force use_sim_time to be true
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    #Hardware (motors) interface (custom)
    hdw_interface = Node(
        package='robot_ros',
        executable='cmdVel_to_pwm_node',
        output='screen',
        parameters=[] 
    )
    
    #Camera
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        output='screen',
        parameters=[{
           #'image_size': [640,480],
           #'camera_frame_id': 'camera_link_optical'
           'video_device': '/dev/video0'
            }]
    )

    #Rosbridge
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9090
            }]
    )

    # Controller manager: 
    # See: https://articulatedrobotics.xyz/mobile-robot-13-ros2-control-real/
    # Unfortunately the controller manager can’t just read it from 
    # the /robot_description topic the way the gazebo_ros2_control plugin does
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    
    controller_params = os.path.join(
        get_package_share_directory(package_name), 
        'config',
        'my_controllers.yaml'
        )
    
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controller_params],
        )
    
    # #We need the robot state publisher to have finished starting up before we run the ros2 param get command
    delayed_controller_manager = TimerAction(period=3.0,actions=[controller_manager])

    # #We can also then delay the controller spawners until the controller manager has started.
    # diff_drive_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner.py",
    #     arguments=["diff_cont"],
    # )

    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=controller_manager,
    #         on_start=[diff_drive_spawner],
    #     )
    # )

    # joint_broad_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner.py",
    #     arguments=["joint_broad"],
    # )

    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=controller_manager,
    #         on_start=[diff_drive_spawner],
    #     )
    # )
    # delayed_joint_broad_spawner = RegisterEventHandler(
    #         event_handler=OnProcessStart(
    #             target_action=controller_manager,
    #             on_start=[joint_broad_spawner],
    #         )
    #     )


    # Launch them all!
    return LaunchDescription([
        #robot_state_publisher, hw interface, camera, rosbridge
        rsp,
        hdw_interface,
        camera_node,
        rosbridge_node,
        #ros_control
        delayed_controller_manager,
        # delayed_diff_drive_spawner,
        # delayed_joint_broad_spawner
    ])

