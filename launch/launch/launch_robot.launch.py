import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='my_bot_test' 
  
    #“Include” our own rsp.launch.py, from our package, and force use_sim_time to be true
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    #Hardware interface
    hdw_interface = Node(
        package='my_bot_test',
        executable='cmdVel_to_pwm_node',
        output='screen',
        parameters=[] #robot_state_publisher richiede il file URDF
    )

    #Controller manager: 
    #Unfortunately the controller manager can’t just read it from 
    #the /robot_description topic the way the gazebo_ros2_control plugin does
    # robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    # controller_params = os.path.join(
    #     get_package_share_directory('articubot_one'), # <-- Replace with your package name
    #     'config',
    #     'my_controllers.yaml'
    #     )
    # controller_manager = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[{'robot_description': robot_description}, controller_params],
    #     )


    # Launch them all!
    return LaunchDescription([
        rsp,
        hdw_interface,
        # controller_manager
    ])

