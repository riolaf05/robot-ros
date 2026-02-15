import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    package_name='robot_ros' 
  
    #“Include” our own rsp.launch.py, from our package, and force use_sim_time to be true
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    # Nota: Hardware interface è ora gestito da ros2_control nel file rsp.launch.py
    # Il nodo cmdVel_to_pwm_node custom è deprecato e sostituito da DiffDriveArduino
  
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

    # LIDAR RPLIDAR
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB1',
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Standard'
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

    # Launch them all!
    return LaunchDescription([
        rsp,
        lidar_node,
        camera_node,
        rosbridge_node
    ])

