import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Process the robot description file..
    pkg_share = FindPackageShare(package='robot_ros').find('robot_ros')
    pkg_path = os.path.join(get_package_share_directory('robot_ros'))

    ## read from xacro file..
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    ## read from urdf file..
    # urdf_file_name = os.path.join(pkg_path,'description','my_robot.urdf')
    # urdf=open(urdf_file_name).read()
    
    use_sim_time = LaunchConfiguration('use_sim_time')
                
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}], #xacro
            # parameters = {'robot_description': urdf, 'use_sim_time': use_sim_time} #urdf
         ),

        # Node(
        #     package='urdf_tutorial',
        #     executable='state_publisher',
        #     name='state_publisher',
        #     output='screen'),

        # Node(
        #     #this is to publish odom -> base_link
        #     package='robot_localization',
        #     executable='ekf_node', #applica il kalman filter
        #     name='ekf_filter_node',
        #     output='screen',
        #     parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        # ),

        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher'
        # ),
    ])
