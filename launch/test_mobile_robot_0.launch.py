import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('behaviour_trees')) 
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params] #robot_state_publisher richiede il file URDF
    )

    # Create a static transform publisher node for the map to odom transform and the base_footprint to base_link transform
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    static_tf_publisher_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_broadcaster',
        arguments=['0', '0', '0.09', '0', '0', '0', 'base_footprint', 'base_link', '30']
    )

    navbehaivortreenode = Node(
        package='behaviour_trees',
        executable='nav_behaviour_tree',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
 
        static_tf_publisher,
        static_tf_publisher_base_footprint,
        node_robot_state_publisher,
        #navbehaivortreenode
    ])
    
