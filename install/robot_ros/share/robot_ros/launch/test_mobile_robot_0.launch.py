import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                    ('/tf_static', 'tf_static')]

    # Process the robot description file..
    pkg_share = FindPackageShare(package='robot_ros').find('robot_ros')
    
    # With Xacro file..
    pkg_path = os.path.join(get_package_share_directory('robot_ros')) 
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    
    # Parameters
    model = LaunchConfiguration('model')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments  
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        name='use_namespace',
        default_value='False',
        description='Whether to apply a namespace to the navigation stack')
                
    # Nodes
    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher')
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params] #robot_state_publisher richiede il file URDF
    )
    
    robot_localization_node = Node(
         package='robot_localization',
         executable='ekf_node', #applica il kalman filter
         name='ekf_filter_node',
         output='screen',
         parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    ld = LaunchDescription()
    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)

    # Add any actions
    ld.add_action(start_robot_state_publisher_cmd)
    #ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(robot_localization_node)
    #ld.add_action(start_joint_state_publisher_gui_node)
    #ld.add_action(static_tf_publisher)

    return ld
