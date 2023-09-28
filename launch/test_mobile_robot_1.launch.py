import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='robot_ros' 
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/rviz_config.rviz')
    default_world_path = os.path.join(pkg_share, 'worlds/my_house_v1.world')
    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_dir, 'launch')
    nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    behavior_tree_xml_path = os.path.join(nav2_bt_path, 'robot_ros', 'navigate_w_replanning_and_recovery.xml')
    static_map_path = os.path.join(pkg_share, 'maps', 'map.yaml')
    
    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    world = LaunchConfiguration('world')
    namespace = LaunchConfiguration('namespace')
    slam = LaunchConfiguration('slam')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')

    # Declare the launch arguments  
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace'
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        name='use_namespace',
        default_value='False',
        description='Whether to apply a namespace to the topic names'
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        name='use_namespace',
        default_value='False',
        description='Whether to apply a namespace to the topic names'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        name='map',
        default_value=static_map_path,
        description='Full path to map file to load'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        name='params_file',
        default_value=nav2_params_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_bt_xml_cmd = DeclareLaunchArgument(
        name='default_bt_xml_filename',
        default_value=behavior_tree_xml_path,
        description='Full path to the behavior tree xml file to use'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        name='map',
        default_value=default_world_path,
        description='Full path to map file to load'
    )

    declare_slam_cmd = DeclareLaunchArgument(
        name='slam',
        default_value='False',
        description='Whether to run SLAM'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use'
    )
        
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=default_world_path,
        description='Full path to world model file to load'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        name='autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    #“Include” our own rsp.launch.py, from our package, and force use_sim_time to be true
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','test_mobile_robot_0.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]), 

    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'robot_ros'],
                        output='screen'
    )

    # Run telop twist keyboard
    rqt_robot_steering_node = Node(
            package='rqt_robot_steering',
            node_executable='rqt_robot_steering',
            output='screen'
    )
    
    # Rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )    
    
    # Launch the ROS 2 Navigation Stack
    start_ros2_navigation_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')), 
    launch_arguments = {'namespace': namespace,
                        'use_namespace': use_namespace,
                        'slam': slam,
                        'map': map_yaml_file,
                        'use_sim_time': use_sim_time,
                        'params_file': params_file,
                        'default_bt_xml_filename': default_bt_xml_filename,
                        'autostart': autostart}.items()
    )

    start_slam_toolbox_node = Node(
        parameters=[
          get_package_share_directory("robot_ros") + '/config/mapper.yaml',
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        node_executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    # Launch them all!
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_autostart_cmd) 
    ld.add_action(declare_world_cmd) 
    ld.add_action(declare_slam_cmd)

    # Launch nodes
    ld.add_action(rsp) #launch file con robot description
    ld.add_action(gazebo) #attiva gazebo
    ld.add_action(spawn_entity) #spawna il robot in gazebo
    #ld.add_action(rqt_robot_steering_node) #per muovere il robot
    # ld.add_action(rviz_node) #attiva rviz
    # ld.add_action(start_ros2_navigation_cmd) #attiva il navigation stack
    # ld.add_action(start_slam_toolbox_node) #attiva il slam_toolbox
    return ld

