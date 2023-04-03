import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='my_bot_test',
            executable='talker',
            name='talker'),
  ])