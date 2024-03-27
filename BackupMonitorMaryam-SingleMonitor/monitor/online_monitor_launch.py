import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='monitor',
            executable='online_monitor_FUAV',
            name='online_monitor_FUAV'),
  ])


