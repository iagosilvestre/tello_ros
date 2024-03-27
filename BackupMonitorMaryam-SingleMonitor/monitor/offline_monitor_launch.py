import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='monitor',
            executable='offline_monitor_FUAV',
            name='offline_monitor_FUAV'),
  ])


