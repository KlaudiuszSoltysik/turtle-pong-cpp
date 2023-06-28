import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'),
        launch_ros.actions.Node(
            package='game',
            executable='game',
            name='game'),
        ])