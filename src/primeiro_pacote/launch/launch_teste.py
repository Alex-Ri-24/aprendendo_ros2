from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            name='nivel_log',
            default_value='INFO',
            choices=
['DEBUG', 'INFO','WARN','ERROR','FATAL'],
            description='Flag to set log level'
        ),

        Node(
            name='no_simples1',
            package='primeiro_pacote',
            executable='ola',
            arguments=
['--ros-args', '--log-level', LaunchConfiguration('nivel_log')]
        ),

        Node(
            name='no_simples2',
            package='primeiro_pacote',
            executable='ola',
            arguments=
['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
    ])
