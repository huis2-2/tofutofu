import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('autorace_slow'), 'config', 'slow.yaml')
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='autorace_slow', executable='slow_node', name='slow_node', output='screen',
            parameters=[config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
