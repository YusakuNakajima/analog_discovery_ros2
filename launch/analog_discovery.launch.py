# analog_discovery.launch.py
#
# Analog Discovery のオシロ入力をトリガ取り込みし、~/capture（analog_discovery_ros2/msg/ScopeCapture）
# と ~/ch1_peak_v, ~/ch2_peak_v（std_msgs/Float32）に流す。
#   ros2 launch analog_discovery_ros2 analog_discovery.launch.py backend:=mock
# 取り込み設定は config/analog_discovery.yaml。

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration('config')
    backend = LaunchConfiguration('backend')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('analog_discovery_ros2'), 'config', 'analog_discovery.yaml']),
            description='parameter file'),
        DeclareLaunchArgument('backend', default_value='dwf', description='dwf | mock'),
        Node(
            package='analog_discovery_ros2',
            executable='analog_discovery_node.py',
            name='analog_discovery',
            output='screen',
            parameters=[config, {'backend': backend}],
        ),
    ])
