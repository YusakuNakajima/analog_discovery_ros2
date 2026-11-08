# ae_record.launch.py
#
# Continuous AE recording on request (ae_record_node.py; ROS 1 acoustic_emission_service.py).
#   ros2 launch analog_discovery_ros2 ae_record.launch.py record_dir:=/path/to/ae
#   ros2 service call /ae_recorder/command analog_discovery_ros2/srv/AeRecordCommand "{command: start}"
#   ros2 service call /ae_recorder/command analog_discovery_ros2/srv/AeRecordCommand "{command: get}"
# Settings: config/ae_record.yaml. Not together with analog_discovery.launch.py (one device).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('analog_discovery_ros2'), 'config', 'ae_record.yaml']),
            description='parameter file'),
        DeclareLaunchArgument('backend', default_value='dwf', description='dwf | mock'),
        DeclareLaunchArgument('record_dir', description='directory for the recordings (required)'),
        DeclareLaunchArgument('feature_log_csv', default_value='',
                              description='append one row per recording; empty = off'),
        Node(
            package='analog_discovery_ros2',
            executable='ae_record_node.py',
            name='ae_recorder',
            output='screen',
            parameters=[LaunchConfiguration('config'), {
                'backend': LaunchConfiguration('backend'),
                'record_dir': LaunchConfiguration('record_dir'),
                'feature_log_csv': LaunchConfiguration('feature_log_csv'),
            }],
        ),
    ])
