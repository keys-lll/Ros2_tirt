import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('mecanum_navigation'), 'config', 'self_filter.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('scan_input_topic', default_value='/scan_raw'),
        DeclareLaunchArgument('scan_output_topic', default_value='/scan'),
        Node(package='laser_filters', executable='scan_to_scan_filter_chain',
             name='scan_to_scan_filter_chain', output='screen', parameters=[config],
             remappings=[('scan', LaunchConfiguration('scan_input_topic')),
                         ('scan_filtered', LaunchConfiguration('scan_output_topic'))]),
    ])
