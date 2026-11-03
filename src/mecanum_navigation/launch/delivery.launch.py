import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    directory = os.path.join(get_package_share_directory('mecanum_navigation'), 'launch')
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to the saved map YAML'),
        DeclareLaunchArgument('start_hardware', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('start_camera', default_value='false'),
        DeclareLaunchArgument('start_qr', default_value='true'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('start_mission_gui', default_value='true'),
        DeclareLaunchArgument('start_operator_gui', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(directory, 'navigation.launch.py')),
             launch_arguments={key: LaunchConfiguration(key) for key in ('map','start_hardware','use_rviz')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(directory, 'delivery_camera.launch.py')),
             launch_arguments={key: LaunchConfiguration(key) for key in ('start_camera','start_qr','video_device','image_topic')}.items()),
        Node(package='auto', executable='mission_gui', name='delivery_mission_gui', output='screen',
             condition=IfCondition(LaunchConfiguration('start_mission_gui'))),
        Node(package='auto', executable='operator_gui', name='delivery_operator_gui', output='screen',
             condition=IfCondition(LaunchConfiguration('start_operator_gui'))),
    ])
