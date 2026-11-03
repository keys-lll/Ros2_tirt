from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_qr', default_value='true'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        Node(package='usb_cam', executable='usb_cam_node_exe', name='delivery_camera',
             condition=IfCondition(LaunchConfiguration('start_camera')), output='screen',
             parameters=[{'video_device': LaunchConfiguration('video_device')}],
             remappings=[('image_raw', LaunchConfiguration('image_topic'))]),
        Node(package='zbar_opencv', executable='zbar_opencv', name='zbar_opencv', output='screen',
             condition=IfCondition(LaunchConfiguration('start_qr')),
             remappings=[('/image_raw', LaunchConfiguration('image_topic'))]),
    ])
