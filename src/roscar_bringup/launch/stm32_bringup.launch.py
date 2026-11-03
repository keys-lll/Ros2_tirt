import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('stm32_port', default_value='/dev/serial/by-id/usb-1a86_USB_Single_Serial_597B018072-if00'),
        Node(package='stm32', executable='stm32_bridge', output='screen',
             parameters=[os.path.join(get_package_share_directory('roscar_bringup'), 'config', 'stm32.yaml'),
                         {'port': LaunchConfiguration('stm32_port')}]),
    ])
