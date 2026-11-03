import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = share('mecanum_navigation')
    def include(package, filename, arguments=None, condition=None):
        options = {'launch_arguments': (arguments or {}).items()}
        if condition is not None:
            options['condition'] = condition
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share(package), 'launch', filename)), **options)
    actions = [
        DeclareLaunchArgument('start_hardware', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('stm32_port', default_value='/dev/serial/by-id/usb-1a86_USB_Single_Serial_597B018072-if00'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5893011986-if00'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        include('mecanum_navigation', 'hardware.launch.py',
                {key: LaunchConfiguration(key) for key in ('stm32_port', 'lidar_port', 'use_lidar')},
                IfCondition(LaunchConfiguration('start_hardware'))),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ]
    actions.append(include('slam_toolbox', 'online_async_launch.py', {
        'use_sim_time': 'false',
        'slam_params_file': os.path.join(pkg, 'config', 'slam_toolbox.yaml'),
    }))
    return LaunchDescription(actions)
