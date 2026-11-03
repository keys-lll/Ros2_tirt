import os
import xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot = xacro.process_file(os.path.join(share('roscar_description'), 'urdf', 'roscar.urdf.xacro')).toxml()
    lidar_config = os.path.join(share('lslidar_driver'), 'params', 'lidar_uart_ros2', 'lsn10p.yaml')
    filter_launch = os.path.join(share('mecanum_navigation'), 'launch', 'scan_filter.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('stm32_port', default_value='/dev/serial/by-id/usb-1a86_USB_Single_Serial_597B018072-if00'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5893011986-if00'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('require_fresh_scan', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(filter_launch)),
        Node(package='stm32', executable='stm32_bridge', output='screen',
             parameters=[os.path.join(share('roscar_bringup'), 'config', 'stm32.yaml'),
                         {'port': LaunchConfiguration('stm32_port'), 'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                          'require_fresh_scan': ParameterValue(LaunchConfiguration('require_fresh_scan'), value_type=bool)}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot, 'use_sim_time': False}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[os.path.join(share('roscar_bringup'), 'config', 'ekf.yaml')], output='screen'),
        Node(package='lslidar_driver', executable='lslidar_driver_node', name='lslidar_driver_node',
             condition=IfCondition(LaunchConfiguration('use_lidar')),
             parameters=[lidar_config, {'serial_port_': LaunchConfiguration('lidar_port'), 'scan_topic': '/scan_raw'}], output='screen'),
    ])
