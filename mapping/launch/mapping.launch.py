#!/usr/bin/env python3
"""TIRT Cartographer mapping, defaulting to the current STM32 mecanum hardware."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    workspace = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_dir = os.path.join(workspace, 'config')
    config_name = 'car_2d.lua'
    config_path = os.path.join(config_dir, config_name)
    if not os.path.isfile(config_path):
        raise FileNotFoundError(f'找不到 Cartographer 設定：{config_path}')

    hardware = LaunchConfiguration('hardware').perform(context)
    if hardware == 'mecanum':
        robot_launch = os.path.join(get_package_share_directory('mecanum_navigation'), 'launch', 'hardware.launch.py')
        hardware_args = {key: LaunchConfiguration(key) for key in ('stm32_port', 'lidar_port', 'use_lidar')}
        default_odom = '/odometry/filtered'
    elif hardware == 'legacy':
        robot_launch = os.path.join(workspace, 'launch', 'robot.launch.py')
        hardware_args = {}
        default_odom = '/odom'
    else:
        raise ValueError('hardware 必須是 mecanum 或 legacy')
    if not os.path.isfile(robot_launch):
        raise FileNotFoundError(f'找不到硬體啟動檔：{robot_launch}')
    odom_topic = LaunchConfiguration('odom_topic').perform(context) or default_odom

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch),
        launch_arguments=hardware_args.items(),
        condition=IfCondition(LaunchConfiguration('start_hardware')),
    )
    cartographer = Node(
        package='cartographer_ros', executable='cartographer_node',
        name='cartographer_node', output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-configuration_directory', config_dir, '-configuration_basename', config_name],
        remappings=[('scan', '/scan'), ('odom', odom_topic)],
    )
    occupancy_grid = Node(
        package='cartographer_ros', executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node', output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(workspace, 'rviz2', 'slam.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )
    return [robot, TimerAction(period=2.0, actions=[cartographer, occupancy_grid]),
            TimerAction(period=3.0, actions=[rviz])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware', default_value='mecanum', choices=['mecanum', 'legacy']),
        DeclareLaunchArgument('start_hardware', default_value='true', description='False if current hardware nodes are already running'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('stm32_port', default_value='/dev/serial/by-id/usb-1a86_USB_Single_Serial_597B018072-if00'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5893011986-if00'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('odom_topic', default_value='', description='Empty selects /odometry/filtered for mecanum, /odom for legacy'),
        OpaqueFunction(function=launch_setup),
    ])
