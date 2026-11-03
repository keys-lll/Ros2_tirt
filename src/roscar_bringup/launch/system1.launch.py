from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory
import xacro

def generate_launch_description():
    ld = LaunchDescription()

    # pkg_roscar_description = get_package_share_directory('roscar_description')
    # pkg_roscar_bringup = get_package_share_directory('roscar_bringup')
    # rviz_file = os.path.join(get_package_share_directory('roscar_bringup'), 'rviz', 'roscar.rviz')


    pkg_roscar_description = get_package_share_directory('roscar_description')
    pkg_roscar_bringup = get_package_share_directory('roscar_bringup')
    rviz_file = os.path.join(pkg_roscar_bringup, 'rviz', 'roscar.rviz')

    xacro_path = os.path.join(pkg_roscar_description, 'urdf', 'roscar.urdf.xacro')
    doc = xacro.parse(open(xacro_path))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    


    stm32_bridge = Node(package='stm32', executable='stm32_bridge', output='screen',
                        parameters=[os.path.join(pkg_roscar_bringup, 'config', 'stm32.yaml')])

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lslidar_driver'),
                         'launch/lsn10p_launch.py')
        )
    )

    urdf_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description,
                    {'use_sim_time':False}],
    )

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'use_sim_time': False},
            {'publish_default_positions': True},
        ]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_roscar_bringup, 'config', 'ekf.yaml')],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file]
    )

    ld.add_action(stm32_bridge)
    ld.add_action(lidar_launch)
    ld.add_action(urdf_node)
    ld.add_action(joint_state_pub)
    ld.add_action(ekf_node)
    ld.add_action(rviz_node)

    return ld