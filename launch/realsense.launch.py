import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('air_vo')

    args = [
        DeclareLaunchArgument('config_path',
                              default_value=os.path.join(pkg_dir, 'configs', 'configs_realsense.yaml')),
        DeclareLaunchArgument('dataroot',
                              default_value='/data/xukuan/cartin_meeting/euroc_style'),
        DeclareLaunchArgument('camera_config_path',
                              default_value=os.path.join(pkg_dir, 'configs', 'realsense_848_480.yaml')),
        DeclareLaunchArgument('model_dir',
                              default_value=os.path.join(pkg_dir, 'output')),
        DeclareLaunchArgument('saving_dir',
                              default_value=os.path.join(pkg_dir, 'debug')),
        DeclareLaunchArgument('traj_path',
                              default_value=os.path.join(pkg_dir, 'debug', 'traj.txt')),
        DeclareLaunchArgument('visualization', default_value='true'),
    ]

    air_vo_node = Node(
        package='air_vo',
        executable='air_vo',
        name='air_vo',
        output='screen',
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
            'dataroot': LaunchConfiguration('dataroot'),
            'camera_config_path': LaunchConfiguration('camera_config_path'),
            'model_dir': LaunchConfiguration('model_dir'),
            'saving_dir': LaunchConfiguration('saving_dir'),
            'traj_path': LaunchConfiguration('traj_path'),
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'airvo.rviz')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('visualization')),
    )

    return LaunchDescription(args + [air_vo_node, rviz_node])
