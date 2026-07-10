import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('air_slam')

    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=os.path.join(pkg_dir, 'configs', 'visual_odometry', 'vo_euroc_dark.yaml'),
    )
    dataroot_arg = DeclareLaunchArgument(
        'dataroot',
        default_value='/media/data/datasets/euroc/dark_euroc/sequences/09',
    )
    camera_config_path_arg = DeclareLaunchArgument(
        'camera_config_path',
        default_value=os.path.join(pkg_dir, 'configs', 'camera', 'dark_euroc.yaml'),
    )
    model_dir_arg = DeclareLaunchArgument(
        'model_dir',
        default_value=os.path.join(pkg_dir, 'output'),
    )
    saving_dir_arg = DeclareLaunchArgument(
        'saving_dir',
        default_value=os.path.join(pkg_dir, 'debug'),
    )
    visualization_arg = DeclareLaunchArgument(
        'visualization',
        default_value='true',
    )

    visual_odometry_node = Node(
        package='air_slam',
        executable='visual_odometry',
        name='visual_odometry',
        output='screen',
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
            'dataroot': LaunchConfiguration('dataroot'),
            'camera_config_path': LaunchConfiguration('camera_config_path'),
            'model_dir': LaunchConfiguration('model_dir'),
            'saving_dir': LaunchConfiguration('saving_dir'),
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'vo.rviz')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('visualization')),
    )

    return LaunchDescription([
        config_path_arg,
        dataroot_arg,
        camera_config_path_arg,
        model_dir_arg,
        saving_dir_arg,
        visualization_arg,
        visual_odometry_node,
        rviz_node,
    ])
