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
        default_value=os.path.join(pkg_dir, 'configs', 'relocalization', 'reloc_euroc.yaml'),
    )
    dataroot_arg = DeclareLaunchArgument(
        'dataroot',
        default_value='/media/data/datasets/euroc/seq/V1_01_easy/cam0/data',
    )
    camera_config_path_arg = DeclareLaunchArgument(
        'camera_config_path',
        default_value=os.path.join(pkg_dir, 'configs', 'camera', 'euroc.yaml'),
    )
    model_dir_arg = DeclareLaunchArgument(
        'model_dir',
        default_value=os.path.join(pkg_dir, 'output'),
    )
    traj_path_arg = DeclareLaunchArgument(
        'traj_path',
        default_value=os.path.join(pkg_dir, 'debug', 'relocalization.txt'),
    )
    voc_path_arg = DeclareLaunchArgument(
        'voc_path',
        default_value=os.path.join(pkg_dir, 'point_voc_L4.bin'),
    )
    map_root_arg = DeclareLaunchArgument(
        'map_root',
        default_value=os.path.join(pkg_dir, 'debug', 'test'),
    )
    visualization_arg = DeclareLaunchArgument(
        'visualization',
        default_value='true',
    )

    relocalization_node = Node(
        package='air_slam',
        executable='relocalization',
        name='relocalization',
        output='screen',
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
            'dataroot': LaunchConfiguration('dataroot'),
            'camera_config_path': LaunchConfiguration('camera_config_path'),
            'model_dir': LaunchConfiguration('model_dir'),
            'traj_path': LaunchConfiguration('traj_path'),
            'voc_path': LaunchConfiguration('voc_path'),
            'map_root': LaunchConfiguration('map_root'),
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(pkg_dir, 'airslam.rviz')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('visualization')),
    )

    return LaunchDescription([
        config_path_arg,
        dataroot_arg,
        camera_config_path_arg,
        model_dir_arg,
        traj_path_arg,
        voc_path_arg,
        map_root_arg,
        visualization_arg,
        relocalization_node,
        rviz_node,
    ])
