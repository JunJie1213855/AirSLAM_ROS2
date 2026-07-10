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
        default_value=os.path.join(pkg_dir, 'configs', 'map_refinement', 'mr_euroc.yaml'),
    )
    map_root_arg = DeclareLaunchArgument(
        'map_root',
        default_value=os.path.join(pkg_dir, 'debug', 'test'),
    )
    model_dir_arg = DeclareLaunchArgument(
        'model_dir',
        default_value=os.path.join(pkg_dir, 'output'),
    )
    voc_path_arg = DeclareLaunchArgument(
        'voc_path',
        default_value=os.path.join(pkg_dir, 'voc', 'point_voc_L4.bin'),
    )
    breakpoint_arg = DeclareLaunchArgument(
        'breakpoint',
        default_value='0',
    )
    visualization_arg = DeclareLaunchArgument(
        'visualization',
        default_value='true',
    )

    map_refinement_node = Node(
        package='air_slam',
        executable='map_refinement',
        name='map_refinement',
        output='screen',
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
            'map_root': LaunchConfiguration('map_root'),
            'model_dir': LaunchConfiguration('model_dir'),
            'voc_path': LaunchConfiguration('voc_path'),
            'breakpoint': LaunchConfiguration('breakpoint'),
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'map_optimization.rviz')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('visualization')),
    )

    return LaunchDescription([
        config_path_arg,
        map_root_arg,
        model_dir_arg,
        voc_path_arg,
        breakpoint_arg,
        visualization_arg,
        map_refinement_node,
        rviz_node,
    ])
