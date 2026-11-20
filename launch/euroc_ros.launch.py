import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('air_vo')

    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=os.path.join(pkg_dir, 'configs', 'configs_euroc.yaml'),
    )
    camera_config_path_arg = DeclareLaunchArgument(
        'camera_config_path',
        default_value=os.path.join(pkg_dir, 'configs', 'euroc.yaml'),
    )
    model_dir_arg = DeclareLaunchArgument(
        'model_dir',
        default_value=os.path.join(pkg_dir, 'output'),
    )
    saving_dir_arg = DeclareLaunchArgument(
        'saving_dir',
        default_value=os.path.join(pkg_dir, 'debug'),
    )
    traj_path_arg = DeclareLaunchArgument(
        'traj_path',
        default_value=os.path.join(pkg_dir, 'debug', 'traj.txt'),
    )
    left_topic_arg = DeclareLaunchArgument('left_topic', default_value='/cam0/image_raw')
    right_topic_arg = DeclareLaunchArgument('right_topic', default_value='/cam1/image_raw')
    visualization_arg = DeclareLaunchArgument('visualization', default_value='true')

    air_vo_node = Node(
        package='air_vo',
        executable='air_vo_ros',
        name='air_vo_ros',
        output='screen',
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
            'camera_config_path': LaunchConfiguration('camera_config_path'),
            'model_dir': LaunchConfiguration('model_dir'),
            'saving_dir': LaunchConfiguration('saving_dir'),
            'traj_path': LaunchConfiguration('traj_path'),
            'left_topic': LaunchConfiguration('left_topic'),
            'right_topic': LaunchConfiguration('right_topic'),
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

    return LaunchDescription([
        config_path_arg,
        camera_config_path_arg,
        model_dir_arg,
        saving_dir_arg,
        traj_path_arg,
        left_topic_arg,
        right_topic_arg,
        visualization_arg,
        air_vo_node,
        rviz_node,
    ])
