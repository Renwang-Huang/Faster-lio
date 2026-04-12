import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    faster_lio_dir = get_package_share_directory('faster_lio')

    declare_rviz_enable = DeclareLaunchArgument('rviz', default_value='false', description='Enable RViz')
    declare_localization_mode = DeclareLaunchArgument('localization_mode', default_value='false')

    faster_lio_node = Node(
        package='faster_lio',
        executable='run_mapping_online',
        name='laserMapping',
        output='screen',
        parameters=[
            os.path.join(faster_lio_dir, 'config', 'mid360.yaml'),
            {'localization_mode_en': LaunchConfiguration('localization_mode')}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(faster_lio_dir, 'rviz_cfg', 'loam_livox.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        declare_rviz_enable,
        declare_localization_mode,
        faster_lio_node,
        rviz_node,
    ])