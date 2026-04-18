import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    faster_lio_dir = get_package_share_directory('faster_lio')
    
    config_path = os.path.join(faster_lio_dir, 'config', 'LivoxMid360.yaml')
    rviz_config_path = os.path.join(faster_lio_dir, 'config', 'rviz', 'faster_lio.rviz')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Whether to start RViz'
    )

    laser_mapping_node = Node(
        package='faster_lio',
        executable='run_mapping_online',  
        name='laserMapping',
        output='screen',
        parameters=[
            config_path,  
            {'runtime_pos_log_enable': False}  
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        laser_mapping_node,
        rviz_node
    ])
    