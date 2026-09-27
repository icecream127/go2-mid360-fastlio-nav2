"""Use Xju FAST-LIO's upstream launch and one project-owned real sensor profile."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    project = get_package_share_directory('go2_fastlio_localization')
    upstream = get_package_share_directory('fast_lio')
    return LaunchDescription([
        DeclareLaunchArgument('show_rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(upstream, 'launch', 'mapping.launch.py')),
            launch_arguments={
                'config_path': os.path.join(project, 'config'),
                'config_file': 'fast_lio_mid360_real.yaml',
                'use_sim_time': 'false',
                'rviz': LaunchConfiguration('show_rviz'),
                'rviz_cfg': os.path.join(project, 'config', 'real_mapping.rviz'),
            }.items(),
        ),
    ])
