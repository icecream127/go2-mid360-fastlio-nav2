"""Optional combined entry using the same acquisition and mapping launches."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('go2_fastlio_localization')
    return LaunchDescription([
        DeclareLaunchArgument('show_rviz', default_value='true'),
        DeclareLaunchArgument('user_config_path', default_value=os.path.join(share, 'config', 'mid360_real.json')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'real_lidar.launch.py')),
            launch_arguments={'user_config_path': LaunchConfiguration('user_config_path'), 'xfer_format': '1'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'real_fastlio.launch.py')),
            launch_arguments={'show_rviz': LaunchConfiguration('show_rviz')}.items(),
        ),
    ])
