"""Real PCD localization only; consumes an already running FAST-LIO."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('go2_real_localization')
    return LaunchDescription([
        DeclareLaunchArgument('map_path', description='Absolute path to the real-room PCD; required'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'gicp.yaml')),
        DeclareLaunchArgument('show_rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='True only for rosbag clock replay'),
        Node(package='go2_real_localization', executable='gicp_localizer', name='gicp_localizer',
             output='screen', parameters=[LaunchConfiguration('params_file'), {
                 'map_path': LaunchConfiguration('map_path'),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
        Node(package='rviz2', executable='rviz2', name='real_localization_rviz',
             arguments=['-d', os.path.join(share, 'rviz', 'localization.rviz')],
             parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
             condition=IfCondition(LaunchConfiguration('show_rviz'))),
    ])
