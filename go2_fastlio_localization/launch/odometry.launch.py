"""FAST-LIO odometry for localization; never saves a reference map.

FAST-LIO still maintains its internal local map for scan-to-map odometry.
This is not a pure fixed-map localization algorithm; ICP handles that role.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("go2_fastlio_localization")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="fastlio_odometry",
            output="screen",
            parameters=[
                os.path.join(share, "config", "fast_lio_mid360_sim.yaml"),
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "pcd_save.pcd_save_en": False,
                    "map_file_path": "",
                    "publish.map_en": False,
                    "publish.path_en": False,
                    "publish.effect_map_en": False,
                },
            ],
        ),
    ])
