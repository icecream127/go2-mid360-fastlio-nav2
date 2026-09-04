import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_share = get_package_share_directory("go2_config")
    slam_share = get_package_share_directory("slam_toolbox")
    nav2_share = get_package_share_directory("nav2_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("slam_params_file")
    rviz = LaunchConfiguration("rviz")

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": params_file,
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_slam",
        arguments=[
            "-d",
            os.path.join(nav2_share, "rviz", "nav2_default_view.rviz"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=os.path.join(
                    config_share, "config", "autonomy", "slam.yaml"
                ),
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            slam,
            rviz_node,
        ]
    )
