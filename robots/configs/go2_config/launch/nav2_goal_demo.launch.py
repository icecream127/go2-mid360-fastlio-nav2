import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    go2_share = get_package_share_directory("go2_config")
    nav2_share = get_package_share_directory("nav2_bringup")
    champ_navigation_share = get_package_share_directory("champ_navigation")

    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    show_rviz = LaunchConfiguration("show_rviz")
    params_file = LaunchConfiguration("params_file")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_share, "launch", "gazebo_mid360.launch.py")
        ),
        launch_arguments={
            "world": world,
            "gui": gui,
            "rviz": "false",
            "scan": "true",
            "ground_truth_odom": "true",
            "use_sim_time": "true",
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_share, "launch", "slam_mid360.launch.py")
        ),
        launch_arguments={"rviz": "false"}.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "autostart": "true",
            "params_file": params_file,
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(champ_navigation_share, "rviz", "navigation.rviz"),
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(show_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(go2_share, "worlds", "mid360_mapping.world"),
            ),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("show_rviz", default_value="true"),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    go2_share, "config", "autonomy", "navigation.yaml"
                ),
            ),
            simulation,
            TimerAction(period=6.0, actions=[slam, navigation]),
            TimerAction(period=10.0, actions=[rviz_node]),
        ]
    )
