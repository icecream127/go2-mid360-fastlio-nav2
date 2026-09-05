import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    go2_share = get_package_share_directory("go2_config")
    fast_lio_share = get_package_share_directory("fast_lio")
    workspace_dir = os.path.abspath(
        os.path.join(go2_share, "..", "..", "..", "..")
    )

    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    show_rviz = LaunchConfiguration("show_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    ground_truth_odom = LaunchConfiguration("ground_truth_odom")
    map_output = LaunchConfiguration("map_output")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_share, "launch", "gazebo_mid360.launch.py")
        ),
        launch_arguments={
            "world": world,
            "gui": gui,
            "rviz": "false",
            # Default to the Livox Gazebo plugin.  Set scan:=true only to
            # bring back the old analytic synthetic-pointcloud fallback.
            "scan": "false",
            "ground_truth_odom": ground_truth_odom,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    fast_lio = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        output="screen",
        parameters=[
            os.path.join(go2_share, "config", "fast_lio_mid360_sim.yaml"),
            {"use_sim_time": use_sim_time, "map_file_path": map_output,
             "pcd_save.pcd_save_en": ParameterValue(
                 LaunchConfiguration("save_pcd"), value_type=bool)},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="fast_lio_rviz",
        output="screen",
        arguments=["-d", os.path.join(fast_lio_share, "rviz", "fastlio.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
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
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("ground_truth_odom", default_value="false"),
            DeclareLaunchArgument("save_pcd", default_value="true"),
            DeclareLaunchArgument(
                "map_output",
                default_value=os.path.join(
                    workspace_dir, "maps", "mid360_3d.pcd"
                ),
            ),
            simulation,
            TimerAction(period=8.0, actions=[fast_lio]),
            TimerAction(period=12.0, actions=[rviz]),
        ]
    )
