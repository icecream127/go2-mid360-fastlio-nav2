import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    go2_share = get_package_share_directory("go2_config")
    nav2_share = get_package_share_directory("nav2_bringup")
    champ_navigation_share = get_package_share_directory("champ_navigation")
    workspace_dir = os.path.abspath(
        os.path.join(go2_share, "..", "..", "..", "..")
    )

    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    show_rviz = LaunchConfiguration("show_rviz")
    nav_rviz = LaunchConfiguration("nav_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    pcd_map = LaunchConfiguration("pcd_map")
    nav_map = LaunchConfiguration("nav_map")
    params_file = LaunchConfiguration("params_file")
    auto_initial_pose = LaunchConfiguration("auto_initial_pose")
    auto_initial_delay = LaunchConfiguration("auto_initial_delay")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_share, "launch", "fast_lio_3d_localization.launch.py")
        ),
        launch_arguments={
            "world": world,
            "gui": gui,
            "mapping_rviz": "false",
            "use_sim_time": use_sim_time,
            "map_file": pcd_map,
            "auto_initial_pose": auto_initial_pose,
            "auto_initial_delay": auto_initial_delay,
            "initial_x": initial_x,
            "initial_y": initial_y,
            "initial_yaw": initial_yaw,
        }.items(),
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time, "yaml_filename": nav_map}],
    )
    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["map_server"],
        }],
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "use_composition": "False",
            "params_file": params_file,
        }.items(),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="nav2_rviz",
        output="screen",
        arguments=["-d", os.path.join(champ_navigation_share, "rviz", "navigation.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(nav_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(go2_share, "worlds", "mid360_mapping.world"),
        ),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("show_rviz", default_value="true"),
        # Capture the public option before nested FAST-LIO launches reuse and
        # overwrite their own `show_rviz` launch configuration.
        SetLaunchConfiguration("nav_rviz", show_rviz),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("auto_initial_pose", default_value="true"),
        DeclareLaunchArgument("auto_initial_delay", default_value="10.0"),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "pcd_map",
            default_value=os.path.join(
                workspace_dir, "maps", "mid360_3d.pcd"
            ),
        ),
        DeclareLaunchArgument(
            "nav_map",
            default_value=os.path.join(
                workspace_dir, "maps", "mid360_3d_nav.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(go2_share, "config", "autonomy", "fast_lio_nav2.yaml"),
        ),
        localization,
        TimerAction(period=16.0, actions=[map_server, map_lifecycle]),
        # Start Nav2 only after the delayed automatic ICP initialization has
        # had time to create map -> odom.  Starting the lifecycle manager
        # against two disconnected TF trees intermittently leaves the
        # planner inactive, especially in fast headless simulations.
        TimerAction(period=28.0, actions=[navigation]),
        TimerAction(period=22.0, actions=[rviz]),
    ])
