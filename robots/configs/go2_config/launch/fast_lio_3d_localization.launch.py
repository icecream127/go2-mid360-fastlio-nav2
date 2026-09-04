import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    go2_share = get_package_share_directory("go2_config")

    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    show_rviz = LaunchConfiguration("show_rviz")
    mapping_rviz = LaunchConfiguration("mapping_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map_file")

    fast_lio_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_share, "launch", "fast_lio_3d_mapping.launch.py")
        ),
        launch_arguments={
            "world": world,
            "gui": gui,
            "show_rviz": mapping_rviz,
            "use_sim_time": use_sim_time,
            # Nav2's local odometry is derived from FAST-LIO below.  Do not
            # start the legacy Gazebo ground-truth odometry helper.
            "ground_truth_odom": "false",
        }.items(),
    )

    fast_lio_odom_bridge = Node(
        package="go2_config",
        executable="fast_lio_odom_bridge.py",
        name="fast_lio_odom_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    localizer = Node(
        package="go2_config",
        executable="pcd_icp_localizer.py",
        name="pcd_icp_localizer",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "map_file": map_file,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(
                    go2_share, "worlds", "mid360_mapping.world"
                ),
            ),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("show_rviz", default_value="true"),
            DeclareLaunchArgument("mapping_rviz", default_value=show_rviz),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "map_file",
                default_value=os.path.join(
                    os.path.expanduser("~"), "go2_ws", "maps", "mid360_3d.pcd"
                ),
            ),
            fast_lio_stack,
            fast_lio_odom_bridge,
            TimerAction(period=14.0, actions=[localizer]),
        ]
    )
