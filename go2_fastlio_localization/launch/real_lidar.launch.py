"""Real MID-360 acquisition only: no Gazebo, SLAM, Nav2 or motor control."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("go2_fastlio_localization")
    return LaunchDescription([
        DeclareLaunchArgument("user_config_path",
                              default_value=os.path.join(share, "config", "mid360_real.json")),
        DeclareLaunchArgument("xfer_format", default_value="1",
                              description="1: Livox CustomMsg for FAST-LIO; 0: PointCloud2 for RViz"),
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="mid360_real_driver",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "user_config_path": LaunchConfiguration("user_config_path"),
                "xfer_format": ParameterValue(LaunchConfiguration("xfer_format"), value_type=int),
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": 10.0,
                "output_data_type": 0,
                "frame_id": "livox_frame",
                "cmdline_input_bd_code": "livox0000000001",
            }],
        ),
    ])
