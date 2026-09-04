import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression


def generate_launch_description():

    robot_name = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    headless = LaunchConfiguration("headless")
    paused = LaunchConfiguration("paused")
    lite = LaunchConfiguration("lite")
    contact_sensor_enabled = LaunchConfiguration("contact_sensor")
    ros_control_file = LaunchConfiguration("ros_control_file")
    world_init_x = LaunchConfiguration("world_init_x")
    world_init_y = LaunchConfiguration("world_init_y")
    world_init_z = LaunchConfiguration("world_init_z")
    world_init_heading = LaunchConfiguration("world_init_heading")
    gazebo_world = LaunchConfiguration("world")
    gz_pkg_share = launch_ros.substitutions.FindPackageShare(package="champ_gazebo").find(
        "champ_gazebo"
    )

    declare_robot_name = DeclareLaunchArgument("robot_name", default_value="champ")
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="True")
    declare_gui = DeclareLaunchArgument("gui", default_value="True")
    declare_headless = DeclareLaunchArgument("headless", default_value="False")
    declare_paused = DeclareLaunchArgument("paused", default_value="False")
    declare_lite = DeclareLaunchArgument("lite", default_value="False")
    declare_contact_sensor = DeclareLaunchArgument(
        "contact_sensor",
        default_value="False",
        description="Run the high-CPU CHAMP foot contact helper",
    )
    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file",
        default_value=os.path.join(gz_pkg_share, "config/ros_control.yaml"),
    )
    declare_gazebo_world = DeclareLaunchArgument(
        "world", default_value=os.path.join(gz_pkg_share, "worlds/default.world")
    )
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.6")
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", default_value="0.6"
    )

    pkg_share = launch_ros.substitutions.FindPackageShare(package="champ_description").find("champ_description")
    default_model_path = os.path.join(pkg_share, "urdf/champ.urdf.xacro")

    declare_description_path = DeclareLaunchArgument(name="description_path", default_value=default_model_path, description="Absolute path to robot urdf file")

    config_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="champ_config"
    ).find("champ_config")
    
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")
    gazebo_config = os.path.join(launch_ros.substitutions.FindPackageShare(
        package="champ_gazebo"
    ).find("champ_gazebo"), "config/gazebo.yaml")
    launch_dir = os.path.join(pkg_share, "launch")
    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=[
            "gzserver",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            gazebo_world,
            '--ros-args',
            '--params-file',
            gazebo_config
        ],
        cwd=[launch_dir],
        output="screen",
    )


    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(gui),
        cmd=["gzclient"],
        cwd=[launch_dir],
        output="screen",
    )
    start_gazebo_spawner_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity",
            robot_name,
            "-topic",
            "/robot_description",
            "-robot_namespace",
            "",
            "-x",
            world_init_x,
            "-y",
            world_init_y,
            "-z",
            world_init_z,
            "-R",
            "0",
            "-P",
            "0",
            "-Y",
            world_init_heading,
        ],
    )

    # The factory service becomes available before Gazebo has fully finished
    # loading a world.  Delay the entity insertion so a previous gzserver
    # teardown or a slow world load cannot leave a launch with no robot.
    delayed_gazebo_spawner = TimerAction(
        period=5.0,
        actions=[start_gazebo_spawner_cmd],
    )

    # TODO as for right now, running contact sensor results in RTF being reduced by factor of 2x.
    # So it needs to be fixed before using that. Unsure what it does actually because even without it
    # Champ seems to be all right
    contact_sensor = Node(
        package="champ_gazebo",
        executable="contact_sensor",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")},links_config],
        condition=IfCondition(contact_sensor_enabled),
        # prefix=['xterm -e gdb -ex run --args'],
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", LaunchConfiguration("description_path")]),
            value_type=str,
        )
    }


    # The controller manager is created by gazebo_ros2_control only after the
    # robot has been spawned.  The controller_manager spawner waits for that
    # service, unlike an immediate `ros2 control load_controller` command.
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="go2_controller_spawner",
        output="screen",
        arguments=[
            "joint_states_controller",
            "joint_group_effort_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    # joint_group_position_controller
    return LaunchDescription(
        [
            declare_robot_name,
            declare_use_sim_time,
            declare_gui,
            declare_headless,
            declare_paused,
            declare_lite,
            declare_contact_sensor,
            declare_ros_control_file,
            declare_gazebo_world,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_heading,
            declare_description_path,
            start_gazebo_server_cmd,
            start_gazebo_client_cmd,
            delayed_gazebo_spawner,
            controller_spawner,
            contact_sensor
        ]
    )
