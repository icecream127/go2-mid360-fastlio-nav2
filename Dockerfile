FROM osrf/ros:humble-desktop-full-jammy

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

WORKDIR /go2_ws/src/unitree-go2-ros2
COPY . .

# The project installer pins and patches all source dependencies, installs the
# Livox SDK inside this workspace, and builds the complete ROS 2 workspace.
RUN ./tools/setup_mid360_dependencies.bash \
    && source /opt/ros/humble/setup.bash \
    && source /go2_ws/install/setup.bash \
    && ros2 pkg prefix go2_mid360_sim \
    && ros2 pkg prefix go2_fastlio_localization \
    && ros2 pkg prefix go2_nav_bringup \
    && ros2 run go2_fastlio_localization pcd_to_nav2_map --help \
    && ros2 launch go2_nav_bringup navigation.launch.py --show-args \
    && rm -rf /var/lib/apt/lists/*

COPY docker/entrypoint.bash /ros_entrypoint_go2.sh
RUN chmod +x /ros_entrypoint_go2.sh

WORKDIR /go2_ws
ENV GAZEBO_MODEL_DATABASE_URI=""
ENV QT_QPA_PLATFORM=xcb
ENV QT_X11_NO_MITSHM=1
ENV ALSOFT_DRIVERS=null

ENTRYPOINT ["/ros_entrypoint_go2.sh"]
CMD ["ros2", "launch", "go2_nav_bringup", "navigation.launch.py", "gui:=true", "show_rviz:=true"]
