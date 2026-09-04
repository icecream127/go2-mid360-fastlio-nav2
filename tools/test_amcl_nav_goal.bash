#!/usr/bin/env bash
set -u

source /opt/ros/humble/setup.bash
source /home/ice/go2_ws/install/setup.bash

export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ice/go2_ws/install/go2_config/share/go2_config/config/fastdds_udp.xml

echo "=== odom before ==="
timeout 8 ros2 topic echo /odom --once --field pose.pose.position || true

echo "=== send Nav2 goal: map (0.5, 0.0) ==="
timeout 60 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'
goal_status=$?

echo "=== odom after ==="
timeout 8 ros2 topic echo /odom --once --field pose.pose.position || true

exit "$goal_status"
