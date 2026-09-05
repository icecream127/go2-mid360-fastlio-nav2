#!/usr/bin/env bash
# Reproduce the pinned external dependencies for this project.
set -euo pipefail

SCRIPT_DIR="$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
REPO_DIR="$(builtin cd "${SCRIPT_DIR}/.." >/dev/null && pwd)"
WORKSPACE_DIR="$(builtin cd "${REPO_DIR}/../.." >/dev/null && pwd)"
SRC_DIR="${WORKSPACE_DIR}/src"

clone_at_commit() {
  local name="$1" url="$2" commit="$3"
  local destination="${SRC_DIR}/${name}"
  if [[ ! -d "${destination}/.git" ]]; then
    git clone --recursive "${url}" "${destination}"
  fi
  git -C "${destination}" fetch --tags origin
  git -C "${destination}" checkout --detach "${commit}"
}

apply_patch_once() {
  local directory="$1" patch_file="$2"
  if git -C "${directory}" apply --check "${patch_file}"; then
    git -C "${directory}" apply "${patch_file}"
  elif git -C "${directory}" apply --reverse --check "${patch_file}"; then
    echo "Patch already present: $(basename "${patch_file}")"
  else
    echo "Patch cannot be applied: ${patch_file}" >&2
    exit 1
  fi
}

sudo apt update
sudo apt install -y \
  python3-rosdep python3-numpy python3-scipy \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-xacro ros-humble-robot-localization \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-pointcloud-to-laserscan ros-humble-navigation2 \
  ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-teleop-twist-keyboard

mkdir -p "${SRC_DIR}"
clone_at_commit Livox-SDK2 https://github.com/Livox-SDK/Livox-SDK2.git \
  f5d9375f84efe2b15bc0a052d3e18482ed13adf4
clone_at_commit livox_ros_driver2 https://github.com/Livox-SDK/livox_ros_driver2.git \
  13eb05e4e6dd7a765b934d0c5fd6236676a57b49
clone_at_commit livox_laser_simulation_ros2 https://github.com/LCAS/livox_laser_simulation_ros2.git \
  dbac363e867676efbe7394df32fc3a90f4dfa35e
clone_at_commit FAST_LIO_ROS2 https://github.com/Ericsii/FAST_LIO_ROS2.git \
  2fffc570a25d0df172720bac034fbdb6a13d2162

# Livox keeps the ROS 2 manifest under this nonstandard filename.  Colcon
# only recognizes package.xml, so create the expected local copy.
cp "${SRC_DIR}/livox_ros_driver2/package_ROS2.xml" \
  "${SRC_DIR}/livox_ros_driver2/package.xml"

apply_patch_once "${SRC_DIR}/livox_laser_simulation_ros2" \
  "${REPO_DIR}/patches/livox_laser_simulation_ros2.patch"
# Livox driver2's Humble branch links the generated message target directly,
# but still references two legacy variables which are unset on newer CMake.
sed -i \
  -e '/${LIVOX_INTERFACES_INCLUDE_DIRECTORIES}   # for custom msgs/d' \
  -e '/${LIVOX_INTERFACE_TARGET}   # for custom msgs/d' \
  "${SRC_DIR}/livox_ros_driver2/CMakeLists.txt"
if ! grep -q 'CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp' \
  "${SRC_DIR}/livox_ros_driver2/CMakeLists.txt"; then
  sed -i '/target_link_libraries(${PROJECT_NAME} "${cpp_typesupport_target}")/a\
    add_dependencies(${PROJECT_NAME} ${LIVOX_INTERFACES})\
    target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp")' \
    "${SRC_DIR}/livox_ros_driver2/CMakeLists.txt"
fi
apply_patch_once "${SRC_DIR}/FAST_LIO_ROS2" \
  "${REPO_DIR}/patches/fast_lio_ros2.patch"

sudo rosdep init 2>/dev/null || true
rosdep update
cd "${WORKSPACE_DIR}"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

# Make the committed example map available at the same workspace-level path
# used by the default launch arguments.  Do not overwrite a user's own map.
mkdir -p "${WORKSPACE_DIR}/maps"
cp -n "${REPO_DIR}/robots/configs/go2_config/maps/mid360_3d.pcd" \
  "${WORKSPACE_DIR}/maps/"
cp -n "${REPO_DIR}/robots/configs/go2_config/maps/mid360_3d_nav.pgm" \
  "${WORKSPACE_DIR}/maps/"
cp -n "${REPO_DIR}/robots/configs/go2_config/maps/mid360_3d_nav.yaml" \
  "${WORKSPACE_DIR}/maps/"

echo "Dependencies and workspace build completed."
