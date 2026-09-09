#!/usr/bin/env bash
# Reproduce the pinned external dependencies for this project.
set -euo pipefail

SCRIPT_DIR="$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
REPO_DIR="$(builtin cd "${SCRIPT_DIR}/.." >/dev/null && pwd)"
WORKSPACE_DIR="$(builtin cd "${REPO_DIR}/../.." >/dev/null && pwd)"
SRC_DIR="${WORKSPACE_DIR}/src"

if [[ "${EUID}" -eq 0 ]]; then
  ROOT_CMD=()
elif command -v sudo >/dev/null 2>&1; then
  ROOT_CMD=(sudo)
else
  echo "Run as root or install sudo before running this script." >&2
  exit 1
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "Install ROS 2 Humble first: /opt/ros/humble/setup.bash is missing." >&2
  exit 1
fi
set +u
source /opt/ros/humble/setup.bash
set -u

clone_at_commit() {
  local name="$1" url="$2" commit="$3"
  local destination="${SRC_DIR}/${name}"
  if [[ -d "${destination}/.git" ]] &&
     [[ -n "$(git -C "${destination}" status --porcelain --untracked-files=all)" ]]; then
    # Preserve the entire checkout (including untracked files) outside src,
    # so colcon cannot discover duplicate packages in the backup.
    local backup
    mkdir -p "${WORKSPACE_DIR}/dependency_backups"
    backup="$(mktemp -d "${WORKSPACE_DIR}/dependency_backups/${name}.XXXXXX")"
    mv "${destination}" "${backup}/${name}"
    echo "Preserved modified dependency: ${backup}/${name}"
  fi
  if [[ ! -d "${destination}/.git" ]]; then
    git clone --recursive "${url}" "${destination}"
  fi
  git -C "${destination}" fetch --tags origin
  git -C "${destination}" checkout --detach "${commit}"
  git -C "${destination}" submodule update --init --recursive
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

retry_command() {
  local maximum_attempts="$1"
  local delay_seconds="$2"
  shift 2

  local attempt=1
  until "$@"; do
    if (( attempt >= maximum_attempts )); then
      echo "Command failed after ${attempt} attempts: $*" >&2
      return 1
    fi
    echo "Command failed (attempt ${attempt}/${maximum_attempts}); retrying in ${delay_seconds}s: $*" >&2
    sleep "${delay_seconds}"
    ((attempt += 1))
  done
}

"${ROOT_CMD[@]}" apt update
"${ROOT_CMD[@]}" apt install -y \
  git build-essential cmake python3-colcon-common-extensions \
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
if ! grep -q 'Ensure generated Livox message headers are available' \
  "${SRC_DIR}/livox_ros_driver2/CMakeLists.txt"; then
  sed -i '/# include file direcotry/i\
  # Ensure generated Livox message headers are available to the driver target.\
  add_dependencies(${PROJECT_NAME} ${LIVOX_INTERFACES})\
  target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp")\
' "${SRC_DIR}/livox_ros_driver2/CMakeLists.txt"
fi
apply_patch_once "${SRC_DIR}/FAST_LIO_ROS2" \
  "${REPO_DIR}/patches/fast_lio_ros2.patch"

"${ROOT_CMD[@]}" rosdep init 2>/dev/null || true
retry_command 3 5 rosdep update
cd "${WORKSPACE_DIR}"
rosdep install --from-paths src --ignore-src -r -y
# Build the SDK first into this workspace.  The driver otherwise finds an
# unrelated /usr/local SDK or races the SDK build on a fresh machine.
SDK_PREFIX="${WORKSPACE_DIR}/install/livox-sdk2"
cmake -S "${SRC_DIR}/Livox-SDK2" -B "${WORKSPACE_DIR}/build/livox_sdk2_bootstrap" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${SDK_PREFIX}"
cmake --build "${WORKSPACE_DIR}/build/livox_sdk2_bootstrap" --parallel 2
cmake --install "${WORKSPACE_DIR}/build/livox_sdk2_bootstrap"
# External sources above are patched in place.  Force CMake to reconfigure so
# a previous interrupted build cannot retain stale include paths.
colcon build --symlink-install --cmake-force-configure --packages-skip livox_sdk2 \
  --cmake-args -DDISTRO_ROS=humble \
  -DLIVOX_LIDAR_SDK_LIBRARY="${SDK_PREFIX}/lib/liblivox_lidar_sdk_shared.so" \
  -DLIVOX_LIDAR_SDK_INCLUDE_DIR="${SDK_PREFIX}/include"

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
