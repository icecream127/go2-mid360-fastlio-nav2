#!/usr/bin/env bash
# Ubuntu 22.04 / ROS 2 Humble, CPU real MID-360 mapping + localization only.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="$(cd -- "${REPO_DIR}/../.." && pwd)"
SRC_DIR="${WORKSPACE_DIR}/src"
JOBS="${GO2_BUILD_JOBS:-2}"

if [[ "${1:-}" == --help ]]; then
  echo 'Usage: bash tools/setup_real_mid360_dependencies.bash [--check]'
  echo 'GO2_BUILD_JOBS=1 reduces memory use; default is 2. Uses build/install/log.'
  echo '--check prints the deployment scope without installing or changing files.'
  exit 0
fi
if [[ $# -gt 1 || ( $# -eq 1 && "$1" != --check ) ]]; then
  echo 'Unknown argument. Use --help.' >&2; exit 2
fi
[[ "$JOBS" =~ ^[1-9][0-9]*$ ]] || { echo 'GO2_BUILD_JOBS must be a positive integer.' >&2; exit 2; }
[[ "$(dirname -- "$REPO_DIR")" == "$SRC_DIR" ]] || {
  echo 'Place the repository directly under WORKSPACE/src before running.' >&2; exit 1;
}
if [[ "${1:-}" == --check ]]; then
  echo "Workspace: $WORKSPACE_DIR"
  echo 'Dependencies: Livox-SDK2, livox_ros_driver2, Xju FAST_LIO, CPU fast_gicp'
  echo 'ROS packages: livox_ros_driver2 fast_lio fast_gicp go2_fastlio_localization go2_real_localization'
  echo 'Excluded: Gazebo, Livox simulation plugin, CHAMP, Nav2'
  echo "Build jobs: $JOBS (sequential packages); output: $WORKSPACE_DIR/install/setup.bash"
  echo 'Existing maps, simulation packages and user changes are not deleted.'
  exit 0
fi
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 ]] || {
  echo 'This script requires Ubuntu 22.04 and ROS 2 Humble.' >&2; exit 1;
}
[[ -f /opt/ros/humble/setup.bash ]] || {
  echo 'Install ROS 2 Humble first; this script does not install ROS itself.' >&2; exit 1;
}
if [[ "$EUID" -eq 0 ]]; then ROOT_CMD=(); else ROOT_CMD=(sudo); fi
set +u
source /opt/ros/humble/setup.bash
set -u

retry() {
  local attempt
  for attempt in 1 2 3; do
    if "$@"; then return 0; fi
    if [[ "$attempt" != 3 ]]; then sleep 5; fi
  done
  echo "Failed after 3 attempts: $*" >&2; return 1
}

# Never reset, delete, move or check out an existing dependency. A mismatched
# checkout needs a deliberate user decision; matching versions keep local edits.
checkout_dependency() {
  local name="$1" url="$2" revision="$3" recursive="$4"
  local target="$SRC_DIR/$name" staging attempt
  if [[ -e "$target" ]]; then
    if [[ ! -d "$target/.git" ]] || [[ "$(git -C "$target" rev-parse HEAD)" != "$revision" ]]; then
      echo "Existing dependency is incomplete or not the pinned version: $target" >&2
      echo 'Keep it as a backup outside src, then rerun; nothing has been overwritten.' >&2
      return 1
    fi
    echo "Reusing $target at $revision (local changes preserved)."
  else
    mkdir -p "$WORKSPACE_DIR/dependency_downloads"
    staging="$(mktemp -d "$WORKSPACE_DIR/dependency_downloads/${name}.XXXXXX")"
    git -C "$staging" init -q
    git -C "$staging" remote add origin "$url"
    retry git -C "$staging" -c http.version=HTTP/1.1 -c http.lowSpeedLimit=1000 -c http.lowSpeedTime=30 \
      fetch --depth 1 origin "$revision"
    git -C "$staging" checkout --detach "$revision"
    if [[ "$recursive" == yes ]]; then
      retry git -C "$staging" -c http.version=HTTP/1.1 submodule update --init --recursive
    fi
    # Destination did not exist. Failed downloads stay outside src for inspection.
    mv -- "$staging" "$target"
  fi
  if [[ "$recursive" == yes ]]; then
    retry git -C "$target" -c http.version=HTTP/1.1 submodule update --init --recursive
  fi
}

apply_once() {
  local directory="$1" patch="$REPO_DIR/patches/$2"
  if git -C "$directory" apply --reverse --check "$patch" 2>/dev/null; then
    echo "Already applied: $2"
  elif git -C "$directory" apply --check "$patch"; then
    git -C "$directory" apply "$patch"
  else
    echo "Patch conflicts with existing changes: $patch. Resolve manually; no reset performed." >&2
    return 1
  fi
}

"${ROOT_CMD[@]}" apt update
"${ROOT_CMD[@]}" apt install -y git build-essential cmake python3-dev \
  python3-colcon-common-extensions python3-rosdep python3-numpy python3-scipy \
  libeigen3-dev libpcl-dev libapr1-dev ros-humble-ament-cmake-auto \
  ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-rviz2 \
  ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins

mkdir -p "$SRC_DIR"
checkout_dependency Livox-SDK2 https://github.com/Livox-SDK/Livox-SDK2.git f5d9375f84efe2b15bc0a052d3e18482ed13adf4 yes
checkout_dependency livox_ros_driver2 https://github.com/Livox-SDK/livox_ros_driver2.git 13eb05e4e6dd7a765b934d0c5fd6236676a57b49 yes
checkout_dependency FAST_LIO https://github.com/XjuHurricaneQuadVision/FAST_LIO.git 440a8e3e909023b7cb084e99b0c5070e3baebe86 yes
checkout_dependency fast_gicp https://github.com/koide3/fast_gicp.git 0e7ec1441c99f7be453db2ea216d5de029387417 no

DRIVER="$SRC_DIR/livox_ros_driver2"
if [[ ! -e "$DRIVER/package.xml" ]]; then
  cp "$DRIVER/package_ROS2.xml" "$DRIVER/package.xml"
elif ! cmp -s "$DRIVER/package_ROS2.xml" "$DRIVER/package.xml"; then
  echo 'Existing driver package.xml differs from package_ROS2.xml; inspect it before continuing.' >&2
  exit 1
fi
# Same Humble generated-header compatibility fix as the full installer.
sed -i \
  -e '/${LIVOX_INTERFACES_INCLUDE_DIRECTORIES}   # for custom msgs/d' \
  -e '/${LIVOX_INTERFACE_TARGET}   # for custom msgs/d' "$DRIVER/CMakeLists.txt"
if ! grep -q 'Ensure generated Livox message headers are available' "$DRIVER/CMakeLists.txt"; then
  sed -i '/# include file direcotry/i\
  # Ensure generated Livox message headers are available to the driver target.\
  add_dependencies(${PROJECT_NAME} ${LIVOX_INTERFACES})\
  target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp")\
' "$DRIVER/CMakeLists.txt"
fi
apply_once "$DRIVER" livox_driver2_monotonic_clock.patch
apply_once "$SRC_DIR/FAST_LIO" fast_lio_handheld.patch
apply_once "$SRC_DIR/FAST_LIO" fast_lio_timestamp_guard.patch

# Explicit roots prevent rosdep/colcon from crawling simulation or old copies.
PACKAGE_PATHS=("$DRIVER" "$SRC_DIR/FAST_LIO" "$SRC_DIR/fast_gicp"
  "$REPO_DIR/go2_fastlio_localization" "$REPO_DIR/go2_real_localization")
PACKAGES=(livox_ros_driver2 fast_lio fast_gicp go2_fastlio_localization go2_real_localization)
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  "${ROOT_CMD[@]}" rosdep init
fi
retry rosdep update
# The shared go2_fastlio_localization package retains OPTIONAL simulation launch
# files. Only that known simulation dependency is skipped in the real profile.
# ament_python is colcon's Python build type; the tool is installed explicitly above.
rosdep install --from-paths "${PACKAGE_PATHS[@]}" --ignore-src -y \
  --skip-keys 'go2_mid360_sim ament_python'

cd "$WORKSPACE_DIR"
SDK_PREFIX="$WORKSPACE_DIR/install/livox-sdk2"
cmake -S "$SRC_DIR/Livox-SDK2" -B "$WORKSPACE_DIR/build/livox_sdk2_bootstrap" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$SDK_PREFIX"
cmake --build "$WORKSPACE_DIR/build/livox_sdk2_bootstrap" --parallel "$JOBS"
cmake --install "$WORKSPACE_DIR/build/livox_sdk2_bootstrap"

export CMAKE_BUILD_PARALLEL_LEVEL="$JOBS"
export MAKEFLAGS="-j$JOBS"
colcon build --base-paths "${PACKAGE_PATHS[@]}" --packages-select "${PACKAGES[@]}" \
  --executor sequential --symlink-install --cmake-force-configure \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DBUILD_VGICP_CUDA=OFF -DBUILD_apps=OFF -DBUILD_test=OFF -DBUILD_PYTHON_BINDINGS=OFF \
  -DDISTRO_ROS=humble \
  -DLIVOX_LIDAR_SDK_LIBRARY="$SDK_PREFIX/lib/liblivox_lidar_sdk_shared.so" \
  -DLIVOX_LIDAR_SDK_INCLUDE_DIR="$SDK_PREFIX/include"

set +u
source "$WORKSPACE_DIR/install/setup.bash"
set -u
for package in "${PACKAGES[@]}"; do ros2 pkg prefix "$package"; done
echo 'Real MID-360 dependencies and workspace build completed.'
echo "Next: source $WORKSPACE_DIR/install/setup.bash"
echo 'Configure the NIC and mid360_real.json, copy your real PCD, then launch go2_real_localization bringup.launch.py.'
