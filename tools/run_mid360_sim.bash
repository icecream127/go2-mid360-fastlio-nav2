#!/usr/bin/env bash

set -e

SCRIPT_DIR="$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
REPO_DIR="$(builtin cd "${SCRIPT_DIR}/.." >/dev/null && pwd)"
WORKSPACE_DIR="$(builtin cd "${REPO_DIR}/../.." >/dev/null && pwd)"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "[ERROR] ROS 2 Humble setup was not found." >&2
    exit 1
fi

if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
    echo "[ERROR] Workspace is not built: ${WORKSPACE_DIR}/install/setup.bash is missing." >&2
    exit 1
fi

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

# Keep Gazebo communication inside WSL and avoid online model-database delays.
export GAZEBO_MASTER_URI="http://127.0.0.1:11345"
export GAZEBO_IP="127.0.0.1"
export GAZEBO_MODEL_DATABASE_URI=""

# WSL usually has no ALSA playback device. OpenAL otherwise blocks world
# initialization for about 29 seconds, longer than spawn_entity's timeout.
export ALSOFT_DRIVERS="null"
export QT_QPA_PLATFORM="xcb"

# Livox-SDK2 does not currently install an environment hook of its own.
export LD_LIBRARY_PATH="${WORKSPACE_DIR}/install/livox-sdk2/lib:${LD_LIBRARY_PATH:-}"

exec ros2 launch go2_config gazebo_mid360.launch.py "$@"
