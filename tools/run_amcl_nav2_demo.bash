#!/usr/bin/env bash

set -e

SCRIPT_DIR="$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
REPO_DIR="$(builtin cd "${SCRIPT_DIR}/.." >/dev/null && pwd)"
WORKSPACE_DIR="$(builtin cd "${REPO_DIR}/../.." >/dev/null && pwd)"
MAP_FILE="${WORKSPACE_DIR}/maps/mid360_mapping_initial.yaml"

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

if [[ ! -f "${MAP_FILE}" ]]; then
    echo "[ERROR] Saved map not found: ${MAP_FILE}" >&2
    exit 1
fi

export GAZEBO_MASTER_URI="http://127.0.0.1:11345"
export GAZEBO_IP="127.0.0.1"
export GAZEBO_MODEL_DATABASE_URI=""
export ALSOFT_DRIVERS="null"
export QT_QPA_PLATFORM="xcb"
export LD_LIBRARY_PATH="${WORKSPACE_DIR}/install/livox-sdk2/lib:${LD_LIBRARY_PATH:-}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${WORKSPACE_DIR}/install/go2_config/share/go2_config/config/fastdds_udp.xml"

exec ros2 launch go2_config amcl_nav2_demo.launch.py map:="${MAP_FILE}" "$@"
