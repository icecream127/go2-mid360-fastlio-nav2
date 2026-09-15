#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /go2_ws/install/setup.bash

export LD_LIBRARY_PATH="/go2_ws/install/livox-sdk2/lib:${LD_LIBRARY_PATH:-}"
export GAZEBO_MASTER_URI="http://127.0.0.1:11345"
export GAZEBO_IP="127.0.0.1"
if [[ "${GO2_DDS_TRANSPORT:-default}" == "udp" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="/go2_ws/install/go2_config/share/go2_config/config/fastdds_udp.xml"
else
  unset FASTRTPS_DEFAULT_PROFILES_FILE
fi

# Seed an empty mounted volume from the installed navigation package.
# Never overwrite a user's maps. Leave partial sets untouched and warn rather
# than mixing maps from different mapping sessions.
map_source="$(ros2 pkg prefix --share go2_nav_bringup)/maps"
map_names=(mid360_3d.pcd mid360_3d_nav.pgm mid360_3d_nav.yaml)
mkdir -p /go2_ws/maps
existing=0
for name in "${map_names[@]}"; do
  if [[ -e "/go2_ws/maps/${name}" ]]; then
    existing=$((existing + 1))
  fi
done
if [[ "${existing}" -eq 0 ]]; then
  for name in "${map_names[@]}"; do
    cp "${map_source}/${name}" /go2_ws/maps/
  done
elif [[ "${existing}" -ne 3 ]]; then
  echo "Warning: /go2_ws/maps contains an incomplete default map set; existing files were preserved. Supply matching PCD/PGM/YAML maps or explicit map arguments." >&2
fi

exec "$@"
