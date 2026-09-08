#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /go2_ws/install/setup.bash

exec "$@"
