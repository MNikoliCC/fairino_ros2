#!/usr/bin/env bash
set -Eeo pipefail

source /opt/ros/humble/setup.bash

if find /ros2_ws/src -name package.xml -print -quit | grep -q .; then
  echo '[ROS 2] Building the mounted workspace...'
  cd /ros2_ws
  colcon build \
    --symlink-install \
    --cmake-args -DBUILD_TESTING=OFF
fi

if [[ -f /ros2_ws/install/setup.bash ]]; then
  source /ros2_ws/install/setup.bash
fi

exec "$@"
