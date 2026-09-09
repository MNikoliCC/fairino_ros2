#!/usr/bin/env bash
set -Eeo pipefail

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ -f /opt/roboplan_ws/install/setup.bash ]]; then
  source /opt/roboplan_ws/install/setup.bash
fi

if find /ros2_ws/src -name package.xml -print -quit | grep -q .; then
  echo '[ROS 2] Building the mounted workspace...'
  cd /ros2_ws
  colcon build \
    --symlink-install \
    --packages-skip roboplan_ros_examples roboplan_ros_franka \
    --cmake-args \
      -DBUILD_TESTING=OFF \
      -DGENERATE_PYTHON_STUBS=OFF
fi

if [[ -f /ros2_ws/install/setup.bash ]]; then
  source /ros2_ws/install/setup.bash
fi

exec "$@"
