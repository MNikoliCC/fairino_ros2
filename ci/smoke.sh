#!/usr/bin/env bash
set -Eeo pipefail
repo=/ros2_ws/src/fairino_repo
ros2 pkg prefix fairino_hardware
ros2 pkg prefix fairino20_v6_moveit2_config
ros2 pkg prefix roboplan_ros_cpp
python3 -c 'import roboplan.core, roboplan.simple_ik, roboplan_ros.cpp'
# Explicit mock hardware: this check never selects the real controller backend.
ros2 launch fairino20_v6_moveit2_config demo.launch.py \
  use_mock_hardware:=true use_rviz:=false > /artifacts/launch.log 2>&1 &
launch_pid=$!
trap 'kill -INT "$launch_pid" 2>/dev/null || true; wait "$launch_pid" 2>/dev/null || true' EXIT
python3 "$repo/ci/collect.py"
kill -0 "$launch_pid"
