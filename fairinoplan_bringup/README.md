# fairinoplan_bringup

Minimal ROS 2 bringup for FAIRINO FR20 using:

- RoboPlan for IK, collision-aware RRT planning, and TOPP-RA time parameterization
- ros2_control `joint_trajectory_controller` for trajectory execution
- FAIRINO `FairinoHardwareInterface` for real robot communication / ServoJ

## Architecture

```text
/joint_states
     |
     v
RoboPlan Scene
     |
     +--> SimpleIK (for pose goals)
     |
     +--> RRT
     |
     +--> TOPP-RA
     |
     v
FollowJointTrajectory
     |
     v
fairino20_controller
     |
     v
ros2_control
     |
     v
fairino_hardware/FairinoHardwareInterface
     |
     v
ServoJ
     |
     v
FAIRINO FR20
```

## Build

```bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## First test with mock hardware

```bash
ros2 launch fairinoplan_bringup fairino20_roboplan.launch.py \
  use_mock_hardware:=true
```

Check:

```bash
ros2 control list_controllers
ros2 topic echo /joint_states
```

Expected active controllers:

```text
joint_state_broadcaster
fairino20_controller
```

## Send a joint-space goal

Positions are radians:

```bash
ros2 topic pub --once /fairinoplan/joint_goal sensor_msgs/msg/JointState "{
  name: [j1, j2, j3, j4, j5, j6],
  position: [0.2, -1.0, 1.2, -0.8, -1.2, 0.0]
}"
```

This plans only. Execute separately:

```bash
ros2 service call /fairinoplan/execute std_srvs/srv/Trigger "{}"
```

Or launch with:

```bash
ros2 launch fairinoplan_bringup fairino20_roboplan.launch.py \
  use_mock_hardware:=true \
  auto_execute:=true
```

## Send a Cartesian goal

The minimal node accepts pose goals in `base_link`.

```bash
ros2 topic pub --once /fairinoplan/pose_goal geometry_msgs/msg/PoseStamped "{
  header: {frame_id: base_link},
  pose: {
    position: {x: 0.7, y: 0.0, z: 0.8},
    orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}
  }
}"
```

Then:

```bash
ros2 service call /fairinoplan/execute std_srvs/srv/Trigger "{}"
```

## Real hardware

```bash
ros2 launch fairinoplan_bringup fairino20_roboplan.launch.py \
  use_mock_hardware:=false
```

The FAIRINO hardware implementation currently hardcodes the controller IP
in its own source. Verify that it matches the real controller before launch.

## RoboPlan packages required

The Docker underlay must include at least:

- roboplan_common
- roboplan_core
- roboplan_simple_ik
- roboplan_rrt
- roboplan_toppra
- roboplan metapackage (needed by roboplan_ros_cpp dependency metadata)

Do not COLCON_IGNORE `roboplan_rrt` or `roboplan_toppra`.

## Important real-robot note

`config/roboplan.yaml` contains conservative placeholder acceleration/jerk
limits because FAIRINO's supplied MoveIt joint-limits file does not specify
real acceleration limits. Validate these values before increasing them.
