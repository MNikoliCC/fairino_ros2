# fairinoplan_bringup

Minimal ROS 2 bringup for FAIRINO FR20 using:

- RoboPlan for IK, collision-aware RRT planning, and TOPP-RA time parameterization
- ros2_control `joint_trajectory_controller` for trajectory execution
- FAIRINO `FairinoHardwareInterface` for real robot communication / ServoJ
- Viser + RoboPlan OInK for interactive Cartesian servoing

## Interactive Viser servo

```bash
# FAIRINO SimMachine or robot at 192.168.58.2
ros2 launch fairinoplan_bringup fairino20_viser_servo.launch.py use_mock_hardware:=false
# mock hardware
ros2 launch fairinoplan_bringup fairino20_viser_servo.launch.py use_mock_hardware:=true
```

Open <http://localhost:8000>, press **Enable Servo** and drag the TCP gizmo.
Viser renders at 30 Hz, the gizmo publishes on change, and the C++ servo
(`fairino_cartesian_servo`) runs the SE(3) filter, Cartesian rate limiter and
RoboPlan OInK at 1 kHz into a `JointGroupPositionController`; `ros2_control`
and ServoJ run at 1 kHz. Plan mode (RRT + TOPP-RA through
`fairino20_controller`) is available from the same UI. The FAIRINO status,
recovery and speed services are provided by the hardware plugin itself
(`/fairino_robot/*`); do not start `ros2_cmd_server` alongside it.

Documentation (repository `docs/`):
[operator guide](../docs/viser_operator_guide.md),
[architecture](../docs/viser_servo_architecture.md),
[faults and recovery](../docs/fairino_fault_recovery.md),
[speed configuration](../docs/fairino_speed_configuration.md),
[benchmark](../docs/servo_benchmark.md),
[SimMachine test plan](../docs/simmachine_test_plan.md).

Test tools installed with this package:
`fairino_servo_benchmark.py`, `fairino_fault_injection_test.py` (SimMachine
only), `fairino_servo_state_machine_test.py` (mock only).

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

The controller IP is the xacro/launch argument `controller_ip`
(default `192.168.58.2`); the hardware parameters are listed in
`config/fairino20_v6_robot.ros2_control.xacro`.

The hardware loop and planned trajectory sampling run at 1 kHz. ServoJ uses
the SDK UDP command transport on robot port 20007 with `cmdT=0.001 s`. For a
real robot, use a real-time kernel, grant the controller manager real-time
scheduling permissions, and connect the host directly to the robot network.

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

`config/roboplan.yaml` overrides the URDF joint velocities with the FR20
maxima (J1–J3 120 deg/s, J4–J6 180 deg/s) and contains conservative
placeholder acceleration/jerk limits for TOPP-RA, because FAIRINO publishes no
FR20 joint accelerations. Validate them before increasing them.

The Viser servo uses its own limits (`fairino_cartesian_servo/config/fairino_cartesian_servo.yaml`):
FR20 joint velocity maxima, 540 deg/s² joint acceleration (the peak FAIRINO's
own planner used on SimMachine) and a 1 m/s Cartesian rate limit. See the
[speed configuration](../docs/fairino_speed_configuration.md) before changing
any speed.
