# Viser Cartesian servo: architecture

This document describes how the FAIRINO FR20 Cartesian servo stack is built:
components, threads, rates, ROS interfaces, controller switching, coordinate
frames and the protections between the solver and the robot. Operation is in
[viser_operator_guide.md](viser_operator_guide.md), faults in
[fairino_fault_recovery.md](fairino_fault_recovery.md), speed settings in
[fairino_speed_configuration.md](fairino_speed_configuration.md), timing in
[servo_benchmark.md](servo_benchmark.md) and the executed tests in
[simmachine_test_plan.md](simmachine_test_plan.md).

There is **no collision checking against the environment** anywhere in the
servo path. The RoboPlan self-collision barrier is a separate, optional phase
(`self_collision_pairs`, default 0) with its own benchmark.

## 1. Components and packages

| Package | Contents |
|---|---|
| `fairino_hardware_v3_9_9` | ros2_control plugin `fairino_hardware/FairinoHardwareInterface`: the only FAIRINO SDK connection, ServoJ streaming with protections, internal `/fairino_robot` node (status, events, recover, stop, speed services) |
| `fairino_msgs` | `RobotStatus`, `ServoStatus`, `TimingStats`, `AuditEvent`, `RecoverRobot`, `SetGlobalSpeed`, `SetSafetySpeed` (plus the vendor messages) |
| `fairino_cartesian_servo` | C++ node `fairino_cartesian_servo` (state machine, SE3 filter, Cartesian rate limiter, RoboPlan OInK, fault catalog, recovery), `fairino_fault_catalog` (prints the catalog as Markdown), unit tests, tool to generate the self-collision model |
| `fairinoplan_bringup` | launch `fairino20_viser_servo.launch.py`, controller/xacro/RoboPlan configuration, `fairino_viser_ui.py`, `fairino_plan_execute.py` (RRT + TOPP-RA), benchmark and test scripts |

```text
Browser ── Viser WebSocket ── fairino_viser_ui.py (Python)
                                 ├─ robot from /joint_states at 30 Hz (Pinocchio ViserVisualizer, same URDF as RoboPlan)
                                 ├─ 6-DoF transform-controls gizmo on wrist3_link
                                 ├─ Enable/Hold/Disable/Reset Target/Plan/Execute/Recover/Reset Fault/mode
                                 └─ status, speed, safety-speed and audit panels
        /fairino_servo/target_pose (PoseStamped, base_link)        /fairino_servo/* services
                                 ▼
fairino_cartesian_servo (C++)
  ├─ state machine DISABLED/SYNCING/ACTIVE/HOLD/RECOVERING/FAULT/LOCKOUT
  ├─ joint-state validation (mapped by name, missing/NaN/Inf rejected, limits checked)
  ├─ target re-acquisition after every synchronization
  ├─ roboplan::SE3LowPassFilter → Cartesian linear/angular rate limiter
  ├─ RoboPlan OInK FrameTask base_link→wrist3_link
  │     PositionLimit + VelocityLimit + AccelerationLimit [+ SelfCollisionBarrier]
  ├─ joint-result validation, joint interpolation (only if solver < 1 kHz)
  ├─ fault catalog, recovery orchestration, audit events, diagnostics
  └─ timing ring buffers (no per-cycle logging)
        /fairino_servo_controller/commands (Float64MultiArray j1..j6, 1 kHz)
                                 ▼
fairino_servo_controller (position_controllers/JointGroupPositionController)   servo mode
fairino20_controller     (joint_trajectory_controller/JointTrajectoryController) plan mode
        six position command interfaces, owned by exactly one controller
                                 ▼
FairinoHardwareInterface (ros2_control, 1 kHz)
  ├─ read():  GetRobotRealTimeState (CNDE, 4 ms), freshness, fault/safety latch
  ├─ write(): discontinuity guard → flow control → per-point step limit → ServoJ UDP (cmdT 1 ms)
  └─ /fairino_robot node: status 50 Hz, events, recover, stop, set_global_speed, set_safety_speed
                                 ▼
FAIRINO SimMachine 3.9.9 (192.168.58.2) / FR20 controller

fairino_plan_execute.py (plan mode): SimpleIk + RRT + shortcutting + TOPP-RA → FollowJointTrajectory
```

### 1.1 Single owner of the FAIRINO SDK

The first integration ran the vendor `ros2_cmd_server` next to the ros2_control
plugin. Both opened an SDK connection; on SimMachine the second client blocked
before "connect success", no `/nonrt_state_data` was published and service
calls hung. Status, recovery and speed operations therefore live in the
hardware plugin, which is the only SDK client. **Do not run `ros2_cmd_server`
together with the hardware plugin.** XML-RPC calls from the plugin's service
threads are serialized by `rpc_mutex_`; the real-time thread never takes it.

### 1.2 FAIRINO realtime state (CNDE)

SDK 3.9.9 receives the controller state through CNDE (robot port 20004).
Verified on SimMachine 3.9.9 (controller V3.9.39-LX):

1. The controller default is fields 3..75 every 8 ms. Fields 76..133 (safety
   door/plane, abnormal stop, UDP command state, driver alarms, speed scales,
   collision levels, ServoJ counter) are **not transmitted** and read as 0.
2. `SetRobotRealtimeStateConfig()` on a live connection makes the SDK parse the
   new layout while the controller still sends the old one: values such as
   `udpCmdState=-1883245793` and `safetyDataState=135` appear.
3. SDK 3.9.9 does not fill `frame_head`/`frame_cnt`.

The plugin therefore configures fields 3..133 (without the two string fields)
at 4 ms **before** `RPC()`, checks the configuration with
`GetRobotRealtimeStateConfig()` after connecting and refuses to start
otherwise; frame freshness uses the controller time `robotTime` (1 ms
resolution). 4 ms is the SDK minimum, so FAIRINO joint feedback is 250 Hz and
`joint_state_broadcaster` publishes at 250 Hz as well.

### 1.3 ServoJ streaming protections (hardware plugin)

| State | `write()` | Entered by |
|---|---|---|
| `ARMED` | ServoJ for the current command | `WAIT_SYNC`, once the command has matched the measured position (≤ 2 mrad) **and stayed unchanged for 20 cycles** |
| `WAIT_SYNC` | nothing sent; command interface primed with the measured position | hardware activation (`ServoMoveStart` succeeded), successful `/fairino_robot/recover` |
| `DISARMED` | nothing sent; command interface primed with the measured position | controller fault, safety signal, robot disabled, control-box or driver-communication error, ServoJ error, discontinuity guard, non-finite command, feedback loss, recovery, stop, deactivation |

Activation never arms directly: a position controller that stayed active
across a hardware re-activation, or a trajectory that is still playing, may
hold a command that is not the measured position. The 20 static cycles make
sure a moving stream is never armed, even when it passes through the measured
position.

In `ARMED`, every cycle applies, in this order:

1. **Discontinuity guard** on the controller command stream: a command change
   may move a joint at most `jump_guard_factor (2) × v_max × T`. T is the write
   period (1 ms), except for a stream that was **moving** (its previous two
   changes ≤ 3 ms apart): then T is the time since the previous change, capped
   at `jump_guard_max_gap` (50 ms). Topic-fed controllers keep the last command
   when a message is delayed (10–20 ms seen under heavy CPU load) and then jump
   to the newest one; that is real motion and passes. A stream that was holding
   (HOLD, a trajectory controller at rest, just armed) gets no gap allowance, so
   a stale command after a switch, recovery or re-activation trips. A trip
   disarms streaming.
2. **Flow control**: the controller executes ServoJ points from a FIFO.
   SimMachine consumed fewer than the 1000 points/s sent, so without
   control the queue and the motion delay grew without bound (2–4 s observed).
   `write()` estimates the queue as sent − executed points (`servoJCmdNum`,
   modulo 10000) and skips a cycle when it exceeds `servoj_max_queue` (8).
   The executed count can never exceed the sent count: a larger counter step
   (the counter is reset by `ServoMoveStart`, possibly after the baseline frame
   was taken) re-baselines to an empty queue instead of being counted. Only
   after **0.5 s without executed progress** (lost UDP points would otherwise
   dead-lock the stream) is the estimate reset. Both rules matter: counting the
   phantom points after a hardware re-activation, and re-baselining after
   40 ms of normal SimMachine stalls, each let the real FIFO grow unseen to a
   permanent 0.3–0.6 s following delay (measured, see
   [servo_benchmark.md](servo_benchmark.md)).
   The opposite error, an estimate that is too high because the controller
   dropped or merged points without counting them, starves the controller
   (SimMachine settled at exactly one point per 4 ms frame). A **drift probe**
   detects it: after 100 ms in which the estimate read "full" while at most two
   points per frame were executed, `write()` pauses for three feedback frames.
   If frames 2–3 show no executed point, the FIFO is empty and the estimate is
   corrected (`servoj_flow_drift_corrections`); if points are still executed,
   the controller is really slow and nothing changes. At most one probe per
   0.5 s (`servoj_flow_probes`).
3. **Per-point step limit**: one point may not move a joint further than
   `point_velocity_factor (1) × v_max × cmdT`. After a skipped cycle or a
   controller stall the robot lags briefly and catches up instead of receiving
   one large step (which produced `main=1 sub=23` speed faults on SimMachine).
4. `ServoJ(cmd, cmdT = 0.001, UDP)`; any non-zero return code disarms.

`read()` disarms on any controller main/sub code, E-stop, safety stop, safety
door/plane, safety data, driver alarm/error, driver communication error,
control-box error, soft-limit error, power-cycle request or a disabled robot
(`abnomalStop` is informational: it stays 1 after a successful reset). It
keeps the last good position on a failed or stale read, disarms after 100 ms
and returns `hardware_interface::return_type::ERROR` after 500 ms. The former
`read()` statement `hardware_interface::return_type::ERROR;` without `return`
is gone. Every return code is published on `/fairino_robot/status` for the
servo.

### 1.4 Hardware lifecycle and the SDK instance

- `read()`/`write()` run only while they get the lifecycle mutex with
  `try_lock` and the SDK instance exists; activation, deactivation and the
  error cleanup hold it while they replace or close the connection.
- `on_error()` is called from the real-time thread. It disarms immediately and
  moves the slow part (stopping the status node, `ServoMoveEnd`/`StopMotion`/
  `CloseRPC` on a possibly dead connection) to a cleanup thread that the next
  activation joins.
- Deactivation and the error path increment a stop generation; a recovery that
  is running when it changes aborts and never re-arms.
- `on_deactivate()` closes the SDK connection and stops `/fairino_robot`. The
  servo sees the status disappear and goes to LOCKOUT ("robot state unknown");
  after `set_hardware_component_state FairinoSystem active`, Reset Fault.
- **SDK 3.9.9 defect**: `CloseRPC()` only closes the sockets; the detached CNDE
  receive thread keeps using its `FRRobot`. Destroying the instance after
  `CloseRPC()` crashed `ros2_control_node` (SIGSEGV in
  `FRCNDEClient::RecvRobotStateThread`) on the second deactivate/activate
  cycle against SimMachine. Closed instances are therefore kept until the
  process exits (one small object per deactivation or error); 6 consecutive
  cycles then ran without a crash.

## 2. Thread model

| Process | Thread | Scheduling | Work |
|---|---|---|---|
| `ros2_control_node` (launched with `chrt -f 40`) | controller manager loop, 1 kHz | SCHED_FIFO 50 | `read()` → controllers → `write()` |
| | executor + DDS threads | SCHED_FIFO 40 (inherited) | controller subscriptions (servo commands), services |
| | `/fairino_robot` executor (3 threads) | SCHED_FIFO 40 (inherited) | status 50 Hz; `recover`, `set_*`, `stop` |
| | FAIRINO SDK threads | inherited | CNDE receive, XML-RPC, UDP |
| `fairino_cartesian_servo` | servo loop (`clock_nanosleep`, 1 kHz) | SCHED_FIFO 80 | sync requests, watchdogs, filter, rate limiter, OInK, interpolation, publish |
| | operation worker | normal | enable/disable/mode/recovery (blocking service calls) |
| | executor (6 threads) | normal | subscriptions, services, 100 Hz supervisor, status 10 Hz, diagnostics 1 Hz |
| `fairino_viser_ui.py` | rclpy executor (4) + Viser server | normal | display 30 Hz, gizmo events, heartbeat 30 Hz, panels 2 Hz |
| `fairino_plan_execute.py` | rclpy | normal | planning on demand |

Rules:

- The servo loop never calls services and never waits; long sequences run in
  the worker. Synchronization is requested through atomics and executed at the
  start of the next loop cycle (it owns the RoboPlan scene and solver).
- Every cycle is timed into lock-free ring buffers; percentiles are computed
  once per second by the supervisor. Nothing is logged per cycle.
- `write()` takes no lock held by a service; the status snapshot and the
  lifecycle mutex are taken with `try_lock` (a cycle is skipped instead of
  waiting).
- Viser callbacks only publish a pose or call a service asynchronously.

Docker Compose grants `SYS_NICE`, `rtprio 99` and `memlock` and sets
`ROS_DOMAIN_ID=42`. The domain is required: with `ipc: host`, another ROS 2
system on the same host (a foreign `/viser_visualizer` node appeared on domain
0) is discovered through shared memory and could publish into this stack.

## 3. Rates

| Stage | Rate | Notes |
|---|---|---|
| Viser robot rendering | 30 Hz | `display_frequency` |
| Gizmo input | event based | each drag event publishes a target |
| Gizmo heartbeat | 30 Hz while a browser is connected | closing the browser lets the target watchdog trigger HOLD |
| SE3 filter, Cartesian rate limiter, OInK | `solver_frequency`, 1 kHz | decided by the benchmark |
| Joint interpolation | 1 kHz | only used when `solver_frequency` < 1 kHz (250 Hz option) |
| Joint command output | 1 kHz | `/fairino_servo_controller/commands` |
| ros2_control | 1 kHz | `controller_manager.update_rate` |
| ServoJ | ≤ 1 kHz, UDP, `cmdT = 0.001 s` | flow control matches the controller's consumption |
| FAIRINO feedback (CNDE) | 250 Hz | SDK minimum period 4 ms |
| `/joint_states` | 250 Hz | broadcaster `update_rate` = feedback rate |
| `/fairino_robot/status` | 50 Hz | |
| `/fairino_servo/status` | 10 Hz, timing window 1 s | |

The Viser target does not need to arrive at 1 kHz: between events the filter
and OInK keep tracking the last valid target.

## 4. ROS interfaces

### Topics

| Topic | Type | Publisher → subscriber |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | broadcaster → servo, UI, planner. Order is j1,j2,j4,j5,j3,j6: always mapped by name |
| `/fairino_servo/target_pose` | `geometry_msgs/PoseStamped` in `base_link` | UI (or benchmark) → servo |
| `/fairino_servo/reset_target_pose` | `PoseStamped`, latched | servo → UI (gizmo jumps to the synchronized TCP) |
| `/fairino_servo_controller/commands` | `std_msgs/Float64MultiArray` j1..j6 [rad] | servo → JointGroupPositionController |
| `/fairino_servo/status` | `fairino_msgs/ServoStatus` | servo → UI |
| `/fairino_servo/diagnostics` | `diagnostic_msgs/DiagnosticArray` | servo |
| `/fairino_servo/events` | `fairino_msgs/AuditEvent`, latched 200 | servo → UI audit tab |
| `/fairino_robot/status` | `fairino_msgs/RobotStatus` | hardware plugin → servo, UI |
| `/fairino_robot/events` | `fairino_msgs/AuditEvent`, latched 200 | hardware plugin → UI |
| `/fairinoplan/pose_goal`, `/fairinoplan/joint_goal` | `PoseStamped` / `JointState` | UI → planner |
| `/fairinoplan/status` | `std_msgs/String` (JSON), latched | planner → UI |
| `/fairinoplan/planned_path` | `trajectory_msgs/JointTrajectory`, latched | planner → UI (drawn in Viser) |

### Services

| Service | Type | Effect |
|---|---|---|
| `/fairino_servo/enable` | `std_srvs/Trigger` | DISABLED → SYNCING → ACTIVE, or HOLD → ACTIVE (target reset) |
| `/fairino_servo/hold` | `Trigger` | ACTIVE → HOLD |
| `/fairino_servo/disable` | `Trigger` | plan mode (trajectory controller); FAULT/LOCKOUT are kept |
| `/fairino_servo/set_mode` | `std_srvs/SetBool` | true: servo mode in HOLD; false: plan mode |
| `/fairino_servo/recover` | `Trigger` | operator recovery R1 (incl. RobotEnable) for known recoverable faults |
| `/fairino_servo/reset_fault` | `Trigger` | operator acknowledgement R2 (no RobotEnable), also from LOCKOUT once the cause is gone |
| `/fairino_servo/reset_target` | `Trigger` | reference reset to the current TCP |
| `/fairino_servo/reset_timing`, `/fairino_servo/timing_report` | `Trigger` | benchmark statistics (JSON in `message`) |
| `/fairino_robot/recover` | `fairino_msgs/RecoverRobot` | hardware recovery steps |
| `/fairino_robot/stop` | `Trigger` | disarm, `ServoMoveEnd(1)`, `StopMotion()` |
| `/fairino_robot/set_global_speed` | `fairino_msgs/SetGlobalSpeed` | `SetSpeed`/`SetSpeedInstant`, confirmation, read-back, audit |
| `/fairino_robot/set_safety_speed` | `fairino_msgs/SetSafetySpeed` | `SetVelReducePara`, refused unless `allow_safety_config_writes` |
| `/fairinoplan/execute` | `Trigger` | execute the last plan on `fairino20_controller` |

## 5. Controller switching

`ros2_controllers.yaml` defines both position controllers for the same joints;
the launch file spawns `joint_state_broadcaster`, then `fairino20_controller`
(active), then `fairino_servo_controller --inactive`. Every switch is STRICT and
contains one activation and one deactivation, so both controllers never own
the interfaces at the same time. `fairino20_controller` sets
`set_last_command_interface_value_as_state_on_activation: false`: with the
Humble default it held the stale command at the moment of a FAIRINO fault
(0.032 rad away on SimMachine) after a restart, and the hardware refused to arm.

Enable from plan mode:

1. The robot must be at rest: two joint samples 50 ms apart agree within
   0.1 mrad and the FAIRINO joint speeds are below 0.5 deg/s (a trajectory that
   is still playing is refused). Then `DISABLED → SYNCING`; the hardware must
   report no fault and streaming `ARMED`.
2. At the next loop cycle, from a joint sample at most 20 ms old: `q_command = q_measured`,
   `target = FK(q_measured)` (base_link → wrist3_link), `filter.reset(target)`,
   rate-limiter state = target, previous OInK velocity = 0,
   `AccelerationLimit.reset()`. The reset pose is published to the UI.
3. The servo publishes `q_command` (= measured) at 1 kHz.
4. STRICT switch: activate `fairino_servo_controller`, deactivate
   `fairino20_controller`.
5. `SYNCING → ACTIVE` (or HOLD for `set_mode`).

After every synchronization (enable, resume, recovery, Reset Target) the servo
accepts a gizmo target only once one arrives **within 1 cm and 0.1 rad of the
reset pose**, i.e. after the UI moved its gizmo there. A stale gizmo pose can
never pull the robot towards an old position.

Disable: ACTIVE → HOLD, wait for standstill (≤ 1 s, joint speeds < 0.5 deg/s),
STRICT switch back; the trajectory controller holds the measured position.

**Leaving ACTIVE** (HOLD, FAULT, LOCKOUT, Disable) does not freeze the command
within one cycle. The servo loop continues the last commanded joint velocity
and ramps it linearly to zero so that the joint with the longest stop
decelerates at its acceleration limit (540 deg/s²) and the others keep the
same direction. Measured on mock hardware: 1.383 rad/s at HOLD → 147 ms ramp,
maximum deceleration 9.42 rad/s² (= 540 deg/s²). A full synchronization ends
the ramp (`q_command = q_measured`); resuming from HOLD waits until the ramp
has finished.

**Reset Target while ACTIVE** keeps the joint velocity and the acceleration
limiter state: OInK decelerates towards the new target (the current TCP) at
the acceleration limit instead of stopping the reference. Reset Target is
refused while SYNCING or RECOVERING, and synchronization requests are
serialized (the worker and the service never share a request slot).

**Hardware streaming supervision**: in ACTIVE and HOLD the servo requires
streaming `ARMED`; anything else (an external `/fairino_robot/stop`, a manual
`/fairino_robot/recover`, a guard trip) is a FAULT, so motion never resumes by
itself when the hardware re-arms. In DISABLED, a disarm also counts as a fault,
except the transient "recovery in progress" of a hardware recovery (it ends
ARMED or "recovery failed"). The servo's own recovery accepts only a status
received after the hardware recovery returned before it reports success.

Controller ownership is polled with `list_controllers`; every switch
increments a generation counter and answers requested before the switch are
discarded.

## 6. Coordinate frames and units

| Item | Frame / unit |
|---|---|
| Viser world | URDF root = `base_link` (robot drawn at the origin) |
| Gizmo | world pose → `base_link` through FK of `base_link` (identity here, kept general) |
| `/fairino_servo/target_pose` | `base_link`; other frames are rejected |
| OInK FrameTask | `base_link` → `wrist3_link` (flange, no tool offset) |
| Joint commands/states in ROS | radians, j1..j6 |
| ServoJ and CNDE joint values | degrees (converted in the plugin) |
| FAIRINO global speed in CNDE | fraction 0..1 (shown ×100 as percent) |
| FAIRINO joint speeds | deg/s |

FAIRINO tool/workpiece frames (`tl_cur_pos` in mm/RPY) are not used; the servo
works on the URDF kinematics only.

## 7. Limits applied by the servo

| Limit | Value | Source |
|---|---|---|
| Joint velocity (OInK `VelocityLimit`) | J1–J3 120 deg/s, J4–J6 180 deg/s (× `velocity_scale`, min with URDF) | FAIRINO FR20 specification |
| Joint acceleration (OInK `AccelerationLimit`, then exact clamp) | 540 deg/s² | peak the controller itself planned for MoveJ at 100 % on SimMachine (541.7 deg/s²). ProxQP meets the bounds only to `eps_abs` = 1e-6 rad, which is ~1 rad/s² at 1 kHz (10.3 rad/s² measured); the result is clamped to the exact acceleration and velocity box before validation |
| Joint position (OInK `PositionLimit`, gain 0.5) | URDF limits | FAIRINO URDF |
| Cartesian rate limiter | 1.0 m/s, π rad/s | servo parameters |
| SE3 filter | τ = 0.05 s | servo parameter |
| Result validation | finite, inside limits, per-step ≤ v_max·dt | servo |
| Tracking watchdog | \|q_command − q_measured\| ≤ 0.15 rad | servo parameter |
| Braking ramp when leaving ACTIVE | last joint velocity → 0 at 540 deg/s² | servo |

These are software limits on the generated command, not FAIRINO safety limits.

## 8. Self-collision barrier (phase 2)

`self_collision_pairs > 0` adds a RoboPlan `SelfCollisionBarrier` limited to
the robot's own link pairs (5 pairs remain after the SRDF). The vendor meshes
(5k–22k triangles per link) took 7–19 ms per distance evaluation, and the
`forearm–wrist2` housings are always 17 mm apart. The barrier therefore uses
`config/fairino20_v6_servo_collision.urdf`: identical kinematics, collision
meshes replaced by convex hulls (generated by
`fairino_cartesian_servo/tools/generate_servo_collision_model.py`; 0
non-conservative cases in 500 random configurations), converted to
`coal::Convex` objects before the OInK solver is built. Parameters: `d_min`
5 mm (below the 17 mm structural clearance), gain 20 (approach speed ≤
20·h/(1+h) m/s), safe-displacement gain 0 (RoboPlan weights that term by
1/‖J_h‖², and ‖J_h‖ ≈ 0 for the rotationally symmetric housings, which froze
every joint). Results are in [servo_benchmark.md](servo_benchmark.md).
