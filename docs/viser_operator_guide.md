# Viser operator guide

How to start the FAIRINO FR20 servo stack, use the Viser interface, and react
to its states and faults. Background: [architecture](viser_servo_architecture.md),
[faults and recovery](fairino_fault_recovery.md), [speed configuration](fairino_speed_configuration.md).

> Before moving a real robot: E-stop within reach, nobody inside the robot
> envelope, the workspace clear (there is **no environment collision
> checking**), payload/tool configured in the WebApp, and the stack tested on
> SimMachine first.

## 1. Start the stack

```bash
cd ~/colcon_ws/src/fairino_ros2/docker
./simmachine.sh up                 # SimMachine + ROS 2 container (first run builds the image)
docker compose exec ros2 bash      # sourced ROS 2 shell (ROS_DOMAIN_ID=42)
```

Inside the container:

```bash
# FAIRINO hardware: SimMachine or a real controller at 192.168.58.2
ros2 launch fairinoplan_bringup fairino20_viser_servo.launch.py use_mock_hardware:=false

# Mock hardware (no FAIRINO controller)
ros2 launch fairinoplan_bringup fairino20_viser_servo.launch.py use_mock_hardware:=true
```

Launch arguments:

| Argument | Default | Meaning |
|---|---|---|
| `use_mock_hardware` | `true` | `false` loads the FAIRINO hardware plugin |
| `controller_ip` | `192.168.58.2` | FAIRINO controller address |
| `auto_recovery` | `true` | automatic recovery of whitelisted faults (never ends in motion: HOLD in servo mode, DISABLED in plan mode) |
| `solver_frequency` | `1000.0` | OInK rate; `250.0` enables 1 kHz interpolation |
| `self_collision_pairs` | `0` | self-collision barrier (phase 2), up to 5 pairs |
| `allow_safety_config_writes` | `false` | SAFETY-RELATED: allow `SetVelReducePara` (SimMachine only) |
| `controller_manager_prefix` | `chrt -f 40` | real-time priority for `ros2_control_node`; use `''` if not permitted |
| `viser_port` | `8000` | Viser port |
| `start_planner` | `true` | RRT + TOPP-RA plan/execute node |

The launch starts robot_state_publisher, ros2_control_node with the hardware
plugin (which also provides `/fairino_robot/*`), joint_state_broadcaster, the
trajectory controller (active), the servo controller (inactive), the planner,
the Cartesian servo and the Viser UI. RViz is not started.

Do not start `ros2_cmd_server` at the same time: the hardware plugin is the
only FAIRINO SDK client.

Startup takes about 5 s for the SDK connection. The hardware log shows
`FAIRINO realtime state: 129 fields every 4 ms` and
`ServoJ UDP streaming armed`. If the controller is faulted or the robot is
disabled at startup, the hardware still starts with streaming disarmed and the
servo shows FAULT with the reason.

## 2. Open Viser

Open <http://localhost:8000> (the port is published by Docker Compose). The
robot shows the measured joint positions (30 Hz). The gizmo is placed on the
flange (`wrist3_link`) at startup.

The panel has a summary line (state · mode · active controller · FAIRINO
connection · reason) and four tabs:

| Tab | Content |
|---|---|
| **Control** | mode selector, servo buttons, Plan/Execute, fault box with Recover/Reset Fault, last command result |
| **Status** | servo state, active controller, fault, suggested action, joint feedback age/rate, Viser target age, tracking error, recovery attempts and result, rates and scheduling, FAIRINO connection, enabled, manual/automatic mode, main/sub code, E-stop, safety stops, collision and collision levels, safety door/plane/data, driver alarms, soft limit/abnormal stop, UDP state, ServoJ streaming, last ServoJ code and counters, feedback age/rate, `write()` period; timing table (mean/p99/max/overruns) |
| **Speed** | global speed (manual/automatic), Apply Global Speed with confirmation, the four speed layers, the safety-speed form (SimMachine only) |
| **Audit** | newest 25 audit events of the servo and the hardware |

Buttons that are not allowed in the current state are greyed out; the
services check again and answer with a reason in *Last command*.

## 3. Servo mode

1. **Controller mode → Servo mode → Apply mode** switches to the servo
   controller and holds (HOLD). Alternatively press **Enable Servo** directly
   from plan mode.
2. **Enable Servo**: the robot must be at rest (Enable is refused while a
   planned trajectory is still executing). The servo copies the measured joints
   into the command, resets the target to the current TCP, resets the filter
   and OInK state, switches controllers and becomes ACTIVE. The gizmo jumps to
   the current flange pose: this is expected.
3. Drag the gizmo (arrows: translation; rings: rotation). The robot follows
   through the SE(3) filter (τ 0.05 s), the Cartesian rate limit (1 m/s, π
   rad/s) and OInK at the FR20 joint limits. A distant gizmo target is approached
   at these speeds, not jumped to.
4. The gizmo must stay connected: while a browser is open, the UI republishes
   the gizmo pose at 30 Hz. If no target arrives for 1 s (browser closed,
   network lost), the servo goes to HOLD.

After every enable, resume or recovery the servo ignores the gizmo until it
reports a pose within 1 cm / 0.1 rad of the reset pose. The UI does this
automatically; it prevents an old gizmo position from pulling the robot.

## 4. Hold, resume, disable

| Button | Effect |
|---|---|
| **Hold** | ACTIVE → HOLD: no further target tracking; the joints decelerate at the acceleration limit (540 deg/s², at most ≈ 0.35 s from full speed) and then hold |
| **Enable Servo** in HOLD | resume (after the deceleration has finished): target reset to the current TCP, then ACTIVE |
| **Reset Target to Current TCP** | moves the gizmo and the servo reference to the current TCP (useful after dragging the gizmo far away). While ACTIVE and moving, the robot decelerates at the acceleration limit towards that point instead of stopping abruptly; refused while synchronizing or recovering |
| **Disable Servo** | ACTIVE → HOLD, wait for standstill, switch back to the trajectory controller (plan mode, DISABLED). From FAULT/LOCKOUT the controller is switched but the fault state remains |

## 5. Plan mode (RRT + TOPP-RA)

1. Make sure the summary shows `DISABLED · mode plan · controller fairino20_controller`.
2. Move the gizmo to the goal pose.
3. **Plan**: the planner solves IK from the measured state, plans with RRT,
   shortcuts and time-parameterizes with TOPP-RA (FR20 velocity limits). The
   planned TCP path is drawn in blue; the planner status shows points,
   duration and planning time.
4. **Execute**: sends the trajectory to `fairino20_controller`. Status goes
   `executing` → `succeeded`.

Plan and Execute are disabled in servo mode. The planner checks robot
self-collisions; there is no environment model.

## 6. Faults and recovery

The fault box shows severity, source, main/sub/SDK code, description, policy,
suggested action and recovery attempts.

| State | Meaning | What to do |
|---|---|---|
| FAULT, policy `auto_recover` | whitelisted fault; automatic recovery runs after 3 s (if enabled) and ends in HOLD (servo mode) or DISABLED (plan mode) | check why it happened; press **Enable Servo** to continue; press **Recover** if automatic recovery is off |
| FAULT, policy `operator_reset` | needs an operator (collision, limits, IO, unknown code, servo-side fault) | remove the cause (WebApp alarm text, drag/jog joints), then **Recover** (known faults, enables the robot) or **Reset Fault** |
| LOCKOUT | safety signal, non-resettable fault, lost communication, or the same fault repeated after 2 recoveries | follow the suggested action; **Reset Fault** is accepted only when no lockout signal is active; power-cycle where stated |
| RECOVERING | hardware steps running (normally < 0.3 s) | wait |

After any recovery the servo is in HOLD (servo mode) or DISABLED (plan mode)
and never moves by itself. Every step is listed in the Audit tab.

If somebody calls `/fairino_robot/stop` (or the hardware disarms ServoJ for
any other reason) while the servo is in HOLD or ACTIVE, the servo goes to
FAULT "hardware ServoJ streaming disarmed: …". **Reset Fault** re-arms
streaming through a controlled recovery; the servo never resumes on its own.

**Recover** and **Reset Fault** report "still running" (not success) when the
recovery has not finished within 60 s; read the state and the Audit tab.

Communication lost or hardware deactivated (LOCKOUT "robot status
missing/stale"; deactivating `FairinoSystem` closes the FAIRINO connection and
the status topic): restore the connection, then in a container shell

```bash
ros2 control list_hardware_components          # FairinoSystem state
ros2 control set_hardware_component_state FairinoSystem active
```

then **Reset Fault** and **Disable Servo** to return to plan mode.

## 7. Speed panel

- **FAIRINO global speed** shows manual and automatic percentages. To change
  it: enter the value, a reason, tick the confirmation and press **Apply Global
  Speed** (`SetSpeedInstant`). The result shows old and new values; the change
  is in the audit log. Global speed does not change the servo (ServoJ) speed.
- **Speed layers** lists the software/OInK limits, the global speed, the
  safety speed (not readable through the SDK) and the physical maximum.
- **Safety speed (SAFETY-RELATED)** writes `SetVelReducePara`. It is refused
  unless the stack was launched with `allow_safety_config_writes:=true`
  (SimMachine only), needs the confirmation box, and the password field is
  not masked. Verify the result in the WebApp.

## 8. Reading the status

| Field | Normal | Attention |
|---|---|---|
| ServoJ streaming | `ARMED — streaming` | `DISARMED` with a reason (fault, guard, stop); `WAIT_SYNC` only briefly (≈ 20 ms) after activation or a recovery |
| Last ServoJ return code | 0, errors 0 | non-zero: SDK code, see the catalog |
| Joint feedback (controller) | age < 5 ms @ 250 Hz | age > 100 ms: communication problem |
| Joint feedback (ROS) | @ 250 Hz | rejected messages are counted in diagnostics |
| Viser target age | < 40 ms while a browser is open | > 1 s → HOLD |
| Tracking error | a few mrad at rest, tens of mrad at high speed | 150 mrad → FAULT (robot cannot follow) |
| write() period | mean 1.000 ms, p99 ≈ 1.01–1.3 ms | overruns > 0: host overloaded |
| Timing table (`cycle`, `oink`) | p99 well below 1 ms | overruns growing: lower `solver_frequency` or `self_collision_pairs` |
| `servoj_flow_skips` (ROS topic) | ≈ 0 on a real controller; 7–60 % on SimMachine | grows on a real robot: controller slower than 1 kHz |
| `servoj_flow_rebaselines` (ROS topic) | a few per activation/recovery (counter reset) | growing during motion: the controller stalled ≥ 0.5 s; check the following delay |

## 9. Stop

Press **Disable Servo**, then stop the launch with `Ctrl+C`. The hardware
plugin ends servo mode (`ServoMoveEnd`, `StopMotion`) and closes the SDK
connection on shutdown. Stop the containers with `./simmachine.sh down`.
