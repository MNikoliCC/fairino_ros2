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
# Real FAIRINO controller at 192.168.58.2
ros2 launch fairinoplan_bringup fairino20_viser_servo.launch.py use_mock_hardware:=false

# SimMachine: 2 ms ServoJ points (it executes fewer than 1000 points/s)
ros2 launch fairinoplan_bringup fairino20_viser_servo.launch.py use_mock_hardware:=false servoj_cmd_period:=0.002 servoj_max_queue:=1

# Mock hardware (no FAIRINO controller)
ros2 launch fairinoplan_bringup fairino20_viser_servo.launch.py use_mock_hardware:=true
```

Launch arguments:

| Argument | Default | Meaning |
|---|---|---|
| `use_mock_hardware` | `true` | `false` loads the FAIRINO hardware plugin |
| `controller_ip` | `192.168.58.2` | FAIRINO controller address |
| `servoj_cmd_period` | `0.001` | ServoJ cmdT [s]: time the controller takes for one point. `0.002` for SimMachine |
| `servoj_max_queue` | `8` | ServoJ FIFO limit [points]; `1` together with `servoj_cmd_period:=0.002` |
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
**FAIRINO TCP** at startup: the URDF frame `flange`, i.e. the TCP of FAIRINO
tool 0 with the same position and orientation as in the WebApp/SimMachine
(x red, y green, z blue, z out of the flange). Small axes show the measured
TCP; the gizmo shows the target. The *Status* tab lists the Viser TCP and the
FAIRINO TCP (`tl_cur_pos`, mm and rx/ry/rz in deg) and their difference
(0.01 mm on SimMachine). With a FAIRINO tool other than 0 the gizmo stays on
the flange and the Status tab says so.

The `base_link` axes (25 cm, labelled) mark the frame of every pose and
offset in the UI. Bottom left: the browser's frame rate and the longest
frame of the last second (`60 fps · max frame 20 ms`), measured in the page;
a low value means the browser/GPU is slow, it does not affect the robot.

The UI starts in dark mode. Viser's **Share** button (top right of the
panel, UI node parameter `show_share_button`, default `true`) creates a
public link through Viser's relay server (needs internet access): **whoever
has that link sees the UI and can operate the robot** (servo, Execute,
Home). Share only on a supervised cell; set `show_share_button:=false` where
that is not wanted.

A small window at the top left (*Robot*; it can be moved and minimized)
shows the state (coloured: 🔵 DISABLED, 🟢 ACTIVE, 🟡 HOLD, 🟠 FAULT,
🔴 LOCKOUT) · mode · active controller · FAIRINO connection · reason and the
plan line (what is planned or executing, its duration, the speed), the line
about the last execution (deviation from the plan), a `❌ Path: …` line when
the drawn path cannot be executed, and the *Last command* line. Messages are
there so that the working panel on the right keeps its size. Only in FAULT
or LOCKOUT the fault box and the **Recover** button appear there (section 6).

The control panel on the right:

| Part | Content |
|---|---|
| **View: ☐ Servo / ☑ Trajectory / ☐ Files** | the views, like tabs. Servo and Trajectory show their content **and switch the robot's mode** (Servo → servo mode, Trajectory → plan mode); Files does not change the mode. Viser tabs cannot do this, because the selected tab is not reported to the server. Servo is refused while a trajectory or Home is executing. At start the view follows the servo's current mode |
| **Record** ↔ **Stop & Save Recording** | shared by all views: a recording continues across servo motion and executed trajectories (one segment per motion, pauses while the robot rests); *Discard Recording* while one is running |
| **Enable/Disable Point Tracking**, *Point [mm]* | shared (one point); each view's gizmo reacts in its own way (below) |
| **Home** | shared: plans and executes the motion to the home pose in one step (section 5); from the servo view the servo is enabled again afterwards |
| *Servo* view | **Servo Hold ↔ Resume Servo**, *Gizmo axes* of the servo gizmo, drawing with the servo gizmo (below): *Drawn since the last waypoint*, **Confirm Waypoint**, *Drawing* **Undo / Discard**; **Enable Servo** appears only when the servo is not active in this view (refused, or after a fault). The servo gizmo is put on the robot TCP on every enable/resume |
| *Trajectory* view | **Path** (editor, **New Trajectory**), **Plan & Execute** (what to plan: *Whole path*, *Segments* From → To, or *To gizmo*; **Speed**; *Plan continuously*; **Plan**; **Execute** right below it; **Gizmo to Robot TCP**), **Waypoint** (the gizmo or a waypoint: pose, *Stop [s]*, offsets, table), **Segments** |
| *Files* view | **Recordings** (library, replay, Edit as Path) and **Path files** (save/load drawn paths) ([trajectory_recording_and_paths.md](trajectory_recording_and_paths.md)); loading switches to the trajectory view |
| **System** (collapsed) | tabs *Status*, *Speed*, *Audit* (below) |

Each view has its own gizmo. The **servo gizmo** is the target the servo
follows; it is synchronized with the robot TCP on every enable. The
**trajectory gizmo** draws and plans; the robot does not follow it, and it
stays where it was left when the view changes (*Gizmo to Robot TCP* or *New
Trajectory* bring it to the robot). The path, the recording trail and the measured
TCP stay visible in both views; the planned trajectory, its preview and the
path's editing handles only in the trajectory view.

| System tab | Content |
|---|---|
| **Status** | servo state, active controller, orientation task (full pose / tool axis only), fault, suggested action, joint feedback age/rate, Viser target age, tracking error, recovery attempts and result, rates and scheduling, FAIRINO connection, enabled, manual/automatic mode, main/sub code, E-stop, safety stops, collision and collision levels, safety door/plane/data, driver alarms, soft limit/abnormal stop, UDP state, ServoJ streaming and executed rate, last ServoJ code and counters, feedback age/rate, `write()` period; timing table (mean/p99/max/overruns) |
| **Speed** | global speed (manual/automatic), Apply Global Speed with confirmation, the four speed layers, the safety-speed form (SimMachine only) |
| **Audit** | newest 25 audit events of the servo and the hardware |

Buttons that are not allowed in the current state are greyed out; the
services check again and answer with a reason in *Last command*.

## 3. Servo mode

Choose **View: Servo**: the UI enables the servo (servo mode, tracking the
gizmo). **View: Trajectory** disables it again (plan mode: the trajectory
controller holds the robot). In the *Servo* folder one button toggles
**Servo Hold** (servo mode, not tracking) / **Resume Servo**.

1. Enabling from plan mode: the robot must be at rest (refused while a
   planned trajectory is still executing). The servo copies the measured joints
   into the command, resets the target to the current TCP, resets the filter
   and OInK state, switches controllers and becomes ACTIVE. The gizmo jumps to
   the current TCP pose: this is expected.
2. Drag the gizmo (arrows: translation along one axis; squares: translation
   in the XY, XZ or YZ plane of the TCP frame; rings: rotation). The robot follows
   through the SE(3) filter (τ 0.03 s), the Cartesian rate limit (2 m/s, π
   rad/s) and OInK at the FR20 joint limits. A distant gizmo target is approached
   at these speeds, not jumped to.
3. The gizmo must stay connected: while a browser is open, the UI republishes
   the gizmo pose at 30 Hz. If no target arrives for 1 s (browser closed,
   network lost), the servo goes to HOLD.

After every enable, resume or recovery the servo ignores the gizmo until it
reports a pose within 1 cm / 0.1 rad of the reset pose. The UI does this
automatically; it prevents an old gizmo position from pulling the robot.

### Point tracking and gizmo axes

**Enable Point Tracking** / **Disable Point Tracking** (one button, shared by
both views). While it is on, you move the TCP with the gizmo as usual and the
tool z axis keeps pointing at the point.

- There is one point (red sphere, shown while tracking is on), set only by
  *Point [mm]* in `base_link`; default −3000, 0, 0 mm. *Path › Orientation ›
  Look at point* uses the same point, also when tracking is off.
- Servo view: the robot follows (below). Trajectory view: the trajectory
  gizmo looks at the point (the robot does not move), Confirm stores that
  orientation and a moved waypoint (Edit) turns toward the point. Switching
  views keeps tracking on; the servo's free tool roll is requested only in
  the servo view.
- While servoing, the target turns toward the point at most 45°/s (switching
  tracking on, or typing a point far away, does not swing the robot round at
  once; measured: 63° in 1.5 s).
- The rotation about the tool axis is free: the servo tracks only the
  position and the direction of the tool axis (service `set_free_tool_roll`; the Status tab shows *Orientation task*), and OInK
  picks that rotation (on the FR20 joint 6) with the least motion and within
  the joint limits; j6 keeps its full range. (The FAIRINO Viser prototype
  computed the roll from a fixed "up" vector and made it jump by up to 180°;
  an earlier version here let the UI choose the roll, which could run j6 into
  its limit.) Measured: the point circled the tool 3 times, j6 stayed at 0.0°,
  the tool axis on the point within 0.0°.
- Switching it off: in plan mode (the robot did not turn) the target goes back
  to the robot's tool orientation; while servoing it keeps the orientation the
  robot has reached. Switching the gizmo axes never changes the orientation;
  only *Look at point* and point tracking turn it toward the point.

**Gizmo axes** (*Servo* folder for the servo gizmo, *Path* folder for the
trajectory gizmo; each its own setting): **☐ base_link / ☑ tool**. Both move and rotate; only the axes differ. `tool`: the gizmo is the
target pose (its arrows and rings are the tool axes). `base_link`: the gizmo
rests at the base axes; moving it moves the target, turning a ring turns the
target about that base axis, and after release the gizmo returns to the base
axes while small axes at the target show its orientation. The same applies to
the waypoint gizmo in the Path editor's Edit mode. While point tracking is on the
gizmo always uses `base_link` axes without rotation (the point decides) and
the choice is hidden.

## 4. Servo Hold, resume, Disable Servo

| Button | Effect |
|---|---|
| **Servo Hold** | ACTIVE → HOLD: no further target tracking; the joints decelerate at the acceleration limit (540 deg/s², at most ≈ 0.35 s from full speed) and then hold |
| **Resume Servo** (in HOLD) | resume (after the deceleration has finished): target reset to the current TCP, then ACTIVE |
| **View: Trajectory** (disables the servo) | ACTIVE → HOLD, wait for standstill, switch back to the trajectory controller (plan mode, state DISABLED). From FAULT/LOCKOUT the controller is switched but the fault state remains |

## 5. Plan mode: Plan & Execute

1. Choose **View: Trajectory**; the top-left window shows `DISABLED · plan
   mode · controller fairino20_controller`.
2. In *Plan & Execute* choose what to plan:

   | Plan | What is planned |
   |---|---|
   | **Whole path** | the drawn path from waypoint 1 to the last one |
   | **Segments** | only the waypoints *From waypoint* → *To waypoint* (e.g. 4 → 5), to tune or repeat one part without running the whole path. The part is highlighted yellow. It moves the robot exactly as within the whole path (the joint motion of the whole path from its start, cut at the part); if the path before the part cannot be followed, the part starts from the configuration nearest to the current one. Picking From ≥ To moves the other field (From 3 with To 2 → To 4) |
   | **To gizmo** | a free motion to the trajectory gizmo: IK from the measured state, RRT, shortcutting and TOPP-RA (FR20 velocity limits, 540 deg/s²). Draw mode only: in Edit the gizmo is hidden, so the choice disappears (Whole path and Segments fill the row; a chosen *To gizmo* becomes *Whole path*) |

   Path and segments start with a free approach (RRT + TOPP-RA) from where
   the robot is to their first waypoint, then follow the drawn path.
3. **Speed** (0.05–1): the execution speed of every plan (path, segments,
   gizmo, Home, replay). 1 = the planned times (fastest, or the time set per
   segment); 0.5 = every segment, stop and the approach take twice as long
   (a 10 s plan takes 20 s). The planner divides all times by it (joint
   velocities scale by the speed, accelerations by its square, so the limits
   hold) and publishes the waiting plan again at once when it changes.
4. **Plan**: the plan line in the top-left window says what was planned
   and how long it takes, e.g. `Planned: segments 4→5 · 1.45 s (approach
   0.63 s) · speed 0.50` or `Planned: home`. The planned TCP path is drawn in blue.
5. **Preview**: after every plan a translucent blue robot plays the planned
   motion once, in real time and at the chosen speed: exactly what Execute
   will do (measured: planned 13.21 s, executed 13.20 s at speed 0.5). With
   *Plan continuously* the path is planned again after every change and the
   preview repeats (1 s pause on the goal). The preview never moves the robot.
6. **Execute** (right below Plan): sends exactly the published trajectory to
   `fairino20_controller` (the FAIRINO global speed does not scale it).
   Status goes `executing` → `succeeded`; the preview disappears.

**Gizmo to Robot TCP** puts the trajectory gizmo on the robot's TCP (Draw mode only, like the gizmo).

**Home** (shared, both views) plans and executes in one step: to the home
joint pose (UI parameter `home_joints_deg`, default 0, −90, 90, −90, −90,
0 deg: tool pointing down). From the servo view it switches to plan mode
first and enables the servo again when the robot is home; the steps appear
as `Home: …` in *Last command*. It is refused in FAULT/LOCKOUT and while a
Home is running.

Plans and Execute are in the trajectory view only (plan mode). The planner checks robot
self-collisions; there is no environment model.

## 5a. Recording, replay and path editor

Short form; details, file format and measurements in
[trajectory_recording_and_paths.md](trajectory_recording_and_paths.md).

- **Record** (both views) arms the recorder, it starts with the next motion
  (servoing or Execute), pauses while the robot is at rest and continues in
  either view; **Stop & Save Recording** writes the file. In *Recordings*
  (Files view) load it, choose *Recorded timing* or *Fastest within
  limits*, **Plan Replay** (at the *Speed* of Plan & Execute), check the
  preview, **Execute**.
  **Edit as Path** turns it into waypoints.
- **Path**, *Draw* mode: **New Trajectory** starts at the robot's TCP and puts
  the trajectory gizmo there. Move the gizmo (the robot does not follow it);
  the green line shows the next segment and turns red as soon as the
  pose is out of reach; **Confirm Waypoint** fixes it. Clicking an earlier
  waypoint puts the gizmo on it; Confirm adds the segment back to it. After
  Undo/Redo/Delete Waypoint the gizmo goes to the end of the path.
- *Edit* mode (☑ Edit next to ☐ Draw, or pick a waypoint): the TCP gizmo and *Confirm Waypoint* disappear; a clicked
  waypoint shows a gizmo to move/rotate it and two small tangent handles
  (toward the previous/next waypoint); drag a handle to bend the path through
  the waypoint, **Straight** removes the curve. *Gizmo axes* switches the
  gizmo between `base_link` axes (move only) and the pose's own orientation
  (move and rotate); *Orientation › Look at point* turns the trajectory gizmo (Draw) or the
  selected waypoint (Edit) toward the red tracking point; with point
  tracking on, a moved waypoint turns toward it by itself. Waypoints are
  numbered from 1. Waypoints at the same point
  get small numbered spheres next to it; click the one you want.
  Selected segment/waypoint: yellow. Summary icon: ✅ planned, ❌ cannot be
  executed, ⏳ planning, ⚪ not planned. Select a segment to see its **fastest time** and enter
  a longer *Time [s]* if wanted. Without *Plan continuously* the path is
  only checked after a change (a segment that cannot be executed turns red,
  the reason is in the top-left window) and **Plan** plans it; with it, it
  is planned after every change. The *Waypoint* folder shows `gizmo` in Draw
  mode (its pose and a `gizmo` row in the table) or the selected waypoint in
  Edit mode (pose, **Stop [s]**: a rest at that waypoint, **Straight**,
  **Delete Waypoint**), moves the gizmo or the waypoint by an exact offset
  (*Move*, *Rotate*, *Frame*, **Apply Offset**) and lists all waypoints in
  `base_link` (⏸ = stop). Check the preview and press **Execute** in *Plan &
  Execute*. **Clear** removes the path and the plan.
- **Drawing in the servo view**: while the servo gizmo is held (mouse
  button down) its motion is recorded from the last waypoint, the robot
  follows it as usual, and a green line shows it. Time runs only while the
  gizmo is held: after a release nothing is recorded; holding it still for
  0.5 s or longer becomes a stop (*Stop [s]* of a waypoint). **Confirm
  Waypoint** turns the drawing into curve waypoints with the drawn segment
  times (as *Edit as Path*, 3 mm tolerance) and ends it with a waypoint at
  the gizmo; drawing then continues from there. Without a drawing it only
  adds a waypoint at the gizmo (teaching by driving the robot; the first one
  starts the path). If the drawing does not start at the last waypoint (e.g.
  the robot was elsewhere), a straight segment joins them; execute the path
  first to continue exactly from its end. **Discard** drops the unconfirmed
  drawing, **Undo** removes the last confirmed addition.
- After every **Execute**, the top-left window shows one line:
  largest and mean distance of the measured TCP from the planned path, the
  executed vs. planned time and the lowest ServoJ rate the controller kept
  up with; above 5 mm a ⚠️ line says how far and why (e.g. the SimMachine
  executed fewer ServoJ points than needed). The Path folder repeats the
  largest distance after a path run. The *Status* tab shows the ServoJ
  points/s the controller executes right now.

## 6. Faults and recovery

In FAULT or LOCKOUT the top-left window shows the fault box (severity,
source, main/sub/SDK code, description, policy, suggested action and recovery
attempts) and one **Recover** button; both disappear when the fault is gone:

- in FAULT it calls `/fairino_servo/recover` (controlled recovery, enables the
  robot if needed); for an unknown fault code that service refuses, and the
  button then calls `/fairino_servo/reset_fault` (operator acknowledgement, no
  RobotEnable);
- in LOCKOUT it calls `/fairino_servo/reset_fault`, which is refused while the
  lockout cause is still active (E-stop pressed, door open, status missing).

Suggested actions and the fault catalog still name the two procedures
("Recover", "Reset Fault"); both are reached through this button.

| State | Meaning | What to do |
|---|---|---|
| FAULT, policy `auto_recover` | whitelisted fault; automatic recovery runs after 3 s (if enabled) and ends in HOLD (servo mode) or DISABLED (plan mode) | check why it happened; press **Enable Servo** to continue; press **Recover** if automatic recovery is off |
| FAULT, policy `operator_reset` | needs an operator (collision, limits, IO, unknown code, servo-side fault) | remove the cause (WebApp alarm text, drag/jog joints), then **Recover** |
| LOCKOUT | safety signal, non-resettable fault, lost communication, or the same fault repeated after 2 recoveries | follow the suggested action; **Recover** (operator reset) is accepted only when no lockout signal is active; power-cycle where stated |
| RECOVERING | hardware steps running (normally < 0.3 s) | wait |

After any recovery the servo is in HOLD (servo mode) or DISABLED (plan mode)
and never moves by itself: press **Enable Servo** to continue servoing. Every step is listed in the Audit tab.

If somebody calls `/fairino_robot/stop` (or the hardware disarms ServoJ for
any other reason) while the servo is in HOLD or ACTIVE, the servo goes to
FAULT "hardware ServoJ streaming disarmed: …". **Recover** re-arms
streaming through a controlled recovery; the servo never resumes on its own.

**Recover** reports "still running" (not success) when the
recovery has not finished within 60 s; read the state and the Audit tab.

Communication lost or hardware deactivated (LOCKOUT "robot status
missing/stale"; deactivating `FairinoSystem` closes the FAIRINO connection and
the status topic): restore the connection, then in a container shell

```bash
ros2 control list_hardware_components          # FairinoSystem state
ros2 control set_hardware_component_state FairinoSystem active
```

then **Recover** and **Disable Servo** to return to plan mode.

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
