# Trajectory recording, replay and path editor

The Viser UI can record any motion of the robot, replay it, turn it into
editable waypoints and plan new Cartesian paths drawn with the TCP gizmo.
Every motion still goes through the same **plan → preview → Execute** flow as
a normal plan: nothing moves the robot before the operator presses Execute in
plan mode. There is no environment collision checking (self-collision, joint
limits and reachability are checked).

The design follows the FAIRINO Viser prototype (`MNikoliCC/fairino`:
`trajectory_io.py`, `main.py`, `viser_two_point_line.py`) and adapts it to
this stack:

| Prototype | Here | Why |
|---|---|---|
| replay sends recorded TCP poses with `ServoCart` and the original timestamps | replay of the recorded **joints** through the planner: approach (RRT) + recorded motion, preview, Execute | no IK branch switches, same joint limits as every plan, nothing moves without preview |
| MoveJ 20 % to the start without preview | approach planned and previewed | same rules as every motion |
| recorded timing executed as recorded | recorded timing kept where it respects the joint limits, stretched only where it does not; or fastest timing within the limits | a measured motion can exceed the limits |
| 5 ms XML-RPC polling | `/joint_states` (250 Hz), stored at 125 Hz | no extra load on the controller |
| whole JSON rewritten after every sample | autosave at 2 Hz, file written on Stop (atomic replace) | performance, no half-written files |
| one `runtime_trajectory.json` | one file per recording (name + time), library in the UI; `runtime_trajectory.json` stays as "latest" | recordings are kept |
| absolute paths | `trajectory_dir` parameter / bind mount | works in Docker and on other computers |
| path drawing tool: positions only, Bezier tangents, no saving, not executable | same drawing flow (green line from the last confirmed waypoint to the gizmo, confirm) starting at the robot TCP; waypoints with full TCP poses; the prototype's Bezier model (symmetric tangent handle at a waypoint, drawn as a cubic Bezier); time per segment (fastest shown, custom allowed); planned automatically while drawing; undo/redo, save/load; executed | the drawn path can be run |
| recordings cannot be edited | **Edit as Path**: recording simplified to waypoints (Ramer–Douglas–Peucker), then edited | editable recordings |

The file format stays compatible with the prototype (same keys; the Blender
replay reads `samples[*].joint_positions` and `fbx_path`).

## 1. Recording (*Record* button, *Recordings* in the Files view)

| Control | Effect |
|---|---|
| **Record** ↔ **Stop & Save Recording** (top of the panel, all views) | arms the recorder; recording starts with the next motion of the robot (servoing or Execute) and continues across the views |
| **Stop & Save** | ends the recording and writes `recordings/<name>_<YYYYmmdd_HHMMSS>.json` and `recordings/runtime_trajectory.json` |
| **Discard** | drops the current recording |
| Library: **Load** / **Unload** / **Refresh** | Load shows the recording's TCP trail (orange, a point every 2 mm of TCP travel) and makes it the one to replay or edit; Unload removes both; **Stop & Save** saves without loading. The autosave of the running recording (`runtime_trajectory.json`) is not listed |
| Replay: *Timing*, **Plan Replay** | plans approach + recorded motion at the *Speed* of Plan & Execute and switches to the trajectory view (plan line `Planned: replay <name>`); check the preview, then **Execute** |
| Edit as path: *Simplify tolerance*, **Edit as Path** | converts the loaded recording into curve waypoints in the *Path* editor (details below); each segment keeps its recorded duration |

How it records (`RecordingPanel` in `fairino_viser_trajectories.py`):

- source: `/joint_states` (mapped by name), sampled at `record_rate_hz`
  (125 Hz); the TCP pose is computed with the URDF frame `flange` (= TCP of
  FAIRINO tool 0) and stored in the WebApp convention (mm, rx/ry/rz deg);
- a motion is a joint speed above 0.5 deg/s; 0.25 s before the motion are
  included (pre-roll) and the segment ends 0.5 s after the robot stopped, so
  every segment starts and ends at rest;
- pauses between segments are cut from the time axis (as in the prototype);
  each segment carries the mode (`servo` or `planning`) it was recorded in;
- autosave of `runtime_trajectory.json` at 2 Hz while recording.

Sample: `{"t": s, "pose": [x, y, z mm, rx, ry, rz deg], "joint_positions": [rad × 6]}`.
Extra file keys: `frame` (`flange`), `base_frame`, `pose_units`, `joint_units`,
`fairino_tool_number`, `name`, `source`.

### Replay

*Recorded timing*: the measured samples are first cleaned
(`smooth_recording`): repeated (stale) samples are removed and a
least-squares cubic B-spline with knots every 40 ms is fitted per joint.
`/joint_states` is stamped when the broadcaster publishes while the FAIRINO
feedback frame behind it can be up to 4 ms older; differentiating the raw
samples gave apparent accelerations above 100 rad/s² for a motion executed at
9.4 rad/s². Then `fit_recorded_timing` keeps the recorded timing wherever the
joint velocity and acceleration limits hold and stretches time only around the
places where they do not (cosine-tapered window of ±0.12 s). On SimMachine the
measured motion contains real short jerks (the simulator catching up after a
stall: 125 rad/s² measured), which a uniform slowdown would have answered
with ×5.2; locally it is ×1.12–1.19. The *Speed* of Plan & Execute plays it
back slower (all times divided by it).

*Fastest within limits*: same geometry, new time-optimal timing within the
joint limits; pauses inside the recording disappear.

If the robot is not at the first recorded configuration (> 2 mrad), an
approach is planned with RRT + TOPP-RA and prepended.

### Edit as Path: recording → waypoints

1. Ramer–Douglas–Peucker picks waypoints (position within *Simplify
   tolerance*, default 3 mm, orientation within 2°).
2. Every inner waypoint gets one symmetric tangent along the bisector of
   its two chords, at most a third of the shorter chord, so uneven spacing
   cannot make loops. Where the motion reverses (> 90°) the tangent is zero:
   the robot stops there instead of overshooting the turning point.
3. Every segment is compared with the recorded samples it replaces, both
   ways (samples to curve, curve to the recorded polyline); a segment off by
   more than the tolerance is split at the worst sample, until all segments
   are within it.

Measured on an 83.5 s recording (23 m, servo teaching): the earlier
Catmull-Rom tangents gave 178 waypoints and up to 55 mm between path and
recording; now 232 waypoints and at most 2.6 mm (2.8 s conversion). The
executed robot motion stayed within 2.6 mm (mean 0.33 mm) of the recording.

## 2. Path editor (trajectory view)

A path is a list of waypoints (full TCP poses: frame `flange` in
`base_link`) and one segment between each pair of consecutive waypoints.

The editor has two modes, shown as two check boxes in one row
(**☑ Draw | ☐ Edit**; exactly one is ticked, *Draw* at start):

- **Draw** — the TCP gizmo is visible; move it and press **Confirm
  Waypoint**. Clicking an earlier waypoint sphere puts the TCP gizmo on that
  waypoint; **Confirm Waypoint** then adds the segment back to it (to close a
  loop or return to a point). Confirm without **New Trajectory** starts the path at
  the robot TCP and leaves the gizmo on the confirmed pose.
- **Edit** — the TCP gizmo is hidden (it comes back in *Draw*, and it is always
  shown while the servo is on, since the robot follows it). Clicking a
  waypoint sphere (or picking it in *Waypoint*) selects it and shows a gizmo
  to move/rotate that waypoint plus two small tangent handles toward the
  previous and the next waypoint (only those that exist).

| Control | Effect |
|---|---|
| **New Trajectory** (top of the folder) | starts a path at the robot's current TCP (waypoint 1), puts the TCP gizmo there and switches to *Draw* |
| green line (Draw) | the next segment: from the last waypoint to the TCP gizmo; checked every 0.3 s while the gizmo moves and turns **red** when that pose cannot be reached from the last waypoint, before Confirm |
| summary | two lines (the controls below never move): ✅ planned and executable · ❌ cannot be executed (the reason is in the top-left window) · ⏳ planning · ⚪ not planned yet; then waypoints, segments, curves (if any), length and (when planned) the path time incl. stops |
| **Confirm Waypoint** | adds the TCP gizmo pose as the next waypoint (shown in Draw only) |
| Edit: **Undo** / **Redo** | edit history (100 steps) |
| TCP gizmo rule | after **Undo**, **Redo** and **Delete Waypoint** the gizmo goes to the last waypoint (the end of the path), where drawing continues; after **Load** too, except in servo mode (a loaded path can end far from the robot). In servo mode the robot follows the gizmo |
| **Clear Trajectory** (below New Trajectory) | removes the path **and the current plan** (planned path line, preview, plan in the planner); Undo brings the path back |
| *Gizmo axes* **☐ base_link / ☑ tool** | axes of the trajectory gizmo (the servo gizmo has its own setting); both move and rotate, only the axes differ: `base_link` = the gizmo (trajectory gizmo in Draw, waypoint gizmo in Edit) rests at the base axes, a ring turns the pose about that base axis and the gizmo returns to the base axes after release; `tool` = the pose's own axes. Hidden while point tracking is on (base_link axes, no rotation; a moved waypoint then turns toward the point) |
| *Orientation* **Look at point** | turns the tool z axis of the trajectory gizmo (Draw) or of the selected waypoint (Edit) toward the red point of *Point tracking* (default −3000, 0, 0 mm), by the smallest rotation; also when tracking is off. The path is planned with the waypoints' own orientations (interpolated between them); files from the earlier "path looks at the point" version load with the orientations they had along that path |
| **Plan continuously** (off by default) | on: what is chosen (path, segments) is planned 0.4 s after every change and again when the robot comes to rest elsewhere, and the preview repeats; off: the plan and its preview are removed **at once** after a change, then only a check runs (red segments, fastest times) until **Plan** |
| *Plan & Execute* › *Plan* **Whole path / Segments / To gizmo** | what **Plan** plans: the whole path; only the waypoints *From waypoint* → *To waypoint* (highlighted yellow, summary `(segments 4→5)`), moved exactly as within the whole path; or a free motion (RRT + TOPP-RA) to the trajectory gizmo. *Plan continuously* plans the chosen path/segments after every change |
| *Speed* (0.05–1) | execution speed of every plan: the planner divides all times by it (0.5 = twice as long; velocities × 0.5, accelerations × 0.25) and publishes a waiting plan again when it changes; the preview shows exactly that trajectory |
| **Plan** | plans approach + path (or the chosen segments) now; the plan line in the top-left window says what is planned (`Planned: path 1→6 · 6.60 s (approach 0.92 s)`, `… · speed 0.50`); the preview plays it once |
| **Execute** | right below Plan: runs the plan |
| *Waypoint* folder | `gizmo` (Draw: the gizmo's pose, a `gizmo` row in the table) or a waypoint number (switches to *Edit*; also by clicking its sphere): its pose in `base_link` (mm, FAIRINO rx/ry/rz °), **Stop [s]** (rest at that waypoint, not at the first/last), **Blend [mm]** (corner blend radius, below), **Straight**, **Delete Waypoint**; **Move [mm]** / **Rotate [deg]** + *Frame* (`base_link`, or `local` = the gizmo's / waypoint's own axes) + **Apply Offset** moves the gizmo (Draw) or the waypoint (Edit) by exactly that amount; table of the shown waypoints (40 per page, *Rows*): x, y, z [mm], stop [s], R [mm] (blend radius) |
| *Segments* | a table of all segments (a drawn stroke is one row, its speed is set in the servo view), 40 rows per page (*Rows*): segment, how it was made (`—` line, `∿` curve, `✍` drawn in the servo view, `⏺` from a recording), fastest and set time [s] (⚠ = below the fastest: executed as fast as possible, drawn red); pick a segment in *Segment* and set *Time [s]* (0 = as fast as possible) |
| Files view › *Path files*: **Save** / **Load** / **Refresh** | `paths/<name>.json` (`format: fairino_path`, version 3; older files load as straight segments); Load switches to the trajectory view |

### Curves, as in the FAIRINO Viser prototype

In *Edit*, a selected waypoint shows two tangent handles. On a straight
waypoint they sit 35 % of the way toward the previous and the next waypoint
(only as markers: the segments stay straight until a handle is dragged).
As in the prototype, each handle gizmo is turned so that one of its axes
lies on the line waypoint → handle, so dragging along that arrow only
lengthens or shortens the tangent (re-aligned 0.6 s after a drag that
changed the direction).
Dragging one handle bends the path through the waypoint as a cubic Bezier;
the other handle mirrors it (one symmetric tangent, as in the prototype), so
the robot passes the waypoint without stopping. **Straight** removes the
curve. The larger gizmo on the waypoint moves/rotates the waypoint itself
(waypoints carry the tool orientation, which the prototype's points did
not); its tangent handles move with it.

Several waypoints at the same point (a path that returns somewhere) share one
sphere; next to it, a short row of small numbered spheres (one per waypoint,
4 cm apart, on the side where the waypoint gizmo has no handles) shows
which they are without overlapping labels. Click one of them to pick that
waypoint (Edit) or to put the gizmo on it (Draw); the *Waypoint* dropdown
works too.

Whatever is planned is checked: a segment that cannot be executed turns red
and the summary says why (unreachable pose, configuration jump/singularity,
joint limit, self-collision). After an edit, a plan made earlier is removed
when *Plan continuously* is off, so Execute never runs a plan that no longer
matches the drawing. The plan, its preview and the only **Execute** button
are in the *Plan & Execute* folder of the trajectory view.

Colours: green = path segments and the segment being drawn, red = a segment
(or the segment being drawn) that cannot be executed, with a red sphere
*✕ fails here* where it fails, grey = not checked because the path fails
elsewhere, yellow = selected
segment/waypoint, cyan = tangent handles, blue (thin) = the planned TCP
trajectory including the approach (the move from where the robot is to the
start of the path; drawn from the planned trajectory every ~2 mm of TCP
travel, so it shows the curves exactly as the preview robot moves).

Teaching workflow: **View: Servo**, drive the robot with the servo gizmo
and press **Confirm Waypoint** (Servo folder) at each pose (the first one
starts the path), or draw: while the gizmo is held, its motion is recorded
from the last waypoint (time runs only while it is held; holding it still
≥ 0.5 s is a stop), and **Confirm Waypoint** turns the drawing into curve
waypoints with the drawn segment times (`drawing_to_path`: still intervals
→ waypoint stops, moving parts → `poses_to_path` as for recordings, 3 mm) and
ends it with a waypoint at the gizmo. **Discard** drops an unconfirmed
drawing, **Undo** the last addition. A drawing that does not start at the
last waypoint is joined to it by a straight segment. **View: Trajectory**;
check the plan and the preview; **Execute** (Plan & Execute). Drawing
without the robot: move the trajectory gizmo and press **Confirm Waypoint**;
it stays where it was put, *Gizmo to Robot TCP* (Plan & Execute) or *New
Trajectory* bring it to the robot.

Stops: a waypoint with *Stop [s]* > 0 is a stop sample (the robot is at
rest there) and the planner inserts the rest after timing (`insert_dwells`);
the speed scales it like every other time. Not at the first or last waypoint
of what is planned. File key `dwell_s` per waypoint.

Corner blend radius (as UR's blend radius or ABB's zone): a straight
waypoint between two segments with a change of direction normally is a stop.
With *Blend [mm]* > 0 (or the blend handle: in *Edit* a third small gizmo on
the inner bisector of the corner with one arrow along it, labelled `R … mm`;
its distance from the waypoint is the radius, closer than 20 mm = no blend) the path leaves the
incoming segment that far before the waypoint and joins the outgoing one as
far after it, along a cubic Bezier tangent to both (for straight segments the
quadratic one with its control point at the corner, whose middle passes
r·|d_out − d_in|/4 from the waypoint), and the robot passes without
stopping. The orientation continues the one of the sharp corner (incoming
segment up to the middle, then the outgoing one), so the turning rate has no
step. Each half of the rounding belongs to the segment on its side: segment
times, waypoint arrival times and *Segments* plans meet at the middle of the
rounding. The radius is limited to half the shorter neighbouring segment
(neighbouring roundings meet at most) and has no effect at curve waypoints
or without a corner; with *Stop [s]* the robot rests at the middle of the
rounding. File key `blend_m` per waypoint. Measured on
the operator's 22-waypoint path, corner of 76° at waypoint 3: segments 2 → 4
take 1.97 s sharp, 1.84 s with 50 mm, 2.02 s with 100 mm, 2.23 s with 200 mm
(the longer rounding is passed at the speed the joint accelerations allow
on that curve, about 0.4–0.6 m/s; a stop costs less than that there), whole
path 12.60 → 12.48 s with 50 mm, executed 0.6 mm from the plan.

Tuning one part: *Plan* **Segments**, *From waypoint* 4, *To waypoint* 5,
**Plan**, **Execute**: the robot makes a free approach to waypoint 4 and runs
only 4 → 5, with the same joint motion as in the whole path (the planner
follows the whole path's IK from waypoint 1 and cuts out the part; only if the
path before the part cannot be followed, the part starts from the
configuration nearest to the current one). Change the segment time, plan and
execute again.

### Stops and curves

Where two straight segments meet with a change of direction (> 1°) the robot
stops at the waypoint; collinear straight segments and curve waypoints are
passed without stopping. A segment is a cubic Bezier when at least one of its
waypoints is a curve waypoint: control points = start + its tangent and
end − its tangent (zero for a non-curve end), drawn with
`add_spline_cubic_bezier` exactly as it is planned. Orientation is interpolated with slerp along every
segment with a smoothstep profile (zero angular rate at the segment ends).

### Time per segment

The planner computes the time-optimal timing (joint velocity and
acceleration limits, curvature, stops, TCP at most 2 m/s) and reports the
fastest time of every segment; segments with a longer requested time are
slowed down by capping their speed (bisection) until they take that long.
Neighbouring segments joined without a stop can change slightly because the
robot accelerates or brakes across the waypoint. A requested time shorter
than the fastest is ignored (the UI says so).

## 3. Planning (planner, `/fairinoplan/path_goal`)

The UI publishes a JSON request (`std_msgs/String`) on
`/fairinoplan/path_goal`; the planner answers on `/fairinoplan/status` and
`/fairinoplan/planned_path` like for any plan.

```json
{"type": "cartesian_path", "name": "path",
 "waypoints": [{"position_m": [x, y, z], "wxyz": [w, x, y, z], "curve": false, "tangent_m": [0, 0, 0]}, …],
 "segments": [{"duration_s": null}, {"duration_s": 2.5}, …],
 "look_at_m": null,                      or [x, y, z] in base_link: the tool z axis looks at that point
 "part": [3, 4]}                         optional: plan only waypoints 4 → 5 (0-based indices)

{"type": "clear"}   removes the plan (empty planned_path, status "ready")

/fairinoplan/path_check (same waypoints/segments + "request_id") → /fairinoplan/path_check_result:
{"request_id", "ok", "message", "segment_fastest_s": […], "failed_segments": […]}   (no plan is made)
```

Cartesian path:

1. `sample_path`: samples every 2 mm (orientation counted as 0.2 m/rad), with
   per-sample speed limit and stop flags (start, end, waypoints with a change
   of direction).
2. IK: the first pose with damped least squares from the current
   configuration (from a distant configuration it can converge to angles of
   several turns: solutions are moved by whole turns toward the current
   configuration and must lie within the joint limits; if it needed such a
   move, damped least squares from the middle of the joint ranges and 12
   fixed pseudo-random seeds are tried too and the solution closest to the
   robot is taken; RoboPlan SimpleIk last), then continuous damped
   least-squares IK seeded by the previous sample (`PathIk.follow`). An
   unreachable sample or a joint jump > 0.25 rad between samples (singularity,
   configuration change) fails with the segment index; the UI draws it red.
   With `"part"` the whole path is followed from waypoint 1 to the end of the
   part and the part is cut out (`slice_path`: at rest at both ends), so the
   part moves exactly as within the whole path; if the path before the part
   fails, the part starts from the configuration nearest to the current one.
   Segment indices in the status stay those of the whole path
   (`first_segment` gives the first planned one).
   Corners with a blend radius are rounded before (`segment_pieces`).
3. Joint limits for every sample, self-collision every 5th sample.
4. Timing (`parameterize_path`): velocity limit curve from the TCP maximum,
   the joint velocity limits and the curvature term, forward/backward
   acceleration passes, cubic spline; where the spline still exceeds a joint
   limit, only that region is slowed down (local iterations); requested
   segment times by capping the segment speed; a uniform slowdown is the last
   fallback. The status carries `segment_fastest_s` and `segment_planned_s`.
5. Stops (`dwell_s` of interior waypoints) are inserted (`insert_dwells`).
6. Approach from the current configuration with RRT + TOPP-RA when needed;
   approach + path are published as one trajectory (4 ms samples, the
   trajectory controller interpolates at 1 kHz).

Every new request clears the previous plan, so Execute can never send a stale
plan, and Execute is refused when the robot is more than 3 mrad away from
the first point of the plan ("the robot moved since this plan was made").
Execute sends the trajectory with a zero header stamp ("start now"); the
published plan keeps the planning time for the UI.

### Tracking report

During every Execute the planner records `/joint_states` and the ServoJ
counters of `/fairino_robot/status`. When the controller reports success it
waits until the robot is at the last point (at most 2 s), then publishes
*succeeded* with `tracking`:

| Field | Meaning |
|---|---|
| `max_deviation_mm`, `mean_deviation_mm`, `p95_deviation_mm` | distance of every measured TCP position to the planned TCP path (spatial: a robot that is late but on the path counts 0) |
| `worst_at_s` | time of the largest distance after the start of the motion |
| `executed_s`, `planned_s` | first to last motion of the robot vs. the plan |
| `arrival_after_controller_s` | how long after the controller finished the robot reached the last point |
| `servoj_min_rate`, `servoj_mean_rate`, `servoj_expected_rate` | executed ServoJ points/s (lowest over 0.5 s windows, mean) and 1/cmdT |
| `servoj_limited_points`, `servoj_flow_skips` | points shortened by the per-point step limit / cycles skipped by flow control during the execution |

The UI shows it under **Execute** (and the largest distance in the Path
tab); above 5 mm with a ServoJ rate below 95 % of 1/cmdT it names that as the
reason (see [servo_benchmark.md](servo_benchmark.md) §7).

## 4. Files and directories

`<trajectory_dir>/recordings/*.json` and `<trajectory_dir>/paths/*.json`.
`trajectory_dir` (UI parameter) defaults to `$FAIRINO_TRAJECTORY_DIR`, then
`/ros2_ws/trajectories`, which `docker/compose.yaml` bind-mounts from
`fairino_ros2/trajectories` on the host, then `~/.ros/fairino_trajectories`
inside the container. The bind mount becomes active when the container is
recreated (`docker compose up -d`); until then files are written to
`~/.ros/fairino_trajectories` in the container.

## 5. Measured (SimMachine 3.9.9, 2026-10-15)

| Test | Result |
|---|---|
| Cartesian path (earlier version with corner blending, since removed from the editor): 5 waypoints, blend 50 mm, sharp corner with 0.6 rad reorientation, smooth Bezier waypoint, 0.3/0.4 m/s | check + plan 0.09–0.12 s; approach 0.29 s + path 3.68 s; executed, TCP max 1.5 mm (mean 0.5 mm) from the designed path; TCP speed in the planned trajectory ≤ 0.400 m/s; 0.28 m/s through the blend, 0.13 m/s through the smooth waypoint, stop (0.009 m/s) at the sharp corner; closest approach to the blended waypoint 17.7 mm (r = 50 mm); no FAIRINO code |
| same path before local slowdown and orientation smoothstep | 5.81 s (uniform slowdown ×1.64), 0.01 m/s at the smooth waypoint |
| Recording through the UI: 3 planned moves | 505 samples, 4.1 s, 3 segments (planning) |
| Replay, recorded timing | 4.63 s (×1.12 local stretch); executed, max 0.23° (mean 0.03°) from the recorded joint path, end pose identical |
| Replay, fastest within limits, 100 % / 50 % | 2.91 s / 4.37 s; 0.24–3.2° / 0.17° from the recorded path (the 3.2° once, while SimMachine lagged: 399 point-limited ServoJ points) |
| Edit as Path → Plan Path → Execute (earlier version: blend corners) | 21 waypoints (2044 mm); reachable; approach 0.56 s + 9.42 s; executed, ended at the recorded end pose |
| Path analysis (earlier version with per-segment speeds), 3 straight segments near home | 0.07 s; max feasible 1555 / 1311 / 1134 mm/s; with a waypoint 2.5 m away: segment 2 → 3 red ("pose not reachable at 18 %"), Plan Path names the same segment |
| Plan + Execute with every segment at its max feasible speed (earlier version) | planned without slowdown (joint velocity 0.85, acceleration 0.94 of the limits), peak 1014 mm/s (short segments), executed, no FAIRINO code |
| Draw flow through the UI (earlier version) | New Path → robot moved, *Reset Target*, Confirm ×2 → 3 waypoints, 2 segments, max feasible 268 mm/s on segment 2 → 3; Bezier on → 280 mm/s; Plan Path (0.54 s approach + 2.72 s); Execute from *Control*: succeeded |
| Editor buttons (headless Chrome, earlier version) | Clear, add at robot ×2, Undo, Redo, Save, Load: as documented |
| Plan mode regression (joint and Cartesian goals) | 7/7 succeed, no FAIRINO code |
| Path v3 (planner): 4 waypoints, two curve waypoints | fastest 0.432 / 0.399 / 0.592 s, planned in 0.09 s; requested 3.0 s and 2 × fastest: planned 0.497 / 3.000 / 1.184 s (0.20 s); executed in 4.71 s (planned 4.68 s), no FAIRINO code |
| Execute after the robot moved / after clear | refused ("the robot moved since this plan was made"; "No planned trajectory.") |
| Path v3 through the UI | New Path, Confirm ×2: planned automatically (1.25 s + 0.47 s approach); curve at waypoint 2: 1.15 s; time 2.5 s on segment 2 → 3: planned 2.50 s (fastest 0.60 s); Execute from *Control*; Clear removed path, planned path line and preview, planner "plan cleared" |
| Path v3 with prototype curves (UI, after a host reboot) | New Path moved the gizmo to the robot; Confirm ×2 planned while drawing (1.07 s + 0.47 s); curve file loaded and planned (0.94 s), executed, approach planned again by itself; Straight → 1.00 s; *Plan while drawing* off + Undo → check only (fastest 0.43 s, no plan); Plan Trajectory → planned; Execute → "executed — press Plan Trajectory to run it again"; Clear → plan cleared |
| Editor with Draw/Edit modes (UI) | Apply Offset +3000 mm on the gizmo → draft line red and "cannot be reached" before Confirm; −2800 mm → green again; Confirm → planned; offsets +250 mm y and −250 mm z, Confirm ×2 → planned 1.01 s while drawing; picking waypoint 2 → Edit, TCP gizmo hidden, waypoint gizmo + two tangent handles shown; *Plan while drawing* off → "plan cleared" immediately; Plan Trajectory reports an unreachable segment |
| Edit as Path, new conversion, executed at the fastest timing (83.5 s recording) | 232 waypoints, path within 2.6 mm of the recording; plan 35.8 s; executed within 0.84 mm of the plan (ServoJ 2 ms / queue 1) and within 2.6 mm (mean 0.33 mm) of the recording |
| Path with point tracking (UI, first version) | 4 waypoints, point 0.5 m below the start and offset; planned 2.01 s + approach 0.35 s (the tool first turns to the point); every planned sample within 0.000° of the point, at most 0.38° rotation between 4 ms points; executed within 0.5 mm of the plan; file stores `look_at_m` |
| Unit tests `test/test_trajectory_tools.py` | 19/19 |

Not tested yet: dragging the waypoint gizmo and the tangent handles, and
clicking waypoint spheres in the 3D view (Draw: back to a waypoint; Edit:
select / cycle through a shared point), because Three.js handles cannot be
driven from the headless test; the code path behind them (waypoint
pose/tangent update, append, redraw) is the same as for the buttons and
dropdowns.
