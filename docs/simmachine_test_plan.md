# SimMachine test plan and results

Test procedures for the Viser Cartesian servo stack, their results on FAIRINO
SimMachine 3.9.9 (FR20-V1-001(V6.0), web v3.9.9, controller V3.9.39-LX) and
on mock hardware, and the staged plan for the first real-robot session.
Executed on 2026-10-14/15 unless noted. Tools used:

| Tool | Purpose |
|---|---|
| `fairino_servo_benchmark.py` | enable, drive a circle, report timing and hardware counters |
| `fairino_fault_injection_test.py --simmachine` | lower global speed, provoke controller faults, check recovery (refuses without `--simmachine`) |
| `fairino_servo_state_machine_test.py` | deterministic state-machine test with a fake `/fairino_robot` (mock hardware only; refuses when real hardware publishes) |
| development scripts (not installed) | braking/Reset-Target measurement on mock, recovery smoke test on SimMachine (14 checks), command-vs-measured lag by cross-correlation, flow-counter logger, Enable-while-moving check |
| `colcon test --packages-select fairino_cartesian_servo` | unit tests (fault catalog, rate limiter, timing statistics) |
| standalone SDK probes (development only) | CNDE configuration, MoveJ speeds, safety-speed effect, error-code timing |

## 1. Preconditions

1. `./simmachine.sh up`; SimMachine WebApp reachable at `http://192.168.58.2`.
2. Robot enabled in the WebApp, no active alarm, no program running.
3. `docker compose exec ros2 bash`; `ROS_DOMAIN_ID` is 42 (isolates the stack
   from other ROS 2 systems on the host).
4. No `ros2_cmd_server` running.
5. Robot in a non-singular pose (home (0, −90, 90, −90, −90, 0)°; T55 used
   (0, −90, 90, 0, 90, 0)°) via plan mode
   or the WebApp).

## 2. Executed tests

| ID | Test | Procedure | Expected | Result |
|---|---|---|---|---|
| T01 | Bringup on SimMachine | `ros2 launch … use_mock_hardware:=false` | controllers JSB/JTC active, servo controller inactive; `/fairino_robot/status` connected; streaming ARMED; Viser HTTP 200 | **PASS** (129 CNDE fields @ 4 ms, feedback 250 Hz, `write()` p99 1.01 ms) |
| T02 | Single SDK owner | status, recover, stop, speed services from the plugin, no command server | no second SDK client | **PASS** |
| T03 | CNDE field configuration | probe: default config, live reconfiguration, configuration before `RPC()` | — | **Finding**: default fields 3..75 @ 8 ms; live change gives garbage values; configuring before `RPC()` works (fixed in the plugin) |
| T04 | Trajectory controller moves SimMachine | 2° J6 trajectory via `follow_joint_trajectory` | motion, no error | **PASS** |
| T05 | Global speed service | without confirmation; `SetSpeedInstant(50)`, `SetSpeed(30)`, restore 100 | refusal audited; values read back | **PASS**; CNDE reports a fraction (0.5) — converted to % |
| T06 | `/fairino_robot/stop` and `/recover` | call both | disarm; full recovery sequence; re-arm | **PASS** after fixing a callback-group lifetime bug (stop service was never executed) |
| T07 | Servo state transitions | disable, set_mode servo, enable, hold, resume, disable | correct states and controller ownership, no jump | **PASS**, 0 guard trips |
| T08 | Plan mode | pose goals ±15 cm, Plan, Execute | plan < 0.1 s, execution succeeds | **PASS**: 18–43 ms planning, TCP error 0.8 mm |
| T09 | Servo benchmark | slow/fast circle, 1 kHz, 250 Hz, barrier 5 | no overruns, ACTIVE throughout | **PASS**, re-measured with the final code: all runs ACTIVE 60 s; one host stall (4–8 ms) in two fast runs (see [servo_benchmark.md](servo_benchmark.md)) |
| T10 | ServoJ latency | step target, trace command vs measured | robot follows in < 50 ms | **Finding then PASS**: 2–4 s delay from a growing controller FIFO (SimMachine consumed 347–932 points/s); fixed with flow control (queue ≤ 9, ~30 ms lag) |
| T11 | Global speed vs ServoJ | same circle at 100 %, 20 %, 5 % | — | **Measured**: no change of ServoJ speed; global speed does not scale ServoJ |
| T12 | Real fault and automatic recovery | `fairino_fault_injection_test.py --simmachine --speed 5` | whitelisted 1/23 → automatic recovery → HOLD; resume | **PASS**: 1/23 recovered in 0.06 s, HOLD, resumed. The fault appeared in 2 of ~32 attempts before T15; since the per-point step limit the test no longer provokes it (2 sessions × 3 × 20 s, no fault), so automatic recovery of a real controller fault is now covered by this earlier run and mock S1/S10 |
| T13 | Recovery in plan mode | fault while the trajectory controller is active | trajectory controller restarted, streaming re-armed | **Finding then PASS**: JTC held the stale command (0.032 rad); hardware refused to arm (correct); fixed with `set_last_command_interface_value_as_state_on_activation: false` + command priming |
| T14 | `abnomalStop` after reset | inspect status after 1/23 + reset | — | **Finding**: flag stays 1 with main/sub 0/0; made informational (was blocking re-arm) |
| T15 | Large single-step after simulator stall | 5 % speed, stalls | no controller speed fault | **Finding then PASS**: 1/23 faults from one large step after a stall; per-point step limit added; 60 s at 5 % with 2 stalls, no fault |
| T16 | `SetVelReducePara` gating | without confirmation; invalid combination; confirmed with/without password | refusals audited; code 0 when allowed | **PASS** (launch with `allow_safety_config_writes:=true`) |
| T17 | Safety-speed effect | MoveJ and ServoJ with manual-mode safety speed 30 deg/s / 100 mm/s | — | **Measured**: MoveJ stopped (stop+alarm) or limited to 30 deg/s (auto); **ServoJ not supervised**. Restored to disabled |
| T18 | Communication loss | `docker pause` SimMachine for 1.5 s with the servo in HOLD | `read()` ERROR after 0.5 s; servo LOCKOUT | **PASS**; recovery: `set_hardware_component_state FairinoSystem active`, Reset Fault, Disable |
| T19 | Command delivery under CPU load | benchmark while 3 headless Chrome instances render | no false jump-guard trip | **Finding then PASS**: a trip (5.1 mrad > 4.2 mrad) from a 10–20 ms delayed command; gap-aware guard + `chrt -f 40`; 45 s under load, 0 trips |
| T20 | Viser UI | screenshots of Control, Status, Speed, Audit on SimMachine | all fields shown, buttons enabled per state | **PASS** |
| T21 | Deterministic state machine (mock) | `fairino_servo_state_machine_test.py` | 15 checks | **PASS 15/15** (auto recovery, repeat → LOCKOUT, Reset Fault from LOCKOUT, E-stop, non-resettable, unknown code, collision, safety door, status lost, ServoJ 29, robot disabled, failing recovery); rerun after all review fixes: 15/15 |
| T22 | Unit tests | `colcon test` | — | **PASS 15/15** (incl. timing-statistics reset applied by the real-time producer) |
| T23 | Mock stack | enable/hold/resume/disable, Plan/Execute | — | **PASS** (TCP reached the goal within 1 mm) |
| T24 | Braking when leaving ACTIVE (mock) | drive at speed, Hold; record the 1 kHz command | deceleration at 540 deg/s², no 1-cycle stop | **Finding then PASS**: command froze within 1 ms; now 1.383 rad/s → 0 in 147 ms at 9.42 rad/s² |
| T25 | Reset Target while moving (mock) | drive at speed, Reset Target | no velocity jump | **Finding then PASS**: velocity zeroed in 1 cycle; then 10.3 rad/s² (ProxQP tolerance); with the exact clamp 9.42 rad/s² |
| T26 | External hardware stop in HOLD | `/fairino_robot/stop`, then Reset Fault, resume | FAULT, no silent re-arm; recovery → HOLD; ACTIVE stays ACTIVE | **PASS**: FAULT "stop requested", Reset Fault 0.10 s → HOLD + ARMED, resume ACTIVE |
| T27 | Hardware recover in plan mode | `/fairino_robot/recover` while DISABLED; Reset Fault + Enable right after a recovery | streaming re-armed, servo DISABLED; Enable works immediately | **Finding then PASS**: the servo accepted a cached ARMED status from before the recovery and Enable then met WAIT_SYNC; now only a status received after the hardware recovery counts. Transient "recovery in progress" is not a FAULT in DISABLED |
| T28 | Hardware deactivate/activate | `set_hardware_component_state FairinoSystem inactive/active`, 1 + 5 cycles | LOCKOUT while inactive, Reset Fault refused without status; after activation WAIT_SYNC → ARMED, Reset Fault, Enable | **Finding then PASS**: `ros2_control_node` crashed (SIGSEGV in the SDK CNDE thread) on the 2nd cycle because `CloseRPC()` leaves that thread running; closed SDK instances are now kept alive; 6 cycles without a crash, 0 guard trips |
| T29 | Following delay after re-activation | T28 cycle, then a 10 s circle with command/measured recording | lag ≈ 30–40 ms | **Finding then PASS**: 568 ms permanent lag (phantom executed points, 40 ms re-baseline); now 38–42 ms, also after 6 cycles and after a 5 % global-speed episode |
| T30 | Flow-control starvation | fast circle with the flow counters logged at 50 Hz | — | **Finding then PASS**: stream stuck at exactly 250 points/s (estimate "full", FIFO empty), fast circle FAULT in 2 of 4 runs; with the drift probe 3/3 fast runs ACTIVE, one correction in 2 runs |
| T31 | Enable while a trajectory is playing | execute a 40° J1 plan, Enable after 0.4 s, Enable again at rest | refused while moving, accepted at rest | **PASS** ("robot is moving (trajectory running?)", then ACTIVE) |
| T32 | Recovery smoke test (SimMachine, final code) | T26–T28 in one script, 14 checks | all pass | **PASS 14/14** |
| T33 | TCP frame vs FAIRINO | `GetForwardKin` for 12 random configurations vs URDF; live servo reset pose vs `tl_cur_pos` | gizmo/servo TCP = FAIRINO TCP (tool 0) | **Finding then PASS**: the gizmo was on `wrist3_link`, 120 mm behind the FAIRINO flange (same orientation); new URDF frame `flange`: 0.011 mm / 0.0004° live, Status tab shows 0.01 mm |
| T34 | Plan-mode acceleration | 7 plan+execute goals (joint and Cartesian) at 5.0 and 9.42 rad/s² | all succeed, no FAIRINO code, no disarm | **PASS**: J1 40° 0.76 s → 0.61 s (≈ 1.7 s at 1.0 rad/s²), peak 120 deg/s |
| T35 | Plan preview in Viser | plan without Execute; screenshots 0.3 s apart (headless Chrome) | translucent robot replays the plan, loops, hidden after Execute | **PASS** (ghost moves from home along the blue path to the goal; 1 s pause on the goal) |
| T36 | Servo with τ 0.03 s and 2.0 m/s | slow/fast circle 60 s | ACTIVE, no fault | **PASS**: TCP RMS 5.7 mm / 43.2 mm (was 7.0 / 51) |
| T37 | Regression after T33–T36 | mock state machine, unit tests | all pass | **PASS** 15/15, 15/15 |
| T38 | Simplified operator UI (headless Chrome clicks) | no mode switch / Replay / separate Reset Fault; Home → preview → Execute; Enable → external stop → Recover; hardware inactive → Recover → active → Recover | buttons work as documented | **PASS**: robot from (40, −80, 100, −110, −90, 20)° to home (0, −90, 90, −90, −90, 0)°; FAULT → Recover → HOLD; LOCKOUT: Recover refused while the status was missing, accepted after activation → DISABLED |
| T39 | Cartesian path (planner) | 5 waypoints: blend 50 mm, sharp corner + 0.6 rad reorientation, smooth Bezier waypoint; check, plan, execute; executed TCP vs designed path | reachable, executes, stops only at the sharp corner | **Finding then PASS**: first version slowed the whole path ×1.64 for one corner and almost stopped at the smooth waypoint (orientation rate jump); with local slowdown and smoothstep orientation 3.68 s, 1.5 mm max deviation, 0.28 / 0.13 / 0.009 m/s at blend / smooth / sharp |
| T40 | Execute of a plan created earlier | plan, wait longer than its duration, Execute | executes from the start | **Finding then PASS**: the plan carried its planning time as start stamp; the controller rejected it ("ends in the past") or would skip its beginning when executed soon; Execute now sends a zero stamp |
| T41 | Recording through the UI | Record, 3 planned moves, Stop & Save | file with 3 segments | **PASS** (505 samples, 4.1 s) |
| T42 | Replay, recorded timing | Load, Plan Replay, Execute; executed joints vs recording | follows the recording | **Finding then PASS**: raw replay needed ×5.2 slowdown (stale samples, stamp jitter, SimMachine catch-up jerks); smoothing + local stretch: ×1.12, max 0.23° from the recording |
| T43 | Replay, fastest within limits | 100 % and 50 % | within limits, follows the path | **PASS**: 2.91 s / 4.37 s; 0.17–3.2° deviation (3.2° once, during SimMachine lag) |
| T44 | Edit as Path | recording → 21 waypoints → Check → Plan Path → Execute | reachable, executes | **PASS** |
| T45 | Path editor buttons | Clear, At Robot, Undo, Redo, Save, Load, Check (headless Chrome) | as documented | **PASS**; gizmo dragging not automated |
| T46 | Unit tests of the trajectory tools | `python3 -m pytest fairinoplan_bringup/test/test_trajectory_tools.py` | — | **PASS 15/15** (current model) |
| T47 | Path editor v2 (UI) | New Path at the robot TCP, Confirm ×2, Bezier on, Plan Path, Execute from *Control* | yellow draft line, max feasible speed shown, executes | **PASS** (268 → 280 mm/s with Bezier; 0.54 s + 2.72 s) |
| T48 | Path analysis (planner) | reachable path; waypoint 2.5 m away | per-segment max speed; failing segment named | **Finding then PASS**: first version reported 130–170 mm/s (corner kink in the derivative); per-segment derivatives: 1555 / 1311 / 1134 mm/s; failing segment 2 → 3 in analysis and Plan Path |
| T49 | Path at the reported max speeds | plan + execute | no slowdown, no fault | **PASS** (0.85 velocity / 0.94 acceleration of the limits, no FAIRINO code) |
| T50 | Path editor v3 (planner) | curve waypoints, requested segment times, execute | requested times met, executes | **PASS** (3.000 s and 2 × fastest met exactly; executed 4.71 s vs 4.68 s planned) |
| T51 | Execute safety | Execute after the robot moved; after clear | refused | **PASS** |
| T52 | Path editor v3 (UI) | draw, curve, time, automatic planning, Execute, Clear | as documented | **Finding then PASS**: Clear did not remove the plan (planning status without its source); fixed; Clear removes path, planned path line, preview and plan |
| T53 | Path editor: prototype curves, plan while drawing on/off (UI) | New Path, Confirm ×2, curve file, Execute, Straight, off + Undo, Plan Trajectory, Execute, Clear | as documented | **PASS** (finding: execution statuses carried no plan source, so the editor kept showing "planned" after Execute; fixed) |
| T54 | Path editor: Draw/Edit modes, live draft check, offsets (UI) | Apply Offset far/back, Confirm, Edit selection, plan while drawing off, Plan Trajectory | draft red before Confirm; Edit hides the TCP gizmo and shows waypoint gizmo + 2 tangent handles; plan cleared at once | **PASS** (findings: plan-while-drawing off took effect only on the next edit, draft line checked only after Confirm, Waypoint dropdown stale after New Path; fixed); dragging handles and clicking spheres not automated |
| T55 | Path editor polish (UI) | curved file from the new home, select segment, click waypoint 3 in Draw, Edit + numbered spheres of the shared point | ✅ summary with time; yellow selection; back-click adds 5 and moves the gizmo there; clicks pick 5 → 3 → 5; FPS readout, base_link axes, no logo | **PASS** (finding: the large waypoint gizmo covered the shared sphere, so clicking it again could not reach the other waypoint; replaced by numbered spheres) |
| T56 | Path editor: modes, gizmo rule, handle axes, recording trail (UI) | Draw/Edit check boxes; Confirm without New Path; Undo/Redo; click waypoint 2 in Draw then Confirm; handles on a diagonal path; record a planned move | one mode always ticked; gizmo stays on the confirmed pose; Undo/Redo put it on the path end; click only moves the gizmo, Confirm adds the waypoint; handle axis on the line (max error 4e-16 over 500 random cases); trail stable and on the planned curve | **PASS** (findings: first Confirm without New Path reset the gizmo to the robot; the trail was resampled every n-th sample with n growing, so it jumped; fixed) |
| T57 | Recording → path fit, tracking report, ServoJ cmdT | 83.5 s recording: Edit as Path, plan at the fastest and recorded timing, Execute with cmdT 1 ms/8, 2 ms/4, 2 ms/1; servo benchmark circles | path within tolerance; tracking report after Execute; servo tracking unchanged | **PASS with 2 ms / 1** (findings: Catmull-Rom tangents overshot by 55 mm, now 2.6 mm; SimMachine executes only 260–930 ServoJ points/s, which capped the joints at 50–93 % with 1 ms points; 2 ms / 4 left a hidden 0.55 s backlog and a servo FAULT; 2 ms / 1 tracks like 1 ms; see servo_benchmark.md §7) |
| T58 | Draft line after Draw → Edit (UI) | Draw with the gizmo out of reach (red line), Edit, Draw, Edit | line only in Draw | **Finding then PASS**: resetting the line's key to `None` on a mode switch or a check result equalled "nothing to draw", so the last draft line stayed in the scene in Edit mode; fixed (separate redraw marker, the line is always removed when nothing is to be drawn) |
| T59 | Look at point: servo and path (UI) | plan mode: target aims; servo enabled with the tool 36° off; servo: point in front, look-at on, point moved 4×; path with look-at, Execute, Save | target and robot aim at the point; guard; path samples aim; file stores the point | **PASS**: target 0.03°, robot 0.0° (FAIRINO `tcp_pose`) for four point positions, TCP position unchanged; servo enabled 36° off → look-at switched itself off; path samples 0.000°, ≤ 0.38° per 4 ms, executed 0.5 mm from the plan; unit test: prototype formula jumps > 45° on a line where ours changes < 1° per mm |
| T60 | Point tracking, separate servo/path points, recording trail (UI) | servo: Enable, tracking on, point moved, TCP gizmo moved twice, tracking off; path: New Path, tracking on, Confirm ×2, plan, Execute; Record tab | servo point created on the tool axis; robot moves and keeps looking; path point independent; TCP gizmo not turned by the path; trail only after Load | **PASS**: servo 0.0° right after switching on (nothing turned), 0.0° after moving the point and the TCP (robot moved 150 mm), no jump when switched off; path point created on the start's tool axis, TCP gizmo orientation unchanged, stored waypoints 0.04–0.06° (display rounding), planned samples 0.001°, executed 0.4 mm from the plan; Library Load/Unload/Refresh, autosave not listed (findings from the operator: one shared point mixed servo and path, the gizmo turned while drawing, the trail of a just-saved recording could not be removed, a gizmo jumped while dragged — redraws set its pose mid-drag; fixed) |
| T61 | Point tracking as a toggle, j6 limit, segment table (UI) | toggles Enable/Disable Servo, Hold/Resume, Point Tracking; TCP moved with tracking; point circled 3× around the tool; path with 3 segments edited in the table, Undo; 231-segment path; Unload | buttons switch label; aim kept; j6 stays in range; table edits plan | **PASS**: aim 0.0° with the TCP moved; while circling j6 went −16° → −160° and stayed at −160° (15° from the limit) with aim 0.0°, servo ACTIVE, no fault; table: 2 → 3 = 2.0 s planned (1.29 → 2.97 s), 0.1 s listed as too fast, Undo restored the field; 25 rows per page; path point by coordinates 0.000° (finding from the operator: the roll had run j6 into its limit, OInK QP failure → LOCKOUT, robot no longer followed; fixed) |
| T62 | TCP gizmo with the Path editor in Edit mode (UI) | Edit mode, Enable Servo, Servo Hold, Disable Servo, Draw | gizmo visible whenever the servo is on | **Finding then PASS**: Edit mode hid the TCP gizmo also in servo mode (Enable Servo showed no gizmo); now visible in ACTIVE/HOLD, hidden again in Edit after Disable Servo, visible in Draw |
| T63 | Waypoint gizmo axes, Orient to point, j6 margin 5° (UI) | 3-waypoint path, Edit waypoint 2 with base_link axes, Orient to point, waypoint axes; Draw: Orient to point; servo: point circled 3× | gizmo axes as chosen; waypoint/gizmo look at the point; j6 held at ±170° | **PASS**: base_link axes without rotation rings, waypoint axes rotated with the waypoint; waypoint 2 63° → 0.04°, TCP gizmo 67° → 0.04°; circling: j6 −16° → −170°, aim 0.0°, servo ACTIVE |
| T64 | One tracking point, shared gizmo axes, gradual turn, labels (UI) | segment labels; base_link ↔ tool in Path and Control; Orient to point (Draw, Edit) with tracking off; tracking on from home with the servo active | one-line labels from 1; axes keep the target orientation and are synced; red point used; robot turns gradually | **PASS**: labels 14 px high (one line), no "start"; base_link keeps 180/0/90° and moves only; orient 0.04° to (−3000, 0, 0); tracking on: 63° turned in 1.5 s, aim 0.0° after; Path axes/Orient hidden while tracking (findings from the operator: label wrapped; base_link did nothing in Draw; Orient used a second purple point; fixed) |
| T65 | Path folder layout, base_link axes with rotation (UI) | Path folder order; base_link in Path and Control | New Path on top; Mode / Gizmo axes / Orientation rows alike; base_link gizmo with rotation rings | **PASS** (layout and rings checked on screenshots; turning a ring needs a mouse drag and was not automated) |
| T66 | Orientation after point tracking (UI, plan mode) | tracking on, off; base_link, tool; Look at point; base_link → tool; Reset Target | axes toggles never turn the target | **Finding then PASS**: switching tracking off kept the orientation toward the point, so pressing *tool* showed it as if *tool* turned to the point; now off → 180/0/90° (robot tool), base_link/tool unchanged, Look at point 0.05°, toggles keep it, Reset Target → robot tool |
| T67 | Free tool roll in the servo (PointingTask) | benchmark full pose before/after; roll-free with roll and tilt oscillation; UI: point circled 3× around the tool, tracking off while servoing | full pose unchanged; roll free: j6 does not follow the roll, tool axis as accurate, OInK not slower; no FAULT | **PASS**: OInK 0.052–0.059 ms mean, ≤ 0.204 ms max, no overruns in the roll-free runs; roll oscillation: j6 0.0…0.0° (full pose −10…9°), tool axis error equal (3.7° fast tilt, lag); UI circling: j6 0.0°, aim 0.0°, robot moved 150 mm while pointing; tracking off: TCP change 0; servo_math 5/5, fault catalog 9/9 (finding: one full-pose run had a single 2.46 ms cycle outside OInK — host jitter, as documented in §3) |
| T68 | Free tool roll as a service (UI, benchmark) | `set_free_tool_roll` from UI and benchmark; external `true` while the UI tracks nothing; UI circling test | servo state follows the requests; UI does not override other clients; status shows the mode | **PASS** (finding: the first UI version forced the servo back to `false`, which cancelled the benchmark's free roll — j6 moved −3.4…3.1°; now only `false` once at UI start and `true` again after a servo restart while tracking; benchmark roll free j6 0.0…0.0°, OInK 0.055–0.057 ms mean; UI circling j6 0.0°, aim 0.0°. A run disturbed by a second operator in the UI ended in FAULT *joint command tracking error 0.1516 rad* — a correct safety stop; repeated undisturbed: PASS) |
| T69 | Two views: Servo / Trajectory (UI) | start; View Servo; Record; point tracking in servo; View Trajectory; Home + Execute; Stop & Save; Add TCP to Path | views switch the mode; one recording across both; each view its own gizmo | **PASS**: start in Trajectory (plan mode); Servo → ACTIVE servo mode; tracking turned the robot (aim 0.0°, free roll on); Trajectory → DISABLED plan mode, free roll off, axes/Look at point hidden while tracking; Home executed; the recording kept going: 2 segments (servo, planning), 459 samples; servo gizmo on the robot, trajectory gizmo where it was left |
| T70 | Simpler layout: top-left status, Files view, Home in one step, Plan & Execute with segments (UI) | Files view load; *Segments* 3 → 4 with the robot far from waypoint 3, Plan, Execute; From/To given way; Home from Trajectory and from Servo; fake FAULT status (UI only); fault-injection test | partial plan moves the robot as within the whole path; Home ends at the home pose and back in servo from the servo view; Recover only on fault | **Finding then PASS**: *segments 3→4* failed "j1 outside its limits at 0 %": the start IK by damped least squares from the robot at waypoint 2 converged to j2 = 1443°, j3 = −4043° (several turns). Now the start IK must be within the limits (else SimpleIk) and a part follows the whole path's IK and is cut out: planned 1.93 s (approach 1.10 s), executed 0.7 mm from the plan, robot at the whole path's waypoint-4 joints (6.0, −88.9, 74.3, −142.8, −99.7, −0.5°). From 3 / To 2 → 3 / 4, To 2 → 1 / 2, From 6 → 5 / 6 (finding: the fields were not written back; fixed). Home from Trajectory: home pose, plan mode; from Servo (robot at waypoint 2): home pose, ACTIVE servo mode again. Recover box shown top left only while the (fake) FAULT status was published. The fault-injection test (global speed 20 % and 5 %) no longer provokes a controller fault with ServoJ 2 ms / queue 1, so the real FAULT path was not run in this test |
| T71 | Speed = preview = execution, stops, drawing in the servo view, layout (UI) | Speed 1 → 0.5 with a waiting plan, Execute; Stop 1 s at waypoint 3, segments 2 → 4; Waypoint folder in Draw/Edit; servo view: drag the gizmo (ring, then arrow) with a 1 s hold, Confirm, Undo, Confirm, plan the result; dark mode, Share button shown | the plan is published again at the new speed and executes in that time; the stop adds its time; Confirm only in Draw; the drawing becomes waypoints with one stop and plans | **PASS**: path 6.60 s → 13.21 s at 0.5 (approach 0.92 → 1.84 s), executed 13.20 s, 0.7 mm from the plan; segments 2 → 4 with a 1 s stop 4.03 s at speed 1 (2.24 s + 1 s + approach), 8.06 s at 0.5; Waypoint list `gizmo` + gizmo row in Draw, Stop field and no Confirm in Edit; drawing by ring: +10 waypoints (orientation only), 1 stop; Undo → 6 waypoints; drawing by arrow: 231 mm, +3 waypoints, 1 stop; path of 10 waypoints planned 11.25 s. Findings: (1) the start IK accepted only solutions within limits and otherwise relied on SimpleIk, which failed after the drag ("first waypoint is not reachable"): now turns are wrapped and more seeds are tried (3 cases checked offline); (2) a fast large rotation drag drove j4 to its limit and the servo went to FAULT "OInK result rejected on j4: joint result outside limits" (protection, robot stopped; not changed); (3) headless Chrome delivered the end of a drag only with the next click, so drawn times were 1–3 s too long in this test. Share link itself not created (it would publish the UI) |
| T72 | Edit mode without gizmo controls (UI) | *To gizmo* chosen in Draw, switch to Edit, back to Draw, Plan | Edit: no *To gizmo*, no *Gizmo to Robot TCP*, scope Whole path | **PASS**: Draw 3 choices + Gizmo to Robot TCP; Edit `☑ Whole path`, `☐ Segments` filling the row, Gizmo to Robot TCP and Confirm hidden; Draw again 3 choices; 22-waypoint path planned 12.95 s (finding from the operator: *To gizmo* in Edit planned to the hidden gizmo, two 0.66 s plans in the log, which looked like nothing happened) |

## 3. Not tested on SimMachine

| Item | Reason | Coverage |
|---|---|---|
| E-stop, safety stop SI0/SI1, safety door, safety plane, safety data | requires IO/safety simulation in the WebApp | mock tests S4, S8; catalog policy LOCKOUT |
| Collision fault (main 4) | no physical contact in SimMachine | mock test S7 |
| Singular pose 10/1, 1/37, 1/38 | not provoked | catalog whitelist |
| SDK 29 from ServoJ | OInK position limits prevent it | mock test S10 |
| Reduced mode, collision level changes | safety configuration, not changed by this stack | WebApp procedure documented |
| Self-collision barrier stopping a real motion | tested in a RoboPlan simulation (stops at d_min = 5 mm) | benchmarked on SimMachine (timing only) |

## 4. Real-robot staged plan

Run only after section 2 passes on the same software version.

1. **Preparation**: risk assessment, E-stop within reach, nobody in the
   envelope, cell cleared (no environment collision checking), payload/TCP in
   the WebApp, network direct to the controller, `controller_ip` set.
2. **Read the WebApp configuration** and record it: safety speed, reduced
   mode, collision levels, soft limits, global speed, robot mode.
3. **Bringup** with `use_mock_hardware:=false`; confirm 129 CNDE fields @ 4 ms,
   ARMED, connected, codes 0/0, `write()` p99 < 1.2 ms, feedback 250 Hz.
4. **ServoJ consumption**: watch `servoj_flow_skips`, `servoj_queue_estimate`,
   `servoj_flow_probes` and `servoj_flow_drift_corrections` at rest for 60 s
   (expected ≈ 0 skips and no probes on a real-time controller).
5. **Plan mode** at reduced speed: small joint goal, then a pose goal; verify
   motion and stop.
6. **Servo, enable without moving the gizmo**: zero jump (command = measured),
   tracking error < 1 mrad.
7. **Servo at `velocity_scale: 0.25`**: millimetre translations, then small
   rotations; Hold, resume, Disable; close the browser → HOLD within 1 s.
8. **Benchmark** slow circle (r = 0.05 m) at `velocity_scale 0.25`, then 0.5,
   then 1.0 if the risk assessment allows; record the tables of
   [servo_benchmark.md](servo_benchmark.md) and the TCP error.
9. **Safety speed vs ServoJ**: with the cell's real safety-speed settings,
   verify whether the controller supervises ServoJ (on SimMachine it did not).
10. **Recovery**: only benign, documented faults (e.g. a joint command speed
    fault provoked at low speed is acceptable only if FAIRINO confirms it is
    harmless); never provoke E-stop/safety faults for automatic recovery.
    Check that safety signals end in LOCKOUT by pressing the E-stop once with
    the robot at rest.
11. **Self-collision barrier** (`self_collision_pairs:=5`) only after the
    above, in a pose far from self-contact first.
12. Keep the ROS logs (`AUDIT` lines, timing, codes) and update the "tested on
    real robot" column in the catalog (`fault_catalog.cpp`), then regenerate
    [fairino_fault_recovery.md](fairino_fault_recovery.md).
13. Flow control on the real controller: `servoj_flow_probes` and
    `servoj_flow_drift_corrections` should stay 0 and `servoj_flow_skips` ≈ 0;
    record the following delay (command vs `/joint_states`) at rest and on the
    slow circle.
