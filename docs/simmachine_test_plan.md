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
5. Robot in a non-singular pose (home (0, −90, 90, −90, −90, 0)° via plan mode
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
