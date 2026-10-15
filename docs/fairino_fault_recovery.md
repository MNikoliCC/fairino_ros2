# FAIRINO fault monitoring and recovery

This document defines how the servo stack detects FAIRINO faults, which faults
may be recovered automatically, how every recovery is executed and audited,
and when the WebApp or a power cycle is needed. The code catalog at the end is
generated from the C++ source
(`ros2 run fairino_cartesian_servo fairino_fault_catalog`), so the table and
the behaviour cannot drift apart.

Automatic recovery is allowed **only** for whitelisted faults and never ends in
motion: in servo mode it ends in `HOLD`, in plan mode in `DISABLED` with the
trajectory controller holding the measured position. The operator must resume
explicitly. Safety and
non-resettable faults end in `LOCKOUT`, which never resets automatically.

## 1. Where fault information comes from

| Source | Path | Content |
|---|---|---|
| FAIRINO realtime state (CNDE, 4 ms) | hardware `read()` → `/fairino_robot/status` (50 Hz) | `main_error_code`, `sub_error_code`, `EmergencyStop`, button-box stop, `SafetyStop0/1State`, `collisionState`, `rbtEnableState`, `udpCmdState`, `safetyDoorAlarm`, `safetyPlaneAlarm`, `safetyDataState`, `driverAlarm`, `driverError`, `driverComError`, `ctrlBoxError`, `outSoftLimitError`, `abnomalStop`, `alarmRebootRobot`, `alarmCheckEmergStopBtn`, speed scales, collision levels |
| `GetRobotErrorCode()` | hardware recovery service | main/sub code verification after `ResetAllError()` |
| SDK return codes | every `ServoJ()` in `write()`, every API call in services | `last_servoj_return_code`, `servoj_errors`, per-step codes in `RecoverRobot.steps` |
| Hardware protections | `write()` | streaming state and reason: discontinuity guard, non-finite command, feedback loss, stop request, recovery |
| Servo watchdogs | servo loop / supervisor | joint-state age, tracking error, joint limits, OInK result, controller ownership, target age |

The hardware plugin never just logs and continues: any controller fault,
safety signal, non-zero ServoJ code or protection trip disarms ServoJ
streaming in the same cycle and is published for the state machine. A failed
or stale `read()` keeps the last good position, disarms after 100 ms and
returns `hardware_interface::return_type::ERROR` after 500 ms.

Three SDK 3.9.9 behaviours seen on SimMachine are handled explicitly:

- `GetRobotErrorCode()` lagged the realtime codes: it returned 0/0 right after
  connecting while the controller reported 1/17, and still 1/17 0.5 s after a
  successful `ResetAllError()`. Recovery therefore polls **both** sources for
  up to 2 s and requires both to read 0/0.
- `abnomalStop` stays 1 after a successful reset (main/sub = 0). It is shown in
  the UI but is not a fault by itself; with a main/sub code the code decides.
- SDK code 29 means "ServoJ joint overrun" (joint value out of range), not a
  joint-speed overspeed. The controller's speed faults are main 1 / sub 23–28.

## 2. Classification

`classifyRobot()` (`fairino_cartesian_servo/src/fault_catalog.cpp`) builds one
fault from all inputs and keeps the strictest policy, then the highest
severity:

1. status missing/stale (> 0.5 s), SDK communication not normal, or FAIRINO
   feedback older than 100 ms → **LOCKOUT** (robot state unknown);
2. E-stop / button-box stop, safety stop SI0/SI1, safety door, safety plane,
   abnormal safety data, power-cycle request, E-stop button communication,
   drive alarm/communication fault → **LOCKOUT**;
3. controller main/sub code → catalog row (unknown codes: raw values, FAULT,
   no guessed procedure);
4. collision, soft-limit error, control-box error → **FAULT** (operator);
5. robot disabled without a main/sub code → **FAULT** (operator, Recover enables);
6. non-zero ServoJ return code → SDK catalog row;
7. hardware discontinuity guard / non-finite command, and ServoJ streaming
   disarmed for any other reason (e.g. an external `/fairino_robot/stop`) →
   **FAULT** (operator). In ACTIVE and HOLD any streaming state other than
   `ARMED` is a FAULT, so the robot never resumes by itself when the hardware
   re-arms; in DISABLED the transient "recovery in progress" of a hardware
   recovery is not counted;
8. servo-side faults (joint state watchdog, tracking error, joint outside the
   safe range, OInK failure, invalid OInK result, controller not active,
   synchronization or switch failure) → **FAULT** (operator).

Policies:

| Policy | State | Who may start recovery |
|---|---|---|
| `auto_recover` (whitelist) | FAULT | automatically (when `auto_recovery` is true) or the operator with Recover |
| `operator_reset` | FAULT | operator only: Recover (R1, incl. `RobotEnable`) for known faults, Reset Fault (R2) for any |
| `lockout` | LOCKOUT | nobody automatically; operator Reset Fault only after the cause is gone |

Whitelist (automatic recovery allowed): controller 1/23–28 (joint command speed
limit), 1/37 (unreachable point), 1/38 (unreachable point, singular pose), 10/1
(singular pose); SDK 28 (IK failed), 29 (ServoJ joint overrun), 38 (singular
pose), 112 (pose unreachable). These are resettable faults caused by the
command stream itself, after which the controller has already stopped the
robot. Physical events (collision, limits, IO, drives) always need an operator.

Never automatically recovered (requirement → implementation):

| Requirement | Implementation |
|---|---|
| E-stop | `emergency_stop` signal, SDK 31/186 → LOCKOUT |
| Safety stop | `safety_stop` signal, SDK 99 → LOCKOUT |
| Safety door / safety plane | `safety_door`, `safety_plane` signals → LOCKOUT |
| `ERR_NON_RESSETTABLE_FAULT` | SDK 30 → LOCKOUT |
| Power/driver fault not resettable by software | main 2, 5, 6, 9, 11; 1/20, 1/29, 1/30, 1/82; `driver_fault`, `alarm_reboot` → LOCKOUT |
| Lost or invalid safety status | `status_lost`, `safety_data_abnormal` → LOCKOUT |
| Joint position outside the safe range | servo joint-limit check, main 3, SDK 32/171 → FAULT (operator); automatic recovery is refused while any joint is outside its limits |
| Repeated overspeed / repeated fault | same fault signature more than `max_recovery_attempts` (2) times in 120 s → LOCKOUT |
| Communication problem while the robot state is unknown | `status_lost`, SDK −2/−3/−15/−16 → LOCKOUT |
| Unknown code | raw main/sub shown, FAULT, Recover refused, only Reset Fault |

## 3. State machine

```text
            enable/set_mode                  fault (any)            whitelisted + auto
DISABLED ──────────────────► SYNCING ──► ACTIVE ◄──► HOLD ───────► FAULT ─────────────► RECOVERING
   ▲   ◄────── disable ─────────────────────┴─────────┘             │  ▲     Recover /        │
   │                                                                  │  └── Reset Fault ─────┤
   │                       lockout condition (from any state)         ▼                       ▼
   └──── disable (state kept) ◄──────────────────────────────── LOCKOUT ◄── repeated / failed  HOLD (servo mode)
                                                                   │     recovery             DISABLED (plan mode)
                                                                   └── Reset Fault once the cause is gone
```

| State | Motion | Entered when | Left by |
|---|---|---|---|
| DISABLED | trajectory controller (plan mode); planning and observation allowed | startup, Disable, recovery in plan mode | Enable / set_mode, fault |
| SYNCING | none (command = measured) | Enable from DISABLED | switch done → ACTIVE/HOLD; failure → FAULT |
| ACTIVE | servo tracks the gizmo | Enable, resume from HOLD | Hold, Disable, target stale (→ HOLD), fault |
| HOLD | braking ramp from the last velocity (540 deg/s²), then the last command is repeated; OInK velocity reset | operator Hold, target stale, recovery complete | Enable (resume with target reset), Disable, fault |
| RECOVERING | none (streaming disarmed during the hardware steps) | auto recovery, Recover, Reset Fault | success → HOLD/DISABLED; failure → FAULT/LOCKOUT |
| FAULT | none (controller stopped / hold) | operator-class fault | Recover, Reset Fault, auto recovery (whitelist) |
| LOCKOUT | none; hardware stop requested (`ServoMoveEnd`, `StopMotion`) | lockout-class fault, repeated or failed recovery | Reset Fault after the cause is gone (manual) |

Every transition, operator request, controller switch and recovery step is an
`AuditEvent` on `/fairino_servo/events` or `/fairino_robot/events` (latched,
last 200) and a log line starting with `AUDIT`.

## 4. Recovery sequences

Responsibility: the servo node decides **whether** a recovery may run (policy,
attempts, cooldown); the hardware plugin executes the FAIRINO steps (the only
SDK owner); the servo re-synchronizes command and target.

| # | Step | Executed by |
|---|---|---|
| 1 | ACTIVE → HOLD/FAULT, no new targets; the command decelerates at 540 deg/s² instead of freezing | servo |
| 2 | → RECOVERING; Viser targets ignored until re-acquired | servo |
| 3 | streaming disarmed | hardware `/fairino_robot/recover` |
| 4 | `ServoMoveEnd(1)` (UDP) | hardware |
| 5 | `StopMotion()` | hardware |
| 6 | wait for standstill: motion state ≠ running and all joint speeds < 0.5 deg/s (≤ 3 s) | hardware |
| 7 | `ResetAllError()` | hardware |
| 8 | poll `GetRobotErrorCode()` and the realtime main/sub codes until both are 0/0 (≤ 2 s) | hardware |
| 9 | if the robot is disabled: `RobotEnable(1)` only for Recover/auto (not for Reset Fault), wait for enabled (≤ 3 s) | hardware |
| 10 | no safety signal may remain | hardware |
| 11 | `ServoMoveStart(1)` (UDP), streaming → WAIT_SYNC | hardware |
| 12 | servo mode: `q_command = q_measured`, `target = FK(q_measured)`, filter, rate limiter and OInK/acceleration state reset; plan mode: trajectory controller restarted so it holds the measured position | servo |
| 13 | hardware arms only when the command equals the measured position (2 mrad) and has not changed for 20 cycles | hardware |
| 14 | streaming `ARMED` and no fault in a status received **after** step 11 (a cached status from before the recovery is not accepted) → HOLD (servo mode) or DISABLED with the trajectory controller holding the measured position (plan mode) | servo |
| 15 | operator presses Enable Servo to resume | operator |

Any failed step leaves the robot stopped and the servo in FAULT (or LOCKOUT
when the attempt limit is reached or a lockout signal appears; a hardware stop
is then requested, and an audit event records it if `/fairino_robot/stop` is
unavailable). An exception in the operation worker becomes a FAULT but never
replaces a LOCKOUT. The response of `/fairino_robot/recover` lists every SDK
call with its return code. A hardware deactivation or error while a recovery
runs aborts it without re-arming (stop generation). The Recover and Reset
Fault services wait up to 60 s; when the result is not known by then they
answer `success: false` ("still running") instead of reporting success.

Limits (servo parameters): `max_recovery_attempts: 2` per fault signature in
`recovery_attempt_window: 120 s`, `recovery_cooldown: 3 s` before an automatic
attempt, a failed attempt counts as an attempt, Reset Fault clears the counters.

## 5. Operator actions, WebApp and power cycle

In the Viser UI both operator procedures are behind one **Recover** button:
FAULT → `/fairino_servo/recover` (R1), falling back to
`/fairino_servo/reset_fault` (R2) when R1 is refused for an unknown code;
LOCKOUT → R2. The servo-side rules below are unchanged.

| Situation | Action |
|---|---|
| FAULT, whitelisted, auto recovery off | inspect, press **Recover** |
| FAULT, operator class (collision, soft limit, IO, unknown code…) | remove the cause (WebApp alarm text, drag/jog joints back), then **Recover** (known faults, enables the robot) or **Reset Fault** (acknowledge) |
| LOCKOUT, E-stop / safety stop / door / plane | make the cell safe, release/close the input on the robot side, then **Reset Fault** (refused while the signal is active) |
| LOCKOUT, repeated fault | find the root cause (e.g. lower `velocity_scale`), then **Reset Fault** |
| LOCKOUT, non-resettable fault (SDK 30/31, main 2/5/6/9/11, 1/20, 1/29, 1/30, `alarm_reboot`) | power-cycle the control box per FAIRINO, then restart the stack |
| FAULT "hardware ServoJ streaming disarmed: stop requested" (someone called `/fairino_robot/stop`) | make sure the stop was intended to end, then **Reset Fault** (re-arms streaming, ends in HOLD or DISABLED) |
| LOCKOUT, communication lost or hardware deactivated (`/fairino_robot/status` missing) | restore the network/controller; when `ros2 control list_hardware_components` shows `FairinoSystem` not active: `ros2 control set_hardware_component_state FairinoSystem active`, then **Reset Fault** (refused while the status is missing) and **Disable Servo** (plan mode) |
| Collision sensitivity questions | never change it from this software; see [fairino_speed_configuration.md](fairino_speed_configuration.md) §6 |

The WebApp is needed to read the alarm text of a code, to drag/jog a joint back
inside its soft limits, to inspect or change safety configuration (safety
speed, reduced mode, collision level) and to clear conditions that the SDK
cannot reset.

## 6. Test results

Executed on 2026-10-14 against FAIRINO SimMachine 3.9.9 (FR20-V1-001(V6.0),
controller V3.9.39-LX) and with the deterministic state-machine test
`fairino_servo_state_machine_test.py` (fake `/fairino_robot` on mock
hardware, 15/15 checks passed, rerun after the review fixes). Details: [simmachine_test_plan.md](simmachine_test_plan.md).

| Case | Result |
|---|---|
| Real FAIRINO fault 1/23 during ServoJ | classified `auto_recover`, automatic recovery 0.06 s, HOLD, operator resume worked. Observed in 2 of ~32 attempts before the per-point step limit; since that limit the fault injection test no longer provokes it (3 × 20 s at 5 % global speed, 2 sessions) |
| External `/fairino_robot/stop` in HOLD | FAULT "streaming disarmed: stop requested"; Reset Fault → hardware recovery → HOLD, streaming ARMED in 0.10–0.11 s; resume stays ACTIVE |
| Manual `/fairino_robot/recover` in plan mode | streaming re-armed, servo stays DISABLED (no spurious FAULT) |
| Hardware deactivated (`set_hardware_component_state … inactive`) | LOCKOUT "robot state unknown"; Reset Fault refused while the status is missing; after re-activation WAIT_SYNC → ARMED, 0 guard trips, Reset Fault → DISABLED, Enable works |
| 6 deactivate/activate cycles in a row | no crash after the SDK-instance fix (before: SIGSEGV in the SDK CNDE thread on the 2nd cycle), ARMED every time |
| Reset Fault after a servo tracking fault | 0.07 s, all hardware steps code 0, HOLD |
| Manual `/fairino_robot/recover` | 0.06–0.22 s, both code sources 0/0, WAIT_SYNC → ARMED without guard trip |
| Plan-mode recovery with a stale trajectory-controller hold | hardware refused to arm (correct); fixed with `set_last_command_interface_value_as_state_on_activation: false` |
| Communication loss (SimMachine paused 1.5 s) | `read()` ERROR after 0.5 s, servo LOCKOUT; hardware reactivation + Reset Fault + Disable restored plan mode |
| Same whitelisted fault 3× | LOCKOUT after 2 attempts, hardware stop requested (mock S2) |
| E-stop | LOCKOUT; Reset Fault refused while pressed, accepted after release (mock S4) |
| Non-resettable 1/30 | LOCKOUT (mock S5) |
| Unknown 42/7 | FAULT, raw codes, no automatic recovery, Recover refused, Reset Fault accepted (mock S6) |
| Collision | FAULT, no automatic recovery, operator Recover → HOLD (mock S7) |
| Safety door | LOCKOUT (mock S8) |
| Status lost | LOCKOUT (mock S9, SimMachine pause) |
| ServoJ code 29 | automatic recovery → HOLD (mock S10) |
| Robot disabled | FAULT, Recover enables (mock S11) |
| Recovery that keeps failing | LOCKOUT after the second failure (mock S12) |

Nothing has been tested on a real FR20 yet ("tested on real robot" = not tested).

## 7. Fault code catalog

Sources: [FAIRINO manual 3.9.9, Appendix 1 (motion controller main/sub codes)](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html),
[FAIRINO SDK error code table 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html),
`fairino_hardware_v3_9_9/libfairino/include/robot_error.h` and `robot_types.h`.
The local `ErrorCode_lnstruction.pdf` only covers the TCP command interface
(−1001…−1103), which this stack does not use.

### Controller main/sub fault codes

| code | subcode | description | severity | automatic recovery allowed | recovery sequence | manual/WebApp action | source | tested in SimMachine | tested on real robot |
|---:|---|---|---|---|---|---|---|---|---|
| 1 | 1 | Joint command point error | error | no | R2 | Check the commanded joint target, then Reset Fault. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 1 | 2-15 | Linear/arc/circle/helix/TPD/spline command point or tool mismatch errors (sub 2-15) | error | no | R2 | Planned-motion command error; check the program/tool, then Reset Fault. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 1 | 17-19 | PTP/TPD/LIN-ARC joint command limit exceeded | error | no | R2 | Target exceeds joint limits; move the target inside the limits, then Reset Fault. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | observed 1/17 (SDK MoveJ beyond the J3 limit); ResetAllError cleared it | not tested |
| 1 | 20 | Cartesian space command overspeed, not resettable (FAIRINO: not resettable) | critical | no | none (LOCKOUT) | Not resettable: inspect the motion, restart the controller per FAIRINO, review speed limits. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 1 | 21 | Joint space torque command limit exceeded | error | no | R2 | Check payload/dynamics settings in the WebApp, then Reset Fault. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 1 | 22 | JOG joint command limit exceeded | warning | no | R2 | Jog back inside the limits, then Reset Fault. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 1 | 23-28 | Axis 1-6 joint space command speed limit exceeded (sub 23 = J1 ... 28 = J6) | error | yes | R1 | Whitelisted: controlled recovery to HOLD. If it repeats, lower OInK/Cartesian limits or the target speed. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | observed 1/23 during ServoJ (simulator stall + step); auto recovery 0.06 s -> HOLD | not tested |
| 1 | 29 | Joint feedback speed limit exceeded, not resettable (FAIRINO: not resettable) | critical | no | none (LOCKOUT) | Not resettable: stop, inspect the robot, restart the controller per FAIRINO. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 1 | 30 | Joint command and feedback deviation too large, not resettable, requires restart (FAIRINO: not resettable) | critical | no | none (LOCKOUT) | Power cycle / restart the controller; check payload, collisions and command continuity. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not reproducible; mock state-machine test S5 -> LOCKOUT | not tested |
| 1 | 31 | DMP target point error (including tool mismatch) | error | no | R2 | Check the DMP target, Reset Fault. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 1 | 33-34 | Joint configuration changed between commands (singular pose on the path) | error | no | R2 | Use a joint-space motion or another target, then Reset Fault. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 1 | 35-36 | LIN command joint speed / adaptive speed threshold exceeded | error | no | R2 | Planned LIN motion too fast; reduce speed, then Reset Fault. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 1 | 37 | Unreachable point in trajectory | error | yes | R1 | Whitelisted: recovery to HOLD; move the gizmo to a reachable pose before resuming. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 1 | 38 | Unreachable point in trajectory - singular pose | error | yes | R1 | Whitelisted: recovery to HOLD; move the gizmo away from the singularity before resuming. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 1 | 49-56 | Arc/weave command sequence or weaving trajectory errors (sub 49-56) | error | no | R2 | Welding/weaving program error; fix the program, then Reset Fault. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 1 | 65-66 | Laser sensor command deviation / seam tracking interrupted | error | no | R2 | Check the laser sensor, then Reset Fault. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 1 | 81 | External axis command speed limit exceeded | error | no | R2 | Check the external axis motion, then Reset Fault. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 1 | 82 | External axis command and feedback deviation too large, not resettable (FAIRINO: not resettable) | critical | no | none (LOCKOUT) | Home the external axis or restart the controller per FAIRINO. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 1 | 83-84 | Extended peripheral (external axis/IO) communication error or packet loss | error | no | R2 | Check peripheral wiring/communication, then Reset Fault. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 1 | 97 | Conveyor tracking start/reference pose change too large | error | no | R2 | Check conveyor tracking setup, then Reset Fault. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 1 | 113-118 | Constant force control exceeded max adjustment (X/Y/Z/RX/RY/RZ) | error | no | R2 | Check the force control task, then Reset Fault. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 1 | 119-123 | External sensor data error or force-guided motion failed (sub 119-123) | error | no | R2 | Check the sensor/force task, then Reset Fault. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 1 | 129-130 | Max torque record points exceeded / speed switching error | error | no | R2 | Check the program, then Reset Fault. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 1 | 147-149 | Focus following error / attitude speed limit / joint status word abnormal | error | no | R2 | Inspect the reported condition, then Reset Fault. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 2 | 1-6 | Axis 1-6 drive fault, not resettable (FAIRINO: not resettable) | critical | no | none (LOCKOUT) | Drive fault: inspect drives/cabling, power cycle per FAIRINO; WebApp alarm log. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 3 | 1-6 | Axis 1-6 soft limit exceeded | error | no | R2 | Joint outside the safe range: no automatic recovery. Reset, then jog/drag the joint back inside the soft limits in the WebApp. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 4 | 1-6 | Axis 1-6 collision fault | error | no | R2 | Collision: inspect the cell and people first. Never lower collision sensitivity to clear it. Then Recover. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not reproducible; mock test S7 (collision signal) -> FAULT, operator Recover | not tested |
| 4 | 7 | End effector collision fault | error | no | R2 | Collision: inspect the cell and tool first, then Recover. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 5 | 1 | Active slave count error, not resettable (FAIRINO: not resettable) | critical | no | none (LOCKOUT) | EtherCAT slave count wrong: power cycle and check the bus. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 6 | 1-6 | Slave offline/state/configuration/initialization/mailbox error, not resettable (FAIRINO: not resettable) | critical | no | none (LOCKOUT) | EtherCAT slave fault: power cycle, check the bus and configuration. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 7 | 1-14 | IO error (channel, value, WaitDI/AI/Axle/Aux timeouts, arc/search/conveyor timeouts) | warning | no | R2 | Check the IO configuration/signals, then Reset Fault. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 8 | 1 | Gripper motion timeout error | warning | no | R2 | Check the gripper, then Reset Fault. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 9 | 1-14 | Configuration file or robot model error, not resettable (FAIRINO: not resettable) | critical | no | none (LOCKOUT) | Controller configuration invalid: fix in the WebApp (robot model / config files), restart. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 10 | 1 | Singular pose | error | yes | R1 | Whitelisted: recovery to HOLD; move the gizmo away from the singularity before resuming. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 11 | 1-6 | Axis 1-6 drive communication error, not resettable (FAIRINO: not resettable) | critical | no | none (LOCKOUT) | Drive communication fault: power cycle, check cabling per FAIRINO. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 12 | 1-4 | External axis 1-4 soft limit exceeded | error | no | R2 | Move the external axis inside its limits, then Reset Fault. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| 13 | 1-17 | Parameter setting error (tool/workpiece numbers, collision level, load, filters, IO ranges) | error | no | R2 | Correct the parameter in the WebApp, then Reset Fault. | [FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html) | not tested | not tested |
| other | any | Unknown code: shown raw, FAULT, no guessed recovery | error | no | R2 (Reset Fault only) | Read the WebApp alarm text and manual | - | mock test S6 | not tested |

### SDK return codes (ServoJ and API calls)

| code | subcode | description | severity | automatic recovery allowed | recovery sequence | manual/WebApp action | source | tested in SimMachine | tested on real robot |
|---:|---|---|---|---|---|---|---|---|---|
| 0 | - | ERR_SUCCESS | none | - | - | - | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | yes (every ServoJ call) | not tested |
| -16 | - | Socket receive failed | critical | no | none (LOCKOUT) | Communication with the controller lost; robot state unknown. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| -15 | - | Socket send failed | critical | no | none (LOCKOUT) | Communication with the controller lost; robot state unknown. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| -4 | - | XML-RPC interface execution failed | error | no | R2 | Check the controller log; Reset Fault after inspection. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| -3 | - | XML-RPC communication failed | critical | no | none (LOCKOUT) | Check network and controller IP; robot state unknown. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| -2 | - | Abnormal communication with the controller | critical | no | none (LOCKOUT) | Check the connection; robot state unknown, no automatic recovery. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| -1 | - | Other error | error | no | R2 | Check the controller log. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| 3 | - | Interface parameter count mismatch | error | no | R2 | Software error in the caller. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| 4 | - | Interface parameter value out of range | error | no | R2 | Software error in the caller (command out of range). | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| 14 | - | Interface execution failed | error | no | R2 | Controller rejected the command; the WebApp/status main-sub code gives the cause. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | observed: calls rejected with 14 while 1/17 was active | not tested |
| 18 | - | Robot program is running | warning | no | R2 | Stop the WebApp program before servoing. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| 28 | - | Inverse kinematics calculation failed | error | yes | R1 | Whitelisted: recovery to HOLD; choose a reachable pose. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| 29 | - | ServoJ joint overrun (joint value out of range) | error | yes | R1 | Whitelisted only while measured joints are inside their limits: recovery to HOLD. Not a joint-speed overspeed code. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not reproducible; mock test S10 -> auto recovery -> HOLD | not tested |
| 30 | - | Non-resettable fault, power off and restart the control box | critical | no | none (LOCKOUT) | Power cycle the control box. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| 31 | - | Emergency stop released, power off and restart the control box | critical | no | none (LOCKOUT) | Power cycle the control box. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| 32 | - | Joint over limit | error | no | R2 | Drag the joint back inside the soft limits, then Reset Fault. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | observed: MoveJ target beyond the J3 limit returned 32 | not tested |
| 38 | - | Singular pose, calculation failed | error | yes | R1 | Whitelisted: recovery to HOLD; change the pose. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| 40 | - | Speed percentage exceeds limit | warning | no | R2 | Use a speed within 0-100 %. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| 42 | - | Posture changes too much | error | no | R2 | Insert an intermediate pose. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| 44 | - | Robot attitude angle exceeds the limit | error | no | R2 | Check the target attitude. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| 64 | - | Not added to the instruction queue | error | no | R2 | Check the controller log. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| 75 | - | Channel error | warning | no | R2 | IO number out of range; check the IO configuration. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| 99 | - | Safety stop triggered | critical | no | none (LOCKOUT) | Check the safety stop inputs; manual intervention required. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not reproducible; safety signals covered by mock tests S4/S8 | not tested |
| 101 | - | Robot is not enabled | warning | no | R2 | Recover (enables the robot) after checking the cell. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| 112 | - | Target pose cannot be reached | error | yes | R1 | Whitelisted: recovery to HOLD; choose a reachable pose. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| 171 | - | Joint soft limit exceeded, soft limit protection failed to start | error | no | R2 | Move all joints inside the soft limits. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| 182 | - | Given physical speed exceeds limit | error | no | R2 | Reduce the commanded speed. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| 183 | - | Given physical acceleration exceeds limit | error | no | R2 | Reduce the commanded acceleration. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| 185 | - | Fault signal triggered, motion stopped | critical | no | none (LOCKOUT) | Check the fault input signal. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| 186 | - | Emergency stop signal triggered, motion stopped | critical | no | none (LOCKOUT) | Release E-stop only when safe; manual reset. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| 187 | - | Current collision level too low | warning | no | R2 | Collision level is a safety setting: change only in the WebApp. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| 204 | - | Command queue is full | error | no | R2 | Command rate too high; check cmdT and the stream. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| 207 | - | Automatic mode safety stop requires teach pendant three-position enable | critical | no | none (LOCKOUT) | Use the teach pendant enabling device. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| 211 | - | Safety parameter operation password error | warning | no | R2 | Safety configuration only: check the password. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| 212 | - | Safety operation password verification required first | warning | no | R2 | Safety configuration only: SafetyOPPasswordCheck first. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| 215 | - | Safety configuration parameter setting failed | error | no | R2 | Use the WebApp Safety page. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| 216 | - | Safety configuration checksum update failed | error | no | R2 | Use the WebApp Safety page. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| 220 | - | Level 3 reduced mode has been triggered | critical | no | none (LOCKOUT) | Check the reduced-mode safety inputs. | [FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), `libfairino/include/robot_error.h` | not tested | not tested |
| other | - | Unknown code: shown raw, FAULT | error | no | R2 (Reset Fault only) | Check the SDK table | - | - | - |

### Safety and status signals

| signal | description | severity | automatic recovery allowed | recovery sequence | manual/WebApp action | source | tested in SimMachine | tested on real robot |
|---|---|---|---|---|---|---|---|---|
| `status_lost` | Robot status missing/stale or SDK communication not normal (robot state unknown) | critical | no | none (LOCKOUT) | Restore communication; the hardware must be restarted before servoing. | `ROBOT_STATE_PKG` in `libfairino/include/robot_types.h` (SDK 3.9.9) | observed: SimMachine paused 1.5 s -> read() ERROR after 0.5 s, LOCKOUT; manual hardware reactivation + Reset Fault (also mock S9) | not tested |
| `emergency_stop` | Emergency stop pressed (EmergencyStop / button box) | critical | no | none (LOCKOUT) | Release E-stop only when safe; Reset Fault; power cycle if SDK code 31 / alarm_reboot_robot. | `ROBOT_STATE_PKG` in `libfairino/include/robot_types.h` (SDK 3.9.9) | mock test S4: LOCKOUT, reset refused while pressed | not tested |
| `safety_stop` | Safety stop SI0/SI1 active | critical | no | none (LOCKOUT) | Clear the safety stop input, then Reset Fault. | `ROBOT_STATE_PKG` in `libfairino/include/robot_types.h` (SDK 3.9.9) | not tested | not tested |
| `safety_door` | Safety door open | critical | no | none (LOCKOUT) | Close the door, then Reset Fault. | `ROBOT_STATE_PKG` in `libfairino/include/robot_types.h` (SDK 3.9.9) | mock test S8 -> LOCKOUT | not tested |
| `safety_plane` | Safety plane (safety wall) entered | critical | no | none (LOCKOUT) | Move out of the safety plane with the WebApp/drag, then Reset Fault. | `ROBOT_STATE_PKG` in `libfairino/include/robot_types.h` (SDK 3.9.9) | not tested | not tested |
| `safety_data_abnormal` | Safety data state abnormal | critical | no | none (LOCKOUT) | Safety configuration invalid: check WebApp Safety pages/checksum. | `ROBOT_STATE_PKG` in `libfairino/include/robot_types.h` (SDK 3.9.9) | not tested | not tested |
| `alarm_reboot` | Controller requests power cycle (E-stop release / joint communication) | critical | no | none (LOCKOUT) | Power cycle the control box. | `ROBOT_STATE_PKG` in `libfairino/include/robot_types.h` (SDK 3.9.9) | not tested | not tested |
| `alarm_check_estop` | E-stop button communication abnormal | critical | no | none (LOCKOUT) | Check the E-stop button and cabling. | `ROBOT_STATE_PKG` in `libfairino/include/robot_types.h` (SDK 3.9.9) | not tested | not tested |
| `driver_fault` | Drive alarm, drive communication error | critical | no | none (LOCKOUT) | Inspect drives; power cycle per FAIRINO. | `ROBOT_STATE_PKG` in `libfairino/include/robot_types.h` (SDK 3.9.9) | not tested | not tested |
| `ctrl_box_error` | Control box error (ctrlBoxError) reported | error | no | R2 | Read the control box error in the WebApp; Reset Fault after the cause is cleared. | `ROBOT_STATE_PKG` in `libfairino/include/robot_types.h` (SDK 3.9.9) | not tested | not tested |
| `collision` | Collision detected (collisionState) | error | no | R2 | Inspect the cell; never change collision sensitivity automatically; then Recover. | `ROBOT_STATE_PKG` in `libfairino/include/robot_types.h` (SDK 3.9.9) | mock test S7 | not tested |
| `out_soft_limit` | Joint outside the soft limit | error | no | R2 | Jog/drag back inside the limits, then Reset Fault. | `ROBOT_STATE_PKG` in `libfairino/include/robot_types.h` (SDK 3.9.9) | not tested | not tested |
| `abnormal_stop` | Abnormal stop flag (abnomalStop): informational, shown in the UI only | info | no | - | Stays 1 after a successful ResetAllError() on SimMachine 3.9.9; the main/sub code decides. | `ROBOT_STATE_PKG` in `libfairino/include/robot_types.h` (SDK 3.9.9) | yes: set with main=1 sub=23 and still set after reset (2026-10-14) | not tested |
| `robot_disabled` | Robot disabled (rbtEnableState = 0) without a main/sub code | warning | no | R2 | Recover enables the robot after a check. With a main/sub code the code decides. | `ROBOT_STATE_PKG` in `libfairino/include/robot_types.h` (SDK 3.9.9) | mock test S11: Recover enables | not tested |
| `command_rejected` | Hardware stopped ServoJ: discontinuity guard or non-finite command | error | no | R2 | Software fault in the command stream; Reset Fault re-synchronizes command and target. | `ROBOT_STATE_PKG` in `libfairino/include/robot_types.h` (SDK 3.9.9) | observed: jump guard trip under CPU load (before the gap-aware guard) | not tested |

### Recovery sequences

- R1 (automatic, whitelisted): ACTIVE->HOLD, stop new targets, ServoMoveEnd(UDP), StopMotion(), wait standstill, ResetAllError(), GetRobotErrorCode()==0/0, RobotEnable(1) if disabled, ServoMoveStart(UDP), q_cmd=q_measured, target=FK(q_measured), reset filter/OInK, HOLD; operator resumes. Max 2 attempts, 3 s cooldown, repeat -> LOCKOUT.
- R2 (operator): same steps as R1, started only by the operator. Recover = R1 incl. RobotEnable(1); Reset Fault = acknowledgement without RobotEnable.
- LOCKOUT (no software recovery): remove the cause manually, then the operator may Reset Fault once no lockout signal is active (power cycle where stated).
