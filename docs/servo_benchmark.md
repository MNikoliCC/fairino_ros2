# Servo benchmark

Timing and tracking measurements of the Cartesian servo, the decision between
1 kHz OInK and 250 Hz OInK with 1 kHz interpolation, and the effect of the
self-collision barrier. All numbers below were measured on 2026-10-14/15;
the SimMachine rows were re-measured with the final code (review fixes,
flow-control corrections in §3).

## 1. Environment

| Item | Value |
|---|---|
| Host CPU | AMD Ryzen 7 5800X, 8 cores / 16 threads, governor `powersave` |
| Host OS / kernel | Ubuntu 24.04, `7.0.0-34-generic` `PREEMPT_DYNAMIC` (not PREEMPT_RT) |
| Container | ROS 2 Humble (`fairino-ros2:humble`), Docker 29.6.1, `SYS_NICE`, `rtprio 99`, `memlock -1`, `ROS_DOMAIN_ID=42` |
| ros2_control | 2.54.3, `update_rate` 1000 Hz, `ros2_control_node` under `chrt -f 40`, CM loop SCHED_FIFO 50 |
| Servo loop | C++ `fairino_cartesian_servo`, SCHED_FIFO 80, `clock_nanosleep` absolute deadlines |
| RoboPlan | commit `ac29aa9`, built **Release** (OInK with ProxQP) |
| Robot side | mock_components/GenericSystem, or FAIRINO SimMachine 3.9.9 container (FR20-V1-001(V6.0), controller V3.9.39-LX) on the same host |
| Other load | other containers on the host (incl. an unrelated Viser process at ~70 % of a core) |

The host is a development PC, not a real-time system. Results on the target
computer (and on a Raspberry Pi 5) must be measured with the same tool.

## 2. Method

`ros2 run fairinoplan_bringup fairino_servo_benchmark.py` enables the servo,
waits for the reset pose, calls `/fairino_servo/reset_timing`, publishes a
target at 100 Hz along a circle in the base Y–Z plane (radius r, period T,
plus a ±0.2 rad rotation about the tool axis, 1 s ramp-in), then reads
`/fairino_servo/timing_report` (exact statistics over the run, ring buffer of
65 536 samples per stage) and the hardware counters, and puts the servo in
HOLD. No cycle is logged individually.

| Stage | Measured | Budget for overruns |
|---|---|---|
| `filter` | `SE3LowPassFilter::update` | solver period |
| `rate_limiter` | Cartesian linear/angular limiter | solver period |
| `oink` | OInK solve (task-only solve for the acceleration limit + constrained solve [+ barrier enforcement]) | solver period |
| `solver_total` | filter + rate limiter + OInK + integration + validation | solver period |
| `publish` | building and publishing the 1 kHz command | 1 ms |
| `cycle` | whole servo cycle work while ACTIVE | 1 ms |
| `period` | interval between servo loop wake-ups | 1.5 ms |

Hardware counters (from `/fairino_robot/status`): ServoJ calls/errors/return
codes, discontinuity-guard trips, UDP command state, joint feedback rate,
`write()` period, ServoJ flow-control skips, re-baselines, drift probes and
drift corrections.

Trajectories: *slow* r = 0.05 m, T = 4 s (0.079 m/s); *fast* r = 0.15 m,
T = 2 s (0.47 m/s). Runs start from the home pose (0, −90, 90, −90, −90, 0)°.

## 3. Results

Timing in ms as mean / p95 / p99 / max (overruns). Duration 60 s unless noted.

| Run | OInK | barrier pairs | `oink` | `cycle` | `period` | `publish` |
|---|---:|---:|---|---|---|---|
| mock, slow | 1 kHz | 0 | 0.065 / 0.098 / 0.108 / 0.150 (0) | 0.082 / 0.118 / 0.128 / 0.241 (0) | 1.000 / 1.002 / 1.004 / 1.037 (0) | 0.014 / 0.018 / 0.019 / 0.158 (0) |
| mock, slow | 1 kHz | 1 | 0.078 / 0.117 / 0.131 / 0.223 (0) | 0.094 / 0.137 / 0.152 / 0.246 (0) | 1.000 / 1.002 / 1.004 / 1.034 (0) | 0.014 / 0.018 / 0.019 / 0.140 (0) |
| mock, slow | 1 kHz | 3 | 0.084 / 0.126 / 0.143 / 0.231 (0) | 0.101 / 0.146 / 0.164 / 0.261 (0) | 1.000 / 1.002 / 1.004 / 1.038 (0) | 0.014 / 0.018 / 0.019 / 0.079 (0) |
| mock, slow | 1 kHz | 5 | 0.087 / 0.128 / 0.151 / 0.257 (0) | 0.104 / 0.147 / 0.172 / 0.277 (0) | 1.000 / 1.002 / 1.003 / 1.119 (0) | 0.014 / 0.017 / 0.019 / 0.140 (0) |
| SimMachine, slow | 1 kHz | 0 | 0.056 / 0.080 / 0.102 / 0.262 (0) | 0.072 / 0.099 / 0.124 / 0.283 (0) | 1.000 / 1.002 / 1.004 / 1.042 (0) | 0.014 / 0.017 / 0.019 / 0.139 (0) |
| SimMachine, fast, run 1 | 1 kHz | 0 | 0.060 / 0.089 / 0.104 / 0.183 (0) | 0.077 / 0.108 / 0.126 / 0.214 (0) | 1.000 / 1.002 / 1.004 / 1.054 (0) | 0.014 / 0.017 / 0.019 / 0.112 (0) |
| SimMachine, fast, run 2 | 1 kHz | 0 | 0.061 / 0.086 / 0.103 / 7.104 (1) | 0.077 / 0.106 / 0.129 / 7.118 (1) | 1.000 / 1.002 / 1.004 / 8.122 (1) | 0.013 / 0.017 / 0.019 / 0.203 (0) |
| SimMachine, fast, run 3 | 1 kHz | 0 | 0.062 / 0.094 / 0.109 / 3.955 (1) | 0.078 / 0.114 / 0.131 / 3.976 (1) | 1.000 / 1.003 / 1.004 / 4.982 (1) | 0.014 / 0.018 / 0.020 / 0.144 (0) |
| SimMachine, slow | 1 kHz | 5 | 0.088 / 0.127 / 0.150 / 0.335 (0) | 0.104 / 0.147 / 0.172 / 0.360 (0) | 1.000 / 1.002 / 1.004 / 1.128 (0) | 0.014 / 0.018 / 0.019 / 0.205 (0) |
| SimMachine, slow | 250 Hz + interpolation | 0 | 0.067 / 0.089 / 0.115 / 0.228 (0) | 0.029 / 0.090 / 0.112 / 0.251 (0) | 1.000 / 1.002 / 1.004 / 1.048 (0) | 0.012 / 0.016 / 0.018 / 0.138 (0) |
| SimMachine, CPU stress (45 s, earlier flow control) | 1 kHz | 0 | 0.072 / 0.093 / 0.106 / 0.238 (0) | 0.091 / 0.113 / 0.127 / 2.507 (4) | 1.000 / 1.002 / 1.004 / 3.514 (4) | 0.016 / 0.018 / 0.021 / 2.403 (3) |

`filter` and `rate_limiter` ≤ 0.04 ms (max) in every run except fast run 2
(0.17 ms in the same host stall). The 250 Hz run reports `oink` over 15 003
solves; its `cycle` mean is lower because three of four cycles only
interpolate. The mock rows were measured before the review fixes; the braking
ramp runs only outside ACTIVE and the exact acceleration clamp is six
`std::clamp` calls, so they do not change these timings measurably.

The single overrun in fast runs 2 and 3 is one wake-up delayed by 4–8 ms
(`period` max 8.1 ms / 5.0 ms) with the servo thread inside the OInK stage: the
thread did not get the CPU (a ProxQP iteration limit would have faulted the
servo). This is the host (PREEMPT_DYNAMIC kernel, other containers, no
`mlockall`), not the solver; the servo stayed ACTIVE and no hardware guard
tripped. The same stalls appeared earlier only under CPU stress.

Tracking and hardware:

| Run | TCP error RMS / max [mm] | max \|q_cmd − q_meas\| [mrad] | ServoJ calls/s | ServoJ errors | guard trips | flow skips / re-baselines / probes / drift corrections | feedback [Hz] | `write()` p99 / max [ms] |
|---|---|---:|---:|---:|---:|---|---:|---|
| mock, slow, no barrier | 4.7 / 5.6 | 0.4 | – | – | – | – | – | – |
| mock, slow, barrier 5 | 4.7 / 5.6 | 0.2 | – | – | – | – | – | – |
| SimMachine, slow | 7.0 / 8.3 | 8.2 | 834 | 0 | 0 | 9 934 / 0 / 1 / 1 | 250.0 | 1.09 / 1.08 |
| SimMachine, fast, run 1 | 51.1 / 67.6 | 34.2 | 664 | 0 | 0 | 20 146 / 0 / 1 / 1 | 250.0 | 1.06 / 1.06 |
| SimMachine, fast, run 2 | 51.4 / 56.5 | 28.5 | 858 | 0 | 0 | 8 437 / 0 / 0 / 0 | 250.0 | 1.15 / 1.14 |
| SimMachine, fast, run 3 | 51.4 / 67.3 | 44.3 | 760 | 0 | 0 | 14 421 / 0 / 1 / 1 | 250.0 | 1.16 / 1.15 |
| SimMachine, barrier 5 | 7.2 / 8.5 | 8.3 | 695 | 0 | 0 | 18 274 / 0 / 0 / 0 | 250.0 | 1.07 / 1.06 |
| SimMachine, 250 Hz | 6.9 / 8.8 | 8.3 | 884 | 0 | 0 | 6 933 / 0 / 0 / 0 | 250.0 | 1.11 / 1.10 |
| SimMachine, CPU stress (earlier flow control) | 34.4 / 108.4 | 127.5 | 861 | 0 | 0 | 6 246 / 0 / – / – | 250.0 | 1.09 / 1.08 |

All runs stayed ACTIVE for the full 60 s.

The TCP error compares FK of the measured joints with the target published at
that moment, so it contains the intended lag: the SE(3) filter (τ = 50 ms ≈
3.9 mm at 0.079 m/s, ≈ 23 mm at 0.47 m/s) and the robot's following delay
(≈ 30 ms on SimMachine: CNDE 4 ms + ServoJ queue + controller). UDP command
state was 0 and every ServoJ return code 0 in all runs; FAIRINO main/sub codes
stayed 0/0. `/joint_states` ran at 1000 Hz in these runs and at 250 Hz in the
final configuration (equal to the FAIRINO feedback rate).

### Servo parameters of 2026-10-15

Tip frame `flange` (TCP of FAIRINO tool 0, 120 mm ahead of `wrist3_link`),
filter τ 0.03 s (was 0.05 s), Cartesian limit 2.0 m/s (was 1.0 m/s; the
circles stay below 0.5 m/s, so only the filter changes their tracking):

| Run | TCP error RMS / max [mm] | max \|q_cmd − q_meas\| [mrad] | `oink` p99 / max [ms] | overruns | ServoJ calls/s | probes / drift corrections |
|---|---|---:|---|---:|---:|---|
| SimMachine, slow | 5.7 / 6.8 (was 7.0 / 8.3) | 8.7 | 0.103 / 0.159 | 0 | 682 | 1 / 1 |
| SimMachine, fast | 43.2 / 48.2 (was 51.1–51.4) | 34.0 | 0.107 / 0.172 | 0 | 823 | 0 / 0 |

Both runs ACTIVE for 60 s, ServoJ errors 0, guard trips 0, codes 0/0. The
tables above were measured with the previous tip frame and filter.

### SimMachine ServoJ consumption and flow control

SimMachine does not run in real time. It executed 900–950 ServoJ points/s at
rest. Four flow-control problems were found and fixed on it:

| # | Symptom | Cause | Fix |
|---|---|---|---|
| 1 | robot moved 2–4 s after the command | FIFO grew: 1000 points/s sent, fewer consumed | flow control: queue = sent − executed (`servoJCmdNum`) ≤ 8 |
| 2 | after a hardware deactivate/activate: robot permanently 0.3–0.6 s behind the command (0.57 s measured, 84 re-baselines) | `ServoMoveStart` reset the executed counter after the baseline frame; the phantom points made the estimate negative and the real FIFO grew unseen | executed may never exceed sent; such a step re-baselines to an empty queue. Lag after a cycle: 38–42 ms |
| 3 | hidden backlog growing over a session | re-baselining after 40 ms of skips (introduced to escape a dead-locked estimate) fired during normal SimMachine stalls and added up to 9 points each time | re-baseline only after 0.5 s without executed progress |
| 4 | during motion the stream settled at **exactly 250 points/s** (one point per 4 ms feedback frame, executed = sent), robot at a quarter of the commanded speed; fast circle ended in a tracking FAULT in 2 of 4 runs | the estimate drifted upwards (points dropped or merged by the simulator without being counted): the real FIFO was empty while the estimate read "full", so a point was sent only after the next frame showed progress | drift probe: after 100 ms of "full" with ≤ 2 points executed per frame, pause sending for 3 frames; no progress in frames 2–3 means the FIFO is empty and the estimate is corrected, progress means the controller is really slow and nothing changes (at most one probe per 0.5 s) |

With all four: lag 34–42 ms on the slow circle, fast circle ACTIVE in 3 of 3
runs (one drift correction in 2 of them, average 664–858 points/s), no lag
growth after 6 deactivate/activate cycles or after a 5 % global-speed
episode. The previously reported 347–932 points/s "during motion" was partly
this starvation, not only the simulator. When the simulator itself falls far
behind, the robot cannot keep up with fast targets and the servo stops with a
tracking-error FAULT; this remains the intended protection. On a real-time
controller (1000 points/s) a drift probe should never trigger: check
`servoj_flow_skips`, `servoj_flow_probes` and `servoj_flow_drift_corrections`
on the real robot.

Lag was measured by recording `/fairino_servo_controller/commands` and
`/joint_states` during a run and cross-correlating each joint (residual after
the best shift < 0.5 mrad).

### Braking and acceleration limit (mock)

| Event | Joint speed at the event | Result |
|---|---|---|
| ACTIVE → HOLD (before the fix) | – | command frozen within one 1 ms cycle |
| ACTIVE → HOLD | 1.383 rad/s | linear ramp to zero in 147 ms (computed 147 ms), max deceleration 9.42 rad/s² = 540 deg/s² |
| ACTIVE → HOLD | 0.167 rad/s | 19 ms ramp (computed 18 ms) |
| Reset Target while moving (before the fix) | 2.094 rad/s | velocity zeroed in one cycle |
| Reset Target while moving, QP bounds only | 2.094 rad/s | max deceleration 10.30 rad/s² (ProxQP `eps_abs` 1e-6 rad ≈ 1 rad/s² at 1 kHz) |
| Reset Target while moving, exact clamp | 0.391 rad/s | max deceleration 9.42 rad/s² |

### Recovery

| Recovery | Duration |
|---|---|
| automatic recovery of FAIRINO 1/23 on SimMachine | 0.06 s (HOLD) |
| Reset Fault after a servo tracking fault | 0.01–0.07 s |
| Reset Fault after an external `/fairino_robot/stop` (servo request → HOLD and ARMED) | 0.10–0.11 s |
| Reset Fault after a hardware re-activation | 0.01–0.30 s |
| `/fairino_robot/recover` with motion stop | 0.06–0.22 s (standstill wait dominates) |
| deterministic state-machine test (mock) | 15/15 checks, recoveries 0.05 s |

## 4. History

| Configuration | Result | Cause |
|---|---|---|
| Python servo prototype, RoboPlan without build type (‑O0), 1 kHz | OInK mean 0.971 ms, p99 1.395 ms, max 2.598 ms, 4 319 overruns / 13 310 solves | unoptimized library + Python |
| same, 250 Hz | mean 1.080, p99 1.570, max 2.849 ms, 0 overruns / 3 512 | |
| same, barrier 4 pairs | mean 3.514, p99 4.675, max 10.814 ms, 75 overruns / 488 | mesh distances |
| C++ servo, RoboPlan ‑O0, singular start pose (q = 0) | OInK mean 6.5 ms, p99 10.2 ms | ‑O0 ProxQP; j5 = 0 wrist singularity |
| C++ servo, RoboPlan **Release** (current) | OInK mean 0.06–0.09 ms, p99 0.10–0.15 ms | — |
| barrier on vendor meshes (5k–22k triangles) | 17–20 ms per solve, QP "maximum number of iterations" | mesh distance 7–19 ms; forearm–wrist2 at 17 mm < d_min 20 mm |
| barrier on convex hulls, safe-displacement gain 0.01 | fast, but the arm froze | 1/‖J_h‖² weight with ‖J_h‖ ≈ 0 |
| barrier on convex hulls, gain 20, safe-displacement 0, d_min 5 mm (current) | +0.02 ms mean, p99 ≤ 0.15 ms, tracking unchanged; stops exactly at d_min when driven into a collision | — |
| flow control with phantom executed points and 40 ms re-baseline | 0.3–0.6 s permanent following delay after a hardware re-activation | flow-control table in §3, rows 2–3 |
| flow control without drift probe | 250 points/s starvation; fast circle FAULT in 2 of 4 runs | flow-control table in §3, row 4 |

The Docker image now builds RoboPlan with `-DCMAKE_BUILD_TYPE=Release`, and
the workspace entrypoint passes the same flag.

## 5. Decision

**OInK runs at 1 kHz** (`solver_frequency: 1000.0`). The complete servo cycle
has p99 0.12–0.17 ms and max ≤ 0.36 ms against the 1 ms budget, with or
without the 5-pair self-collision barrier. The 250 Hz + 1 kHz interpolation
mode remains available (`solver_frequency:=250.0`) and performed equally well
(tracking RMS 6.9 mm vs 7.0 mm); it is the fallback for slower computers.

The only overruns were single host stalls of the servo thread: 4 in 45 000
cycles while three headless Chrome instances rendered with software WebGL,
and 1 in 60 000 cycles in two of the three fast-circle runs (4–8 ms). The servo
stayed ACTIVE and no hardware protection tripped. For the real robot use a
PREEMPT_RT kernel, isolate a CPU for the servo loop (`cpu_affinity`
parameter), consider locking the servo's memory (`mlockall`, not implemented
yet) and keep browsers and other heavy processes off that computer.

Self-collision barrier (phase 2) is benchmarked and usable at 1 kHz with the
convex-hull model, but stays **off by default** until it has been validated on
the real robot (`self_collision_pairs:=5` to enable).

## 6. Reproduce

```bash
# in the ROS container, stack running (mock or SimMachine)
ros2 run fairinoplan_bringup fairino_servo_benchmark.py --duration 60 --radius 0.05 --period 4 \
    --label "1 kHz, no barrier" --output /tmp/bench.json
# other solver rate / barrier: restart the servo with parameters, e.g.
ros2 launch fairinoplan_bringup fairino20_viser_servo.launch.py use_mock_hardware:=false \
    solver_frequency:=250.0 self_collision_pairs:=5
```
