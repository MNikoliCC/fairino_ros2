# FAIRINO FR20 Viser Cartesian servo — documentation

| Document | For | Content |
|---|---|---|
| [viser_operator_guide.md](viser_operator_guide.md) | operators | start the stack, Viser UI, servo/plan mode, Hold, Recover, Reset Fault, reading the status |
| [viser_servo_architecture.md](viser_servo_architecture.md) | developers | components, single SDK owner, CNDE, ServoJ protections, threads, rates, interfaces, controller switching, frames, limits |
| [fairino_fault_recovery.md](fairino_fault_recovery.md) | developers, safety | fault sources, classification, state machine, recovery sequences, WebApp/power-cycle cases, complete code catalog |
| [fairino_speed_configuration.md](fairino_speed_configuration.md) | developers, safety | the four speed layers, API calls, measured effects, WebApp procedures, collision sensitivity |
| [trajectory_recording_and_paths.md](trajectory_recording_and_paths.md) | operators, developers | recording, replay, path editor (draw from the robot TCP, curve waypoints, time per segment, automatic planning), planning pipeline, file format, measurements |
| [servo_benchmark.md](servo_benchmark.md) | developers | environment, method, timing and tracking results, 1 kHz decision, self-collision barrier |
| [simmachine_test_plan.md](simmachine_test_plan.md) | test/commissioning | executed tests and results, open items, real-robot staged plan |

Quick start:

```bash
cd ~/colcon_ws/src/fairino_ros2/docker && ./simmachine.sh up
docker compose exec ros2 bash
# SimMachine (ServoJ 2 ms, see servo_benchmark.md §7); real robot: without the two servoj_* arguments
ros2 launch fairinoplan_bringup fairino20_viser_servo.launch.py use_mock_hardware:=false servoj_cmd_period:=0.002 servoj_max_queue:=1
# browser: http://localhost:8000
```
