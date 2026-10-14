#!/usr/bin/env python3

"""Repeatable benchmark of the FAIRINO Cartesian servo.

Enables the servo, drives the target along a circle (plus an orientation
oscillation) at a fixed publish rate, and reports the exact servo timing
(/fairino_servo/timing_report), joint feedback rate, ServoJ return codes, UDP
state, write() period and the servo state over the run. Nothing is logged per
1 ms cycle; the servo node keeps the samples in a ring buffer.

Example:
  ros2 run fairinoplan_bringup fairino_servo_benchmark.py \
      --duration 60 --radius 0.05 --period 4 --label "1 kHz, no barrier" \
      --output /tmp/bench_1khz.json
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time

import numpy as np
import pinocchio as pin
import rclpy
from ament_index_python.packages import get_package_share_path
from fairino_msgs.msg import RobotStatus, ServoStatus
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class Benchmark(Node):
    def __init__(self) -> None:
        super().__init__("fairino_servo_benchmark")
        self.lock = threading.Lock()
        self.servo: ServoStatus | None = None
        self.robot: RobotStatus | None = None
        self.reset_pose: PoseStamped | None = None
        self.reset_count = 0
        self.joint_count = 0
        self.states: dict[str, int] = {}
        self.max_tracking_error = 0.0
        self.robot_samples: list[RobotStatus] = []
        self.pub = self.create_publisher(PoseStamped, "/fairino_servo/target_pose", 1)
        self.create_subscription(ServoStatus, "/fairino_servo/status", self.on_servo, 10)
        self.create_subscription(RobotStatus, "/fairino_robot/status", self.on_robot, 10)
        self.create_subscription(PoseStamped, "/fairino_servo/reset_target_pose", self.on_reset, LATCHED)
        self.create_subscription(JointState, "/joint_states", self.on_joint, 10)
        self.clients_ = {
            name: self.create_client(Trigger, f"/fairino_servo/{name}")
            for name in ["enable", "hold", "disable", "reset_timing", "timing_report"]
        }
        self.recording = False
        self.last_joint: JointState | None = None
        urdf = (get_package_share_path("fairino_description") / "urdf" / "fairino20_v6.urdf").read_text()
        self.model = pin.buildModelFromXML(urdf)
        self.data = self.model.createData()
        self.tip = self.model.getFrameId("wrist3_link")

    def measured_tcp(self) -> np.ndarray | None:
        with self.lock:
            msg = self.last_joint
        if msg is None:
            return None
        q = pin.neutral(self.model)
        for name, value in zip(msg.name, msg.position):
            if self.model.existJointName(name):
                q[self.model.joints[self.model.getJointId(name)].idx_q] = value
        pin.framesForwardKinematics(self.model, self.data, q)
        return self.data.oMf[self.tip].translation.copy()

    def on_servo(self, msg: ServoStatus) -> None:
        with self.lock:
            self.servo = msg
            if self.recording:
                self.states[msg.state] = self.states.get(msg.state, 0) + 1
                self.max_tracking_error = max(self.max_tracking_error, msg.tracking_error_rad)

    def on_robot(self, msg: RobotStatus) -> None:
        with self.lock:
            self.robot = msg
            if self.recording:
                self.robot_samples.append(msg)

    def on_reset(self, msg: PoseStamped) -> None:
        with self.lock:
            self.reset_pose = msg
            self.reset_count += 1

    def on_joint(self, msg: JointState) -> None:
        with self.lock:
            self.last_joint = msg
            if self.recording:
                self.joint_count += 1

    def call(self, name: str, timeout: float = 20.0) -> Trigger.Response:
        client = self.clients_[name]
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"/fairino_servo/{name} not available")
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise RuntimeError(f"/fairino_servo/{name} timed out")
        return future.result()


def pose_to_se3(msg: PoseStamped) -> pin.SE3:
    o = msg.pose.orientation
    p = msg.pose.position
    return pin.SE3(pin.Quaternion(o.w, o.x, o.y, o.z).normalized().matrix(), np.array([p.x, p.y, p.z]))


def se3_to_pose(pose: pin.SE3, stamp) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = "base_link"
    msg.header.stamp = stamp
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in pose.translation)
    q = pin.Quaternion(pose.rotation)
    msg.pose.orientation.x, msg.pose.orientation.y = float(q.x), float(q.y)
    msg.pose.orientation.z, msg.pose.orientation.w = float(q.z), float(q.w)
    return msg


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--radius", type=float, default=0.05, help="circle radius [m] (YZ plane of base_link)")
    parser.add_argument("--period", type=float, default=4.0, help="circle period [s]")
    parser.add_argument("--rotation", type=float, default=0.2, help="orientation oscillation amplitude [rad]")
    parser.add_argument("--rate", type=float, default=100.0, help="target publish rate [Hz]")
    parser.add_argument("--ramp", type=float, default=1.0, help="radius ramp-in time [s]")
    parser.add_argument("--label", default="")
    parser.add_argument("--output", default="")
    parser.add_argument("--disable-at-end", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = Benchmark()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    try:
        deadline = time.monotonic() + 15.0
        while node.servo is None and time.monotonic() < deadline:
            time.sleep(0.05)
        if node.servo is None:
            raise RuntimeError("no /fairino_servo/status")
        resets_before = node.reset_count
        response = node.call("enable")
        print(f"enable: success={response.success} {response.message}")
        if not response.success:
            raise RuntimeError("servo could not be enabled")
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and (node.reset_count == resets_before or node.servo.state != "ACTIVE"):
            time.sleep(0.01)
        if node.servo.state != "ACTIVE" or node.reset_count == resets_before:
            raise RuntimeError(f"servo not ACTIVE after enable (state {node.servo.state})")
        start_pose = pose_to_se3(node.reset_pose)
        print(f"start TCP: {np.round(start_pose.translation, 4).tolist()}")

        node.call("reset_timing")
        with node.lock:
            node.recording = True
        omega = 2.0 * math.pi / args.period
        dt = 1.0 / args.rate
        started = time.monotonic()
        next_time = started
        position_errors = []
        while True:
            t = time.monotonic() - started
            if t >= args.duration:
                break
            scale = min(1.0, t / args.ramp) if args.ramp > 0.0 else 1.0
            scale = scale * scale * (3.0 - 2.0 * scale)
            offset = np.array([0.0, math.sin(omega * t), 1.0 - math.cos(omega * t)]) * args.radius * scale
            rotation = pin.exp3(np.array([0.0, 0.0, args.rotation * scale * math.sin(omega * t)]))
            target = pin.SE3(start_pose.rotation @ rotation, start_pose.translation + offset)
            node.pub.publish(se3_to_pose(target, node.get_clock().now().to_msg()))
            measured = node.measured_tcp()
            if measured is not None and t > args.ramp:
                position_errors.append(float(np.linalg.norm(measured - target.translation)))
            next_time += dt
            time.sleep(max(0.0, next_time - time.monotonic()))
        elapsed = time.monotonic() - started
        with node.lock:
            node.recording = False
        report = json.loads(node.call("timing_report").message)
        final_state = node.servo.state
        hold = node.call("hold")
        print(f"hold: {hold.success} {hold.message}")
        if args.disable_at_end:
            print(f"disable: {node.call('disable').message}")

        robot = node.robot_samples
        result = {
            "label": args.label,
            "duration_s": elapsed,
            "trajectory": {
                "radius_m": args.radius,
                "period_s": args.period,
                "peak_tcp_speed_m_s": args.radius * omega,
                "rotation_amplitude_rad": args.rotation,
                "publish_rate_hz": args.rate,
            },
            "servo": report,
            "servo_states_seen": node.states,
            "final_state": final_state,
            "max_tracking_error_rad": node.max_tracking_error,
            # TCP(measured joints) vs. published target; includes the filter
            # lag (tau) and the robot following delay by design.
            "tcp_position_error_mm": {
                "rms": float(np.sqrt(np.mean(np.square(position_errors))) * 1000.0) if position_errors else None,
                "max": float(np.max(position_errors) * 1000.0) if position_errors else None,
            },
            "joint_states_rate_hz": node.joint_count / elapsed,
        }
        if robot:
            covered = (robot[-1].header.stamp.sec - robot[0].header.stamp.sec) + (
                robot[-1].header.stamp.nanosec - robot[0].header.stamp.nanosec
            ) * 1e-9
            result["robot"] = {
                "status_samples": len(robot),
                "status_window_s": covered,
                "servoj_calls_per_s": (robot[-1].servoj_calls - robot[0].servoj_calls) / covered if covered > 0 else 0.0,
                "servoj_flow_skips": robot[-1].servoj_flow_skips - robot[0].servoj_flow_skips,
                "servoj_flow_rebaselines": robot[-1].servoj_flow_rebaselines - robot[0].servoj_flow_rebaselines,
                "servoj_flow_probes": robot[-1].servoj_flow_probes - robot[0].servoj_flow_probes,
                "servoj_flow_drift_corrections": robot[-1].servoj_flow_drift_corrections
                - robot[0].servoj_flow_drift_corrections,
                "servoj_queue_estimate_max": max(r.servoj_queue_estimate for r in robot),
                "joint_velocity_deg_s_max": [
                    float(max(abs(r.joint_velocity_deg_s[j]) for r in robot)) for j in range(6)
                ],
                "speed_scale_manual": robot[-1].speed_scale_manual,
                "speed_scale_auto": robot[-1].speed_scale_auto,
                "robot_mode": int(robot[-1].robot_mode),
                "model": robot[-1].robot_model,
                "controller_version": robot[-1].controller_version,
                "servoj_calls": robot[-1].servoj_calls - robot[0].servoj_calls,
                "servoj_errors": robot[-1].servoj_errors - robot[0].servoj_errors,
                "servoj_return_codes_seen": sorted({r.last_servoj_return_code for r in robot}),
                "jump_guard_trips": robot[-1].jump_guard_trips - robot[0].jump_guard_trips,
                "udp_cmd_state_seen": sorted({r.udp_cmd_state for r in robot}),
                "streaming_states_seen": sorted({r.streaming_state for r in robot}),
                "joint_feedback_rate_hz_mean": float(np.mean([r.joint_feedback_rate_hz for r in robot])),
                "joint_feedback_age_ms_max": float(max(r.joint_feedback_age_s for r in robot) * 1000.0),
                "write_period_mean_ms": float(np.mean([r.write_period_mean_ms for r in robot])),
                "write_period_p99_ms_max": float(max(r.write_period_p99_ms for r in robot)),
                "write_period_max_ms": float(max(r.write_period_max_ms for r in robot)),
                "write_period_overruns": int(sum(r.write_period_overruns for r in robot)),
                "main_sub_codes_seen": sorted({(r.main_error_code, r.sub_error_code) for r in robot}),
            }
        print(json.dumps(result, indent=2))
        print()
        print(f"| stage | samples | mean ms | p95 ms | p99 ms | max ms | budget ms | overruns |  ({args.label})")
        print("|---|---:|---:|---:|---:|---:|---:|---:|")
        for stage in report["stages"]:
            print(
                f"| {stage['name']} | {stage['samples']} | {stage['mean_ms']:.4f} | {stage['p95_ms']:.4f} | "
                f"{stage['p99_ms']:.4f} | {stage['max_ms']:.4f} | {stage['budget_ms']:.2f} | {stage['overruns']} |"
            )
        if args.output:
            with open(args.output, "w", encoding="utf-8") as handle:
                json.dump(result, handle, indent=2)
            print(f"written {args.output}")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
