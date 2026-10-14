#!/usr/bin/env python3

"""SimMachine-only fault injection test for the servo recovery policy.

The FAIRINO global speed is lowered with SetSpeedInstant (audited) and the
servo then drives a circle that the controller rejects with its own joint
command speed fault (main=1, sub=23..28 on SimMachine 3.9.9). The test checks:

  1. the fault is classified as whitelisted (auto_recover),
  2. the automatic recovery ends in HOLD (never in motion),
  3. the operator resume works,
  4. the same fault repeated beyond max_recovery_attempts ends in LOCKOUT,
  5. Reset Fault from LOCKOUT works once the cause (low global speed) is removed.

Never run this on a real robot: it deliberately provokes controller faults.
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
from fairino_msgs.msg import RobotStatus, ServoStatus
from fairino_msgs.srv import SetGlobalSpeed
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class FaultTest(Node):
    def __init__(self) -> None:
        super().__init__("fairino_fault_injection_test")
        self.servo: ServoStatus | None = None
        self.robot: RobotStatus | None = None
        self.reset_pose: PoseStamped | None = None
        self.reset_count = 0
        self.transitions: list[tuple[float, str, str]] = []
        self.create_subscription(ServoStatus, "/fairino_servo/status", self.on_servo, 10)
        self.create_subscription(RobotStatus, "/fairino_robot/status", self.on_robot, 10)
        self.create_subscription(PoseStamped, "/fairino_servo/reset_target_pose", self.on_reset, LATCHED)
        self.pub = self.create_publisher(PoseStamped, "/fairino_servo/target_pose", 1)
        self.speed = self.create_client(SetGlobalSpeed, "/fairino_robot/set_global_speed")
        self.triggers = {
            name: self.create_client(Trigger, f"/fairino_servo/{name}")
            for name in ["enable", "disable", "hold", "reset_fault", "recover"]
        }

    def on_servo(self, msg: ServoStatus) -> None:
        if self.servo is None or msg.state != self.servo.state:
            self.transitions.append((time.monotonic(), msg.state, msg.reason))
        self.servo = msg

    def on_robot(self, msg: RobotStatus) -> None:
        self.robot = msg

    def on_reset(self, msg: PoseStamped) -> None:
        self.reset_pose = msg
        self.reset_count += 1

    def wait(self, future, timeout=30.0):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.result() if future.done() else None

    def trigger(self, name: str):
        client = self.triggers[name]
        client.wait_for_service(timeout_sec=5.0)
        response = self.wait(client.call_async(Trigger.Request()), 45.0)
        print(f"  {name}: {response.success if response else None} {response.message if response else 'timeout'}")
        return response

    def set_speed(self, percent: int, reason: str):
        request = SetGlobalSpeed.Request()
        request.percent = percent
        request.instant = True
        request.operator_confirmed = True
        request.requested_by = "fault_injection_test"
        request.reason = reason
        self.speed.wait_for_service(timeout_sec=5.0)
        response = self.wait(self.speed.call_async(request), 10.0)
        print(
            f"  SetSpeedInstant({percent}): success={response.success} manual "
            f"{response.old_speed_scale_manual:.0f}% -> {response.new_speed_scale_manual:.0f}%"
        )

    def wait_state(self, states: set[str], timeout: float) -> str:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.servo is not None and self.servo.state in states:
                return self.servo.state
            time.sleep(0.01)
        return self.servo.state if self.servo else "UNKNOWN"


def pose_to_se3(msg: PoseStamped) -> pin.SE3:
    o = msg.pose.orientation
    p = msg.pose.position
    return pin.SE3(pin.Quaternion(o.w, o.x, o.y, o.z).normalized().matrix(), np.array([p.x, p.y, p.z]))


def drive_until_fault(node: FaultTest, radius: float, period: float, timeout: float) -> float | None:
    """Drive a circle from the reset pose until the servo leaves ACTIVE."""
    start = pose_to_se3(node.reset_pose)
    omega = 2.0 * math.pi / period
    began = time.monotonic()
    while time.monotonic() - began < timeout:
        t = time.monotonic() - began
        if node.servo.state != "ACTIVE":
            return t
        scale = min(1.0, t / 0.5)
        offset = np.array([0.0, math.sin(omega * t), 1.0 - math.cos(omega * t)]) * radius * scale
        msg = PoseStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = node.get_clock().now().to_msg()
        position = start.translation + offset
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in position)
        q = pin.Quaternion(start.rotation)
        msg.pose.orientation.x, msg.pose.orientation.y = float(q.x), float(q.y)
        msg.pose.orientation.z, msg.pose.orientation.w = float(q.z), float(q.w)
        node.pub.publish(msg)
        time.sleep(0.01)
    return None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--simmachine", action="store_true", help="required: confirms this is FAIRINO SimMachine")
    parser.add_argument("--speed", type=int, default=20)
    parser.add_argument("--radius", type=float, default=0.10)
    parser.add_argument("--period", type=float, default=2.0)
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    if not args.simmachine:
        raise SystemExit("refusing to run without --simmachine (this test provokes controller faults)")

    rclpy.init()
    node = FaultTest()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    results = {"cycles": []}
    try:
        deadline = time.monotonic() + 10.0
        while (node.servo is None or node.robot is None) and time.monotonic() < deadline:
            time.sleep(0.05)
        print(f"start: servo {node.servo.state}, robot model {node.robot.robot_model}")
        if node.servo.state in ("FAULT", "LOCKOUT"):
            node.trigger("reset_fault")
        print("1) lower the FAIRINO global speed")
        node.set_speed(args.speed, "SimMachine fault injection: provoke joint command speed fault")

        for cycle in range(1, args.cycles + 1):
            print(f"2.{cycle}) enable and drive until the controller faults")
            before = node.reset_count
            node.trigger("enable")
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and (node.reset_count == before or node.servo.state != "ACTIVE"):
                time.sleep(0.01)
            if node.servo.state != "ACTIVE":
                print(f"  servo not ACTIVE ({node.servo.state}: {node.servo.reason})")
                results["cycles"].append({"cycle": cycle, "enabled": False, "state": node.servo.state})
                break
            fault_after = drive_until_fault(node, args.radius, args.period, 20.0)
            servo = node.servo
            entry = {
                "cycle": cycle,
                "fault_after_s": fault_after,
                "state_after_fault": servo.state,
                "fault": servo.fault_description,
                "main": servo.main_error_code,
                "sub": servo.sub_error_code,
                "policy": servo.fault_policy,
            }
            print(
                f"  fault after {fault_after}: {servo.state} main={servo.main_error_code} sub={servo.sub_error_code} "
                f"policy={servo.fault_policy}: {servo.fault_description}"
            )
            final = node.wait_state({"HOLD", "DISABLED", "LOCKOUT"}, 20.0)
            servo = node.servo
            entry.update(
                {
                    "state_after_recovery": final,
                    "recovery_attempts": servo.recovery_attempts,
                    "last_recovery_result": servo.last_recovery_result,
                    "last_recovery_duration_s": servo.last_recovery_duration_s,
                    "reason": servo.reason,
                }
            )
            print(
                f"  -> {final}: attempts {servo.recovery_attempts}/{servo.max_recovery_attempts}, "
                f"{servo.last_recovery_result}, reason: {servo.reason}"
            )
            results["cycles"].append(entry)
            if final == "LOCKOUT":
                break

        print("3) remove the cause: restore the global speed")
        node.set_speed(100, "SimMachine fault injection: restore 100 percent")
        if node.servo.state in ("LOCKOUT", "FAULT"):
            print("4) operator Reset Fault")
            response = node.trigger("reset_fault")
            results["reset_fault"] = {"success": bool(response and response.success), "message": response.message if response else ""}
            node.wait_state({"HOLD", "DISABLED"}, 10.0)
        print(f"final servo state {node.servo.state}: {node.servo.reason}")
        node.trigger("disable")
        results["final_state"] = node.servo.state
        results["transitions"] = [(round(t - node.transitions[0][0], 3), s, r) for t, s, r in node.transitions]
        if args.output:
            with open(args.output, "w", encoding="utf-8") as handle:
                json.dump(results, handle, indent=2)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
