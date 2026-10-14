#!/usr/bin/env python3

"""Deterministic state-machine test of fairino_cartesian_servo (mock hardware only).

A fake FAIRINO status source replaces the hardware plugin: it publishes
/fairino_robot/status and serves /fairino_robot/recover and /fairino_robot/stop
with scripted behaviour, so every fault class of the catalog can be injected
without a controller. Run it against the mock stack with the servo started with
robot_status_required:=true:

  ros2 launch fairinoplan_bringup fairino20_viser_servo.launch.py use_mock_hardware:=true
  # restart the servo with robot_status_required:=true, then
  ros2 run fairinoplan_bringup fairino_servo_state_machine_test.py

It refuses to run when another /fairino_robot/status publisher exists (real
hardware or SimMachine).
"""

from __future__ import annotations

import sys
import threading
import time

import rclpy
from fairino_msgs.msg import AuditEvent, RobotStatus, ServoStatus
from fairino_msgs.srv import RecoverRobot
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


class FakeRobot(Node):
    def __init__(self) -> None:
        super().__init__("fairino_robot")
        self.lock = threading.Lock()
        self.status = RobotStatus()
        s = self.status
        s.connected = True
        s.robot_model = "FAKE"
        s.robot_enabled = True
        s.robot_mode = 0
        s.joint_feedback_age_s = 0.004
        s.streaming_state = RobotStatus.STREAMING_ARMED
        s.streaming_reason = "streaming"
        self.publishing = True
        self.recover_ok = True
        self.recover_clears = True
        self.recover_calls = 0
        self.stop_calls = 0
        self.pub = self.create_publisher(RobotStatus, "~/status", 10)
        self.create_service(RecoverRobot, "~/recover", self.on_recover)
        self.create_service(Trigger, "~/stop", self.on_stop)
        self.create_timer(0.02, self.publish)

    def publish(self) -> None:
        with self.lock:
            if not self.publishing:
                return
            self.status.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(self.status)

    def set(self, **fields) -> None:
        with self.lock:
            for key, value in fields.items():
                setattr(self.status, key, value)
            fault = (
                self.status.main_error_code != 0
                or self.status.emergency_stop
                or self.status.safety_stop0
                or self.status.safety_door_alarm
                or self.status.collision
                or not self.status.robot_enabled
            )
            if fault and "streaming_state" not in fields:
                self.status.streaming_state = RobotStatus.STREAMING_DISARMED
                self.status.streaming_reason = "controller fault/safety signal"
        # Let at least two status messages reach the servo before acting.
        time.sleep(0.15)

    def on_recover(self, request, response):
        self.recover_calls += 1
        time.sleep(0.05)
        with self.lock:
            response.steps = ["fake ServoMoveEnd", "fake StopMotion", "fake ResetAllError"]
            safety = self.status.emergency_stop or self.status.safety_stop0 or self.status.safety_door_alarm
            if not self.recover_ok or safety:
                response.success = False
                response.failed_step = "verify cleared"
                response.message = "fake: fault remains"
                return response
            if self.recover_clears:
                self.status.main_error_code = 0
                self.status.sub_error_code = 0
                self.status.collision = False
                self.status.last_servoj_return_code = 0
            if not self.status.robot_enabled and request.enable_robot:
                self.status.robot_enabled = True
            if not self.status.robot_enabled:
                response.success = False
                response.failed_step = "robot disabled"
                response.message = "fake: robot disabled, enabling not requested"
                return response
            if self.status.main_error_code != 0:
                response.success = False
                response.failed_step = "verify cleared"
                response.message = "fake: fault remains"
                return response
            self.status.streaming_state = RobotStatus.STREAMING_ARMED  # mock hardware syncs at once
            self.status.streaming_reason = "streaming"
        response.success = True
        response.robot_enabled = True
        response.message = "fake recovery complete"
        return response

    def on_stop(self, _request, response):
        self.stop_calls += 1
        with self.lock:
            self.status.streaming_state = RobotStatus.STREAMING_DISARMED
            self.status.streaming_reason = "stop requested"
        response.success = True
        response.message = "fake stop"
        return response


class Driver(Node):
    def __init__(self) -> None:
        super().__init__("fairino_servo_state_machine_test")
        self.servo: ServoStatus | None = None
        self.events: list[AuditEvent] = []
        self.create_subscription(ServoStatus, "/fairino_servo/status", self.on_servo, 10)
        self.create_subscription(AuditEvent, "/fairino_servo/events", self.events.append, 200)
        self.clients_ = {
            name: self.create_client(Trigger, f"/fairino_servo/{name}")
            for name in ["enable", "disable", "hold", "recover", "reset_fault"]
        }

    def on_servo(self, msg: ServoStatus) -> None:
        self.servo = msg

    def call(self, name: str) -> Trigger.Response:
        client = self.clients_[name]
        client.wait_for_service(timeout_sec=5.0)
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 45.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.result()

    def wait_state(self, states, timeout=8.0) -> str:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.servo is not None and self.servo.state in states:
                return self.servo.state
            time.sleep(0.02)
        return self.servo.state if self.servo else "UNKNOWN"


def main() -> None:
    rclpy.init()
    driver = Driver()
    time.sleep(1.0)
    if driver.count_publishers("/fairino_robot/status") > 0:
        print("another /fairino_robot/status publisher exists (hardware/SimMachine): refusing to run")
        sys.exit(2)
    fake = FakeRobot()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(fake)
    executor.add_node(driver)
    threading.Thread(target=executor.spin, daemon=True).start()
    results: list[tuple[str, bool, str]] = []

    def check(name: str, condition: bool, detail: str) -> None:
        results.append((name, condition, detail))
        print(f"[{'PASS' if condition else 'FAIL'}] {name}: {detail}")

    def healthy() -> None:
        fake.set(main_error_code=0, sub_error_code=0, emergency_stop=False, safety_stop0=False,
                 safety_door_alarm=False, collision=False, robot_enabled=True, last_servoj_return_code=0,
                 streaming_state=RobotStatus.STREAMING_ARMED, streaming_reason="streaming")
        fake.recover_ok = True
        fake.recover_clears = True

    def back_to_hold() -> None:
        """Bring the servo to HOLD in servo mode from any state."""
        healthy()
        state = driver.wait_state({"DISABLED", "HOLD", "ACTIVE", "FAULT", "LOCKOUT"}, 3.0)
        if state in ("FAULT", "LOCKOUT"):
            driver.call("reset_fault")
        state = driver.wait_state({"DISABLED", "HOLD", "ACTIVE"}, 5.0)
        if state == "DISABLED":
            driver.call("enable")
            driver.wait_state({"ACTIVE"}, 5.0)
            driver.call("hold")
        elif state == "ACTIVE":
            driver.call("hold")
        driver.wait_state({"HOLD"}, 5.0)

    try:
        deadline = time.monotonic() + 10.0
        while driver.servo is None and time.monotonic() < deadline:
            time.sleep(0.05)
        if driver.servo is None or not driver.servo.robot_status_required:
            print("servo not running with robot_status_required:=true")
            sys.exit(2)
        time.sleep(1.0)
        back_to_hold()
        check("baseline", driver.servo.state == "HOLD", f"state {driver.servo.state}")

        # S1 whitelisted controller fault -> automatic recovery -> HOLD.
        driver.call("enable")
        driver.wait_state({"ACTIVE"})
        mark = len(driver.events)
        fake.set(main_error_code=1, sub_error_code=25)
        time.sleep(1.0)
        state = driver.wait_state({"HOLD"}, 10.0)
        faulted = [e for e in driver.events[mark:] if e.action == "ACTIVE->FAULT"]
        recovered = [e for e in driver.events[mark:] if e.action == "RECOVERING->HOLD"]
        check("S1 main=1 sub=25 -> FAULT(auto_recover) -> automatic recovery -> HOLD",
              bool(faulted) and "policy=auto_recover" in faulted[0].message and bool(recovered) and state == "HOLD",
              f"{faulted[0].message if faulted else 'no FAULT event'} | {driver.servo.last_recovery_result}")

        # S2 same fault repeated beyond max_recovery_attempts -> LOCKOUT.
        for _ in range(3):
            driver.call("enable")
            driver.wait_state({"ACTIVE"}, 5.0)
            fake.set(main_error_code=1, sub_error_code=25)
            state = driver.wait_state({"HOLD", "LOCKOUT"}, 12.0)
            if state == "LOCKOUT":
                break
        check("S2 repeated whitelisted fault -> LOCKOUT", driver.servo.state == "LOCKOUT",
              f"state {driver.servo.state}: {driver.servo.reason}")
        check("S2 hardware stop requested on LOCKOUT", fake.stop_calls > 0, f"stop calls {fake.stop_calls}")

        # S3 Reset Fault from LOCKOUT after the cause is gone.
        healthy()
        response = driver.call("reset_fault")
        state = driver.wait_state({"HOLD"}, 5.0)
        check("S3 Reset Fault from LOCKOUT", response.success and state == "HOLD", response.message)

        # S4 E-stop -> LOCKOUT; reset refused while active; accepted after release.
        back_to_hold()
        fake.set(emergency_stop=True)
        state = driver.wait_state({"LOCKOUT"}, 2.0)
        refused = driver.call("reset_fault")
        check("S4 E-stop -> LOCKOUT, reset refused while pressed",
              state == "LOCKOUT" and not refused.success, refused.message)
        fake.set(emergency_stop=False)
        accepted = driver.call("reset_fault")
        check("S4 reset after E-stop release", accepted.success, accepted.message)

        # S5 non-resettable controller fault -> LOCKOUT.
        back_to_hold()
        fake.set(main_error_code=1, sub_error_code=30)
        state = driver.wait_state({"LOCKOUT"}, 2.0)
        check("S5 main=1 sub=30 (non-resettable) -> LOCKOUT", state == "LOCKOUT", driver.servo.fault_description)

        # S6 unknown code -> FAULT, no automatic recovery, Recover refused, Reset Fault ok.
        back_to_hold()
        fake.set(main_error_code=42, sub_error_code=7)
        state = driver.wait_state({"FAULT"}, 2.0)
        time.sleep(4.0)  # longer than the recovery cooldown
        still = driver.servo.state
        recover = driver.call("recover")
        healthy()
        reset = driver.call("reset_fault")
        check("S6 unknown main=42 sub=7 -> FAULT, raw codes shown, no auto/Recover",
              state == "FAULT" and still == "FAULT" and not recover.success and reset.success
              and driver.servo.main_error_code in (0, 42),
              f"{recover.message} | reset: {reset.message}")

        # S7 collision -> FAULT (operator), operator Recover -> HOLD.
        back_to_hold()
        fake.set(collision=True)
        state = driver.wait_state({"FAULT"}, 2.0)
        time.sleep(4.0)
        still = driver.servo.state
        recover = driver.call("recover")
        state_after = driver.wait_state({"HOLD"}, 5.0)
        check("S7 collision -> FAULT, no auto, operator Recover -> HOLD",
              state == "FAULT" and still == "FAULT" and recover.success and state_after == "HOLD", recover.message)

        # S8 safety door -> LOCKOUT.
        back_to_hold()
        fake.set(safety_door_alarm=True)
        check("S8 safety door -> LOCKOUT", driver.wait_state({"LOCKOUT"}, 2.0) == "LOCKOUT",
              driver.servo.fault_description)

        # S9 status lost -> LOCKOUT.
        back_to_hold()
        with fake.lock:
            fake.publishing = False
        state = driver.wait_state({"LOCKOUT"}, 3.0)
        with fake.lock:
            fake.publishing = True
        check("S9 robot status lost -> LOCKOUT", state == "LOCKOUT", driver.servo.fault_description)

        # S10 ServoJ return code 29 (joint overrun) -> whitelisted recovery.
        back_to_hold()
        driver.call("enable")
        driver.wait_state({"ACTIVE"})
        fake.set(last_servoj_return_code=29, streaming_state=RobotStatus.STREAMING_DISARMED,
                 streaming_reason="ServoJ returned code 29")
        state = driver.wait_state({"FAULT"}, 2.0)
        sdk = driver.servo.sdk_return_code
        state = driver.wait_state({"HOLD"}, 10.0)
        check("S10 ServoJ code 29 -> auto recovery", sdk == 29 and state == "HOLD",
              f"sdk {sdk}, final {state}")

        # S11 robot disabled -> FAULT (operator); Recover enables.
        back_to_hold()
        fake.set(robot_enabled=False)
        state = driver.wait_state({"FAULT"}, 2.0)
        recover = driver.call("recover")
        check("S11 robot disabled -> FAULT, Recover enables", state == "FAULT" and recover.success, recover.message)

        # S12 recovery that fails -> FAULT, then LOCKOUT on the second failure.
        back_to_hold()
        fake.recover_ok = False
        fake.set(main_error_code=1, sub_error_code=37)
        first = driver.wait_state({"FAULT"}, 2.0)
        final = driver.wait_state({"LOCKOUT"}, 15.0)
        check("S12 failing recovery -> LOCKOUT after attempts", first == "FAULT" and final == "LOCKOUT",
              driver.servo.reason)
        healthy()
        driver.call("reset_fault")
        driver.call("disable")
    finally:
        passed = sum(1 for _, ok, _ in results if ok)
        print(f"\n{passed}/{len(results)} checks passed")
        executor.shutdown()
        fake.destroy_node()
        driver.destroy_node()
        rclpy.shutdown()
        sys.exit(0 if results and passed == len(results) else 1)


if __name__ == "__main__":
    main()
