"""Collect bounded mock feedback and optionally probe the Docker simulator."""
import json
import math
import os
import socket
import time
import urllib.request
from pathlib import Path

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


def collect_joints():
    rclpy.init()
    node = rclpy.create_node("ci_joint_state_collector")
    samples = []
    expected = {f"j{i}" for i in range(1, 7)}

    def receive(msg):
        positions = dict(zip(msg.name, msg.position))
        if not expected.issubset(positions):
            return
        if not all(math.isfinite(positions[j]) for j in expected):
            return
        samples.append({"received_monotonic": time.monotonic(),
                        "stamp_sec": msg.header.stamp.sec,
                        "stamp_nanosec": msg.header.stamp.nanosec,
                        "position": positions})

    subscription = node.create_subscription(
        JointState, "/joint_states", receive, qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + 90
        while len(samples) < 100 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        Path("/artifacts/joint_states.json").write_text(json.dumps(samples, indent=2))
        if len(samples) < 100:
            raise RuntimeError(f"Expected 100 valid six-joint samples; received {len(samples)}")
        print(f"Collected {len(samples)} valid mock joint-state samples", flush=True)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


def probe_simulator():
    # Docker DNS resolves this job's simulator, not the host's real robot IP.
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    deadline = time.monotonic() + 120
    last_error = "not ready"
    while time.monotonic() < deadline:
        try:
            with opener.open("http://simmachine/", timeout=3) as response:
                status = response.status
            with socket.create_connection(("simmachine", 8083), timeout=3) as connection:
                connection.settimeout(3)
                data = connection.recv(4096)
            if not data:
                raise RuntimeError("8083 closed without status data")
            result = {"http_status": status, "status_port": 8083,
                      "received_bytes": len(data),
                      "note": "Connectivity check only; protocol/trajectory not validated"}
            Path("/artifacts/simmachine.json").write_text(json.dumps(result, indent=2))
            print(json.dumps(result), flush=True)
            return
        except (OSError, RuntimeError) as error:
            last_error = str(error)
            time.sleep(2)
    Path("/artifacts/simmachine.json").write_text(json.dumps({"error": last_error}))
    raise RuntimeError(f"SimMachine readiness timeout: {last_error}")


if __name__ == "__main__":
    collect_joints()
    if os.environ.get("CI_MODE") == "simmachine":
        probe_simulator()
