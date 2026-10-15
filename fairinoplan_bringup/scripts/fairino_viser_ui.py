#!/usr/bin/env python3

"""Viser operator UI for the FAIRINO FR20 Cartesian servo.

The UI only displays state and publishes requests: the TCP gizmo publishes a
PoseStamped target, buttons call services asynchronously. No IK, no solver and
no FAIRINO SDK call runs in a Viser callback. The robot is drawn from
/joint_states at ``display_frequency`` (30 Hz).

The gizmo and every published pose use the URDF frame ``flange``, which
coincides with the TCP of FAIRINO tool 0 as the WebApp/SimMachine shows it.
A planned trajectory is previewed with a translucent robot before Execute.
"""

from __future__ import annotations

import json
import threading
import time
from collections import deque

import numpy as np
import pinocchio as pin
import rclpy
import viser
from ament_index_python.packages import get_package_share_path
from fairino_msgs.msg import AuditEvent, RobotStatus, ServoStatus
from fairino_msgs.srv import SetGlobalSpeed, SetSafetySpeed
from geometry_msgs.msg import PoseStamped
from pinocchio.visualize import ViserVisualizer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory


JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]
LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
EVENTS_QOS = QoSProfile(
    depth=200,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
STREAMING = {0: "DISARMED", 1: "WAIT_SYNC", 2: "ARMED"}
MOTION_STATE = {1: "stopped", 2: "running", 3: "paused", 4: "drag"}
SAFETY_ENABLE = ["0 - disabled", "1 - enabled in manual mode", "2 - enabled in all modes"]
SAFETY_STRATEGY = [
    "0 - stop and alarm",
    "1 - automatic speed limiting (manual mode only)",
    "2 - stop, alarm and disable",
]


def yes_no(value: bool) -> str:
    return "**YES**" if value else "no"


def fmt_age(value: float) -> str:
    return "n/a" if value is None or value < 0.0 else f"{value * 1000.0:.1f} ms"


class FairinoViserUi(Node):
    def __init__(self) -> None:
        super().__init__("fairino_viser_ui")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8000)
        self.declare_parameter("display_frequency", 30.0)
        self.declare_parameter("heartbeat_frequency", 30.0)
        self.declare_parameter("robot_status_required", True)
        self.declare_parameter("tip_frame", "flange")
        # Home pose for the "Home" button [deg], j1..j6 (also the mock start pose).
        self.declare_parameter("home_joints_deg", [0.0, -90.0, 90.0, -90.0, -90.0, 0.0])
        self._robot_status_required = bool(self.get_parameter("robot_status_required").value)

        # Same URDF as RoboPlan.
        description_share = get_package_share_path("fairino_description")
        urdf_xml = (description_share / "urdf" / "fairino20_v6.urdf").read_text()
        package_paths = [description_share.as_posix(), description_share.parent.as_posix()]
        self._model = pin.buildModelFromXML(urdf_xml)
        self._data = self._model.createData()
        collision_model = pin.buildGeomFromUrdfString(
            self._model, urdf_xml, pin.GeometryType.COLLISION, package_dirs=package_paths
        )
        visual_model = pin.buildGeomFromUrdfString(
            self._model, urdf_xml, pin.GeometryType.VISUAL, package_dirs=package_paths
        )
        self._viz = ViserVisualizer(self._model, collision_model, visual_model)
        # Viser 1.x: pass our own server so host/port are honoured.
        self._server = viser.ViserServer(
            host=str(self.get_parameter("host").value),
            port=int(self.get_parameter("port").value),
        )
        self._viz.initViewer(viewer=self._server, open=False, loadModel=True)
        # Translucent copy of the robot that replays a planned trajectory.
        self._preview = ViserVisualizer(self._model, collision_model, visual_model)
        self._preview.initViewer(viewer=self._server, open=False, loadModel=False)
        self._preview.loadViewerModel(rootNodeName="plan_preview", visual_color=[0.2, 0.55, 1.0, 0.35])
        self._preview.displayCollisions(False)
        self._preview.displayFrames(False)
        self._preview.displayVisuals(False)

        self._joint_q_index = {
            name: self._model.joints[self._model.getJointId(name)].idx_q for name in JOINT_NAMES
        }
        self._base_frame = self._model.getFrameId("base_link")
        self._tip_frame = self._model.getFrameId(str(self.get_parameter("tip_frame").value))
        self._lock = threading.RLock()
        self._q = pin.neutral(self._model)
        self._have_joint_state = False
        self._marker_initialized = False
        self._suppress_marker_event = False
        self._servo: ServoStatus | None = None
        self._robot: RobotStatus | None = None
        self._plan_status: dict = {"state": "unknown", "message": "planner not running"}
        self._events: deque = deque(maxlen=60)
        # Planned trajectory for the preview: times [s] and joint positions (N x 6, JOINT_NAMES).
        self._preview_times: np.ndarray | None = None
        self._preview_positions: np.ndarray | None = None
        self._preview_start = 0.0
        self._preview_shown = False
        self._last_result = "—"
        self._start_time = self.get_clock().now()

        # ROS interfaces.
        self._target_pub = self.create_publisher(PoseStamped, "/fairino_servo/target_pose", 1)
        self._plan_goal_pub = self.create_publisher(PoseStamped, "/fairinoplan/pose_goal", 1)
        self._joint_goal_pub = self.create_publisher(JointState, "/fairinoplan/joint_goal", 1)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 1)
        self.create_subscription(ServoStatus, "/fairino_servo/status", self._on_servo_status, 5)
        self.create_subscription(RobotStatus, "/fairino_robot/status", self._on_robot_status, 5)
        self.create_subscription(
            PoseStamped, "/fairino_servo/reset_target_pose", self._on_reset_target, LATCHED
        )
        self.create_subscription(AuditEvent, "/fairino_robot/events", self._on_event, EVENTS_QOS)
        self.create_subscription(AuditEvent, "/fairino_servo/events", self._on_event, EVENTS_QOS)
        self.create_subscription(String, "/fairinoplan/status", self._on_plan_status, LATCHED)
        self.create_subscription(
            JointTrajectory, "/fairinoplan/planned_path", self._on_planned_path, LATCHED
        )
        self._servo_clients = {
            name: self.create_client(Trigger, f"/fairino_servo/{name}")
            for name in ["enable", "hold", "disable", "recover", "reset_fault", "reset_target"]
        }
        self._execute_client = self.create_client(Trigger, "/fairinoplan/execute")
        self._speed_client = self.create_client(SetGlobalSpeed, "/fairino_robot/set_global_speed")
        self._safety_client = self.create_client(SetSafetySpeed, "/fairino_robot/set_safety_speed")

        self._build_scene()
        self._build_gui()

        self.create_timer(1.0 / float(self.get_parameter("display_frequency").value), self._display)
        self.create_timer(1.0 / float(self.get_parameter("display_frequency").value), self._animate_preview)
        self.create_timer(
            1.0 / float(self.get_parameter("heartbeat_frequency").value), self._heartbeat
        )
        self.create_timer(0.5, self._update_panels)
        self.get_logger().info(
            "Viser UI ready at http://localhost:%d" % int(self.get_parameter("port").value)
        )

    # ------------------------------------------------------------------ scene
    def _build_scene(self) -> None:
        scene = self._server.scene
        scene.add_grid("/ground", width=4.0, height=4.0, cell_size=0.25, position=(0.0, 0.0, 0.0))
        self._server.initial_camera.position = (1.6, -2.6, 2.0)
        self._server.initial_camera.look_at = (-0.4, 0.0, 0.7)
        # Arrows: one axis; squares: translation in the XY/XZ/YZ plane of the
        # TCP frame; rings: rotation.
        self._marker = scene.add_transform_controls(
            "/fairino/tcp_target", scale=0.25, depth_test=False, disable_sliders=False
        )
        self._marker.on_update(lambda _: self._on_marker_moved())
        # Measured TCP (flange) axes; the gizmo shows the target.
        self._tcp_axes = scene.add_frame("/fairino/tcp_measured", axes_length=0.08, axes_radius=0.004)
        self._path_handle = None

    # -------------------------------------------------------------------- gui
    def _build_gui(self) -> None:
        gui = self._server.gui
        gui.configure_theme(control_width="large")
        self._summary = gui.add_markdown("**Servo:** starting…")
        tabs = gui.add_tab_group()

        with tabs.add_tab("Control"):
            # Enable Servo switches to servo mode, Disable Servo back to plan mode.
            with gui.add_folder("Servo"):
                self._enable_button = gui.add_button("Enable Servo", color="green")
                self._hold_button = gui.add_button(
                    "Servo Hold", color="yellow", hint="Stop tracking the gizmo; the servo keeps the joints"
                )
                self._disable_button = gui.add_button(
                    "Plan Mode", hint="Leave servoing: the trajectory controller holds the robot (Plan / Execute)"
                )
                self._reset_target_button = gui.add_button("Reset Target to Current TCP")
            with gui.add_folder("Plan (RRT + TOPP-RA)"):
                self._plan_button = gui.add_button("Plan", hint="Plan to the gizmo pose (plan mode only)")
                self._home_button = gui.add_button(
                    "Home", hint="Plan to the home joint pose; check the preview, then Execute"
                )
                self._preview_checkbox = gui.add_checkbox(
                    "Preview planned motion", True, hint="Translucent robot replays the plan before Execute"
                )
                self._preview_speed = gui.add_slider("Preview speed", min=0.25, max=2.0, step=0.25, initial_value=1.0)
                self._execute_button = gui.add_button("Execute", color="blue")
                self._plan_markdown = gui.add_markdown("—")
            with gui.add_folder("Fault"):
                self._fault_markdown = gui.add_markdown("—")
                self._recover_button = gui.add_button(
                    "Recover",
                    color="orange",
                    hint="FAULT: controlled recovery; LOCKOUT or unknown fault: operator reset once the cause is gone",
                )
            self._result_markdown = gui.add_markdown("**Last command:** —")

        with tabs.add_tab("Status"):
            self._status_markdown = gui.add_markdown("waiting for status…")
            self._timing_markdown = gui.add_markdown("")

        with tabs.add_tab("Speed"):
            self._speed_markdown = gui.add_markdown("waiting for robot status…")
            self._speed_number = gui.add_number(
                "Requested global speed [%]", 30, min=1, max=100, step=1
            )
            self._speed_reason = gui.add_text("Reason", "")
            self._speed_confirm = gui.add_checkbox(
                "I confirm: cell checked, global speed change", False
            )
            self._speed_button = gui.add_button("Apply Global Speed")
            self._limits_markdown = gui.add_markdown("")
            with gui.add_folder("Safety speed (SAFETY-RELATED)", expand_by_default=False):
                self._safety_info = gui.add_markdown("")
                self._safety_enable = gui.add_dropdown("Function", SAFETY_ENABLE, initial_value=SAFETY_ENABLE[1])
                self._safety_tcp = gui.add_number("Max TCP speed [mm/s]", 250.0, min=0.0, max=1000.0, step=1.0)
                self._safety_strategy = gui.add_dropdown(
                    "Overspeed strategy", SAFETY_STRATEGY, initial_value=SAFETY_STRATEGY[0]
                )
                self._safety_joints = gui.add_text("Joint limits J1..J6 [deg/s]", "45,45,45,45,45,45")
                self._safety_password = gui.add_text("Safety password (not masked)", "")
                self._safety_reason = gui.add_text("Reason", "")
                self._safety_confirm = gui.add_checkbox(
                    "SAFETY-RELATED: I am authorized; SimMachine/approved test", False
                )
                self._safety_button = gui.add_button("Apply Safety Speed", color="red")

        with tabs.add_tab("Audit"):
            self._audit_markdown = gui.add_markdown("no events yet")

        self._enable_button.on_click(lambda _: self._trigger("enable"))
        self._hold_button.on_click(lambda _: self._trigger("hold", "Servo Hold"))
        self._disable_button.on_click(lambda _: self._trigger("disable", "Plan Mode"))
        self._reset_target_button.on_click(lambda _: self._reset_target())
        self._plan_button.on_click(lambda _: self._plan())
        self._home_button.on_click(lambda _: self._plan_home())
        self._execute_button.on_click(lambda _: self._execute())
        self._recover_button.on_click(lambda _: self._recover())
        self._speed_button.on_click(lambda _: self._apply_speed())
        self._safety_button.on_click(lambda _: self._apply_safety_speed())

    # -------------------------------------------------------------- callbacks
    def _on_joint_state(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if any(name not in positions for name in JOINT_NAMES):
            return
        values = [float(positions[name]) for name in JOINT_NAMES]
        if not all(np.isfinite(values)):
            return
        with self._lock:
            for name, value in zip(JOINT_NAMES, values):
                self._q[self._joint_q_index[name]] = value
            self._have_joint_state = True

    def _on_servo_status(self, msg: ServoStatus) -> None:
        with self._lock:
            self._servo = msg

    def _on_robot_status(self, msg: RobotStatus) -> None:
        with self._lock:
            self._robot = msg

    def _on_event(self, msg: AuditEvent) -> None:
        with self._lock:
            self._events.appendleft(msg)

    def _on_plan_status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            self._plan_status = status

    def _on_reset_target(self, msg: PoseStamped) -> None:
        # Ignore a latched reset pose from a previous session.
        if rclpy.time.Time.from_msg(msg.header.stamp) < self._start_time:
            return
        p = msg.pose.position
        o = msg.pose.orientation
        base_target = pin.SE3(pin.Quaternion(o.w, o.x, o.y, o.z).normalized().matrix(), np.array([p.x, p.y, p.z]))
        with self._lock:
            if not self._have_joint_state:
                return
            world_base, _ = self._world_poses()
            self._set_marker(world_base * base_target)

    def _on_planned_path(self, msg: JointTrajectory) -> None:
        if rclpy.time.Time.from_msg(msg.header.stamp) < self._start_time and msg.header.stamp.sec != 0:
            return
        if not msg.points:
            return
        step = max(1, len(msg.points) // 200)
        indices = [msg.joint_names.index(name) for name in JOINT_NAMES if name in msg.joint_names]
        if len(indices) != len(JOINT_NAMES):
            return
        points = []
        with self._lock:
            q = self._q.copy()
            data = self._model.createData()
        for point in msg.points[::step] + [msg.points[-1]]:
            for name, index in zip(JOINT_NAMES, indices):
                q[self._joint_q_index[name]] = point.positions[index]
            pin.framesForwardKinematics(self._model, data, q)
            points.append(data.oMf[self._tip_frame].translation.copy())
        if self._path_handle is not None:
            self._path_handle.remove()
        self._path_handle = self._server.scene.add_spline_catmull_rom(
            "/fairino/planned_path", points=np.asarray(points), thickness=0.006, color=(30, 120, 220)
        )
        times = np.array([p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in msg.points])
        positions = np.array([[p.positions[i] for i in indices] for p in msg.points])
        with self._lock:
            self._preview_times = times
            self._preview_positions = positions
            self._preview_start = time.monotonic()

    # ------------------------------------------------------------ kinematics
    def _world_poses(self) -> tuple[pin.SE3, pin.SE3]:
        pin.framesForwardKinematics(self._model, self._data, self._q)
        return self._data.oMf[self._base_frame].copy(), self._data.oMf[self._tip_frame].copy()

    def _set_marker(self, world_pose: pin.SE3) -> None:
        quaternion = pin.Quaternion(world_pose.rotation)
        self._suppress_marker_event = True
        try:
            self._marker.position = world_pose.translation.copy()
            self._marker.wxyz = np.array([quaternion.w, quaternion.x, quaternion.y, quaternion.z])
        finally:
            self._suppress_marker_event = False
        self._marker_initialized = True

    def _marker_pose(self) -> PoseStamped | None:
        with self._lock:
            if not self._have_joint_state or not self._marker_initialized:
                return None
            world_base, _ = self._world_poses()
            wxyz = np.asarray(self._marker.wxyz, dtype=float)
            world_target = pin.SE3(
                pin.Quaternion(wxyz[0], wxyz[1], wxyz[2], wxyz[3]).normalized().matrix(),
                np.asarray(self._marker.position, dtype=float),
            )
            base_target = world_base.inverse() * world_target
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (
            float(v) for v in base_target.translation
        )
        quaternion = pin.Quaternion(base_target.rotation)
        msg.pose.orientation.x = float(quaternion.x)
        msg.pose.orientation.y = float(quaternion.y)
        msg.pose.orientation.z = float(quaternion.z)
        msg.pose.orientation.w = float(quaternion.w)
        return msg

    def _on_marker_moved(self) -> None:
        if self._suppress_marker_event:
            return
        pose = self._marker_pose()
        if pose is not None:
            self._target_pub.publish(pose)

    # ----------------------------------------------------------------- timers
    def _display(self) -> None:
        with self._lock:
            if not self._have_joint_state:
                return
            q = self._q.copy()
            initialize = not self._marker_initialized
            _, world_tip = self._world_poses()
        self._viz.display(q)
        quaternion = pin.Quaternion(world_tip.rotation)
        self._tcp_axes.position = world_tip.translation.copy()
        self._tcp_axes.wxyz = np.array([quaternion.w, quaternion.x, quaternion.y, quaternion.z])
        if initialize:
            with self._lock:
                self._set_marker(world_tip)

    def _animate_preview(self) -> None:
        # The preview runs only while a plan waits for Execute; it loops with a
        # 1 s pause on the goal pose. It never commands the robot.
        with self._lock:
            times = self._preview_times
            positions = self._preview_positions
            start = self._preview_start
            planned = self._plan_status.get("state") == "planned"
            q = self._q.copy()
        show = bool(self._preview_checkbox.value) and planned and times is not None and len(times) > 1
        if show != self._preview_shown:
            self._preview.displayVisuals(show)
            self._preview_shown = show
        if not show:
            return
        duration = float(times[-1])
        elapsed = (time.monotonic() - start) * float(self._preview_speed.value)
        t = min(elapsed % (duration + 1.0), duration)
        for column, name in enumerate(JOINT_NAMES):
            q[self._joint_q_index[name]] = np.interp(t, times, positions[:, column])
        self._preview.display(q)

    def _heartbeat(self) -> None:
        # A still gizmo is still a valid reference. The heartbeat only runs
        # while a browser is connected, so closing the browser lets the servo
        # target watchdog switch to HOLD.
        if self._server.get_clients():
            pose = self._marker_pose()
            if pose is not None:
                self._target_pub.publish(pose)

    # ---------------------------------------------------------------- actions
    def _set_result(self, text: str) -> None:
        self._last_result = text
        self._result_markdown.content = f"**Last command:** {text}"

    def _call(self, client, request, label: str, on_done=None) -> None:
        if not client.service_is_ready():
            self._set_result(f"{label}: service `{client.srv_name}` not available")
            self.get_logger().error(f"{label}: service {client.srv_name} not available")
            return
        self._set_result(f"{label}: requested…")
        self.get_logger().info(f"Operator request from Viser: {label}")
        future = client.call_async(request)

        def done(result_future):
            try:
                response = result_future.result()
            except Exception as exc:  # noqa: BLE001
                self._set_result(f"{label}: failed ({exc})")
                return
            message = getattr(response, "message", "")
            ok = getattr(response, "success", False)
            self._set_result(f"{label}: {'OK' if ok else 'REJECTED'} — {message}")
            if on_done is not None:
                on_done(response)

        future.add_done_callback(done)

    def _trigger(self, name: str, label: str | None = None) -> None:
        self._call(self._servo_clients[name], Trigger.Request(), label or name.replace("_", " ").title())

    def _recover(self) -> None:
        # One operator button for both servo services; the servo keeps its
        # rules (no exit from LOCKOUT while the cause is active, no guessed
        # procedure for unknown codes) and every recovery ends without motion.
        servo = self._servo
        if servo is not None and servo.state == "LOCKOUT":
            self._trigger("reset_fault")
            return

        def fallback(response) -> None:
            if not response.success and response.message.startswith("unknown fault"):
                self._trigger("reset_fault")

        self._call(self._servo_clients["recover"], Trigger.Request(), "Recover", on_done=fallback)

    def _reset_target(self) -> None:
        if self._servo_clients["reset_target"].service_is_ready():
            self._trigger("reset_target")
            return
        with self._lock:
            if self._have_joint_state:
                _, world_tip = self._world_poses()
                self._set_marker(world_tip)
        self._set_result("Reset target: gizmo moved to the current TCP (servo not running)")

    def _plan(self) -> None:
        servo = self._servo
        if servo is not None and (servo.state != "DISABLED" or servo.mode != "plan"):
            self._set_result("Plan: only in plan mode with the servo DISABLED")
            return
        pose = self._marker_pose()
        if pose is None:
            self._set_result("Plan: no gizmo pose yet")
            return
        self._plan_goal_pub.publish(pose)
        self._set_result("Plan: goal sent to /fairinoplan/pose_goal")

    def _plan_home(self) -> None:
        servo = self._servo
        if servo is not None and (servo.state != "DISABLED" or servo.mode != "plan"):
            self._set_result("Home: only in plan mode with the servo DISABLED")
            return
        goal = JointState()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.name = list(JOINT_NAMES)
        goal.position = [float(np.radians(v)) for v in self.get_parameter("home_joints_deg").value]
        self._joint_goal_pub.publish(goal)
        home = ", ".join(f"{v:.0f}" for v in self.get_parameter("home_joints_deg").value)
        self._set_result(f"Home ({home} deg): goal sent; check the preview, then Execute")

    def _execute(self) -> None:
        servo = self._servo
        if servo is not None and (servo.state != "DISABLED" or servo.mode != "plan"):
            self._set_result("Execute: only in plan mode with the servo DISABLED")
            return
        self._call(self._execute_client, Trigger.Request(), "Execute")

    def _apply_speed(self) -> None:
        request = SetGlobalSpeed.Request()
        request.percent = int(self._speed_number.value)
        request.instant = True
        request.operator_confirmed = bool(self._speed_confirm.value)
        request.requested_by = "viser"
        request.reason = str(self._speed_reason.value)
        if not request.operator_confirmed:
            self._set_result("Global speed: tick the confirmation box first")
            return

        def done(response):
            self._speed_confirm.value = False
            self._set_result(
                f"Global speed: {'OK' if response.success else 'FAILED'} code={response.sdk_return_code} "
                f"manual {response.old_speed_scale_manual:.0f}%→{response.new_speed_scale_manual:.0f}%, "
                f"auto {response.old_speed_scale_auto:.0f}%→{response.new_speed_scale_auto:.0f}% — {response.message}"
            )

        self._call(self._speed_client, request, f"SetSpeedInstant({request.percent})", done)

    def _apply_safety_speed(self) -> None:
        try:
            joints = [float(v) for v in str(self._safety_joints.value).split(",")]
        except ValueError:
            self._set_result("Safety speed: joint limits must be 6 comma-separated numbers")
            return
        if len(joints) != 6:
            self._set_result("Safety speed: 6 joint limits required")
            return
        request = SetSafetySpeed.Request()
        request.enable = SAFETY_ENABLE.index(self._safety_enable.value)
        request.max_tcp_velocity_mm_s = float(self._safety_tcp.value)
        request.strategy = SAFETY_STRATEGY.index(self._safety_strategy.value)
        request.max_joint_velocity_deg_s = joints
        request.safety_password = str(self._safety_password.value)
        request.operator_confirmed = bool(self._safety_confirm.value)
        request.requested_by = "viser"
        request.reason = str(self._safety_reason.value)
        if not request.operator_confirmed:
            self._set_result("Safety speed: tick the SAFETY-RELATED confirmation first")
            return

        def done(response):
            self._safety_confirm.value = False
            self._safety_password.value = ""
            self._set_result(
                f"SetVelReducePara: {'OK' if response.success else 'FAILED'} code={response.sdk_return_code} "
                f"password_check={response.password_check_return_code} — {response.message}"
            )

        self._call(self._safety_client, request, "SetVelReducePara", done)

    # ------------------------------------------------------------------ panels
    def _update_panels(self) -> None:
        try:
            self._refresh_panels()
        except Exception as exc:  # noqa: BLE001 - a display bug must not stop the UI
            self.get_logger().error(f"panel update failed: {exc}", throttle_duration_sec=5.0)

    def _refresh_panels(self) -> None:
        with self._lock:
            servo = self._servo
            robot = self._robot
            plan = dict(self._plan_status)
            events = list(self._events)[:25]
        self._update_summary(servo, robot)
        self._update_buttons(servo, robot, plan)
        self._update_status(servo, robot)
        self._update_speed(servo, robot)
        self._plan_markdown.content = (
            f"**Planner:** `{plan.get('state', '?')}` — {plan.get('message', '')}"
            + (
                f"  \npoints {plan['points']}, duration {plan.get('trajectory_duration_s', 0.0):.2f} s, "
                f"planning {plan.get('planning_time_s', 0.0):.2f} s"
                if "points" in plan
                else ""
            )
        )
        self._update_audit(events)

    def _update_summary(self, servo, robot) -> None:
        if servo is None:
            self._summary.content = "**Servo:** no status from `fairino_cartesian_servo`"
            return
        connection = (
            "mock hardware"
            if not servo.robot_status_required
            else ("connected" if robot is not None and robot.connected else "**NOT CONNECTED**")
        )
        self._summary.content = (
            f"**{servo.state}** · mode `{servo.mode}` · controller `{servo.active_position_controller}` · "
            f"FAIRINO {connection}  \n{servo.reason}"
        )
        if servo.fault_policy != "none":
            self._fault_markdown.content = (
                f"**{servo.fault_severity.upper()}** ({servo.fault_source}) main={servo.main_error_code} "
                f"sub={servo.sub_error_code} sdk={servo.sdk_return_code}  \n{servo.fault_description}  \n"
                f"Policy: `{servo.fault_policy}` · Action: {servo.suggested_action}  \n"
                f"Recovery attempts {servo.recovery_attempts}/{servo.max_recovery_attempts} · "
                f"last: {servo.last_recovery_result}  \n"
                "**Recover** runs the controlled recovery, or the operator reset (\"Reset Fault\") "
                "for LOCKOUT and unknown faults once the cause is removed."
            )
        else:
            self._fault_markdown.content = (
                f"No active fault. Last recovery: {servo.last_recovery_result}"
                + (f"  \n{servo.suggested_action}" if servo.suggested_action else "")
            )

    def _update_buttons(self, servo, robot, plan) -> None:
        state = servo.state if servo is not None else "UNKNOWN"
        plan_mode = servo is not None and servo.mode == "plan" and state == "DISABLED"
        self._enable_button.disabled = state not in ("DISABLED", "HOLD")
        self._hold_button.disabled = state != "ACTIVE"
        self._disable_button.disabled = state in ("DISABLED", "RECOVERING", "SYNCING", "UNKNOWN")
        self._recover_button.disabled = state not in ("FAULT", "LOCKOUT")
        self._plan_button.disabled = not plan_mode
        self._home_button.disabled = not plan_mode
        self._execute_button.disabled = not plan_mode or plan.get("state") != "planned"
        self._safety_button.disabled = robot is None or not robot.safety_config_writes_allowed

    def _update_status(self, servo, robot) -> None:
        lines = ["| item | value |", "|---|---|"]
        if servo is not None:
            lines += [
                f"| Servo state | `{servo.state}` — {servo.reason} |",
                f"| Active ROS controller | `{servo.active_position_controller}` (mode {servo.mode}) |",
                f"| Fault | {servo.fault_description or 'none'} |",
                f"| Suggested action | {servo.suggested_action or '—'} |",
                f"| Joint feedback age (ROS) | {fmt_age(servo.joint_state_age_s)} @ {servo.joint_state_rate_hz:.0f} Hz |",
                f"| Viser target age | {fmt_age(servo.target_age_s)} |",
                f"| Tracking error | {servo.tracking_error_rad * 1000.0:.2f} mrad |",
                f"| Recovery | attempts {servo.recovery_attempts}/{servo.max_recovery_attempts}, "
                f"auto {'on' if servo.auto_recovery_enabled else 'off'}, last {servo.last_recovery_result} "
                f"({servo.last_recovery_duration_s:.2f} s) |",
                f"| OInK / command rate | {servo.solver_frequency_hz:.0f} Hz / {servo.command_frequency_hz:.0f} Hz, "
                f"{'SCHED_FIFO' if servo.realtime_scheduling else 'no RT scheduling'} |",
            ]
        lines += self._tcp_rows(robot)
        if robot is not None:
            lines += [
                f"| FAIRINO connection | {'connected' if robot.connected else '**DISCONNECTED**'} "
                f"(SDK state {robot.sdk_com_state}) · {robot.robot_model} · ctrl {robot.controller_version} |",
                f"| Robot enabled | {'enabled' if robot.robot_enabled else '**DISABLED**'} |",
                f"| Mode | {'manual' if robot.robot_mode == 1 else 'automatic'} · motion "
                f"{MOTION_STATE.get(robot.motion_state, robot.motion_state)} |",
                f"| main_error_code / sub_error_code | **{robot.main_error_code} / {robot.sub_error_code}** |",
                f"| E-stop | {yes_no(robot.emergency_stop or robot.button_box_stop)} |",
                f"| Safety stop SI0 / SI1 | {yes_no(robot.safety_stop0)} / {yes_no(robot.safety_stop1)} |",
                f"| Collision | {yes_no(robot.collision)} (levels {list(robot.collision_level)}) |",
                f"| Safety door / plane / data | {yes_no(robot.safety_door_alarm)} / "
                f"{yes_no(robot.safety_plane_alarm)} / {yes_no(robot.safety_data_abnormal)} |",
                f"| Driver alarm / error axis | {robot.driver_alarm_axis} / {robot.driver_error_axis} |",
                f"| Soft limit / abnormal stop | {yes_no(robot.out_soft_limit_error)} / {yes_no(robot.abnormal_stop)} |",
                f"| UDP command state (20007) | {robot.udp_cmd_state} |",
                f"| ServoJ streaming | `{STREAMING.get(robot.streaming_state, robot.streaming_state)}` — "
                f"{robot.streaming_reason} |",
                f"| Last ServoJ return code | {robot.last_servoj_return_code} "
                f"(errors {robot.servoj_errors} / calls {robot.servoj_calls}, guard trips {robot.jump_guard_trips}) |",
                f"| Joint feedback age (controller) | {fmt_age(robot.joint_feedback_age_s)} @ "
                f"{robot.joint_feedback_rate_hz:.0f} Hz |",
                f"| write() period | mean {robot.write_period_mean_ms:.3f} / p99 {robot.write_period_p99_ms:.3f} / "
                f"max {robot.write_period_max_ms:.3f} ms, overruns {robot.write_period_overruns} |",
            ]
        elif servo is not None and not servo.robot_status_required:
            lines.append("| FAIRINO | mock hardware: no controller status |")
        self._status_markdown.content = "\n".join(lines)

        if servo is not None and servo.timing:
            timing = [
                "**Servo loop timing [ms], last 1 s** (overruns = samples above the budget)",
                "",
                "| stage | mean | p99 | max | overruns |",
                "|---|---:|---:|---:|---:|",
            ]
            for t in servo.timing:
                timing.append(
                    f"| {t.name} ({t.budget_ms:.1f}) | {t.mean_ms:.3f} | {t.p99_ms:.3f} | {t.max_ms:.3f} | "
                    f"{t.overruns_window}/{t.overruns_total} |"
                )
            self._timing_markdown.content = "\n".join(timing)

    def _tcp_rows(self, robot) -> list[str]:
        """Viser TCP (URDF flange from /joint_states) next to the FAIRINO TCP, in
        the WebApp convention: mm and rx, ry, rz [deg] about the fixed axes."""
        with self._lock:
            if not self._have_joint_state:
                return []
            world_base, world_tip = self._world_poses()
        tip = world_base.inverse() * world_tip
        rpy = np.degrees(pin.rpy.matrixToRpy(tip.rotation))
        position = tip.translation * 1000.0
        rows = [
            f"| TCP (Viser gizmo frame `flange`) | {position[0]:.1f}, {position[1]:.1f}, {position[2]:.1f} mm · "
            f"{rpy[0]:.2f}, {rpy[1]:.2f}, {rpy[2]:.2f} deg |"
        ]
        if robot is None or not robot.connected:
            return rows
        tcp = list(robot.tcp_pose)
        fairino = pin.SE3(pin.rpy.rpyToMatrix(*np.radians(tcp[3:])), np.array(tcp[:3]) / 1000.0)
        delta = tip.inverse() * fairino
        rows.append(
            f"| TCP (FAIRINO, tool {robot.tool_number}) | {tcp[0]:.1f}, {tcp[1]:.1f}, {tcp[2]:.1f} mm · "
            f"{tcp[3]:.2f}, {tcp[4]:.2f}, {tcp[5]:.2f} deg |"
        )
        note = (
            ""
            if robot.tool_number == 0
            else " — **the gizmo is at the flange; FAIRINO tool "
            f"{robot.tool_number} has its own TCP offset**"
        )
        rows.append(
            f"| Viser vs FAIRINO TCP | {np.linalg.norm(delta.translation) * 1000.0:.2f} mm, "
            f"{np.degrees(np.linalg.norm(pin.log3(delta.rotation))):.2f} deg{note} |"
        )
        return rows

    def _update_speed(self, servo, robot) -> None:
        if robot is None:
            self._speed_markdown.content = (
                "FAIRINO global speed: n/a (mock hardware or no `/fairino_robot/status`)"
            )
        else:
            active = "manual" if robot.robot_mode == 1 else "automatic"
            self._speed_markdown.content = (
                f"**FAIRINO global speed** (active mode: {active})  \n"
                f"manual {robot.speed_scale_manual:.0f} % · automatic {robot.speed_scale_auto:.0f} %  \n"
                "Applied with `SetSpeedInstant`; confirmation required; old/new values are audited."
            )
        velocity = (
            ", ".join(f"{np.degrees(v):.0f}" for v in servo.joint_velocity_limit_rad_s)
            if servo is not None
            else "?"
        )
        acceleration = (
            ", ".join(f"{np.degrees(v):.0f}" for v in servo.joint_acceleration_limit_rad_s2)
            if servo is not None and servo.joint_acceleration_limit_rad_s2
            else "off"
        )
        cartesian = (
            f"{servo.max_linear_speed_m_s:.2f} m/s, {np.degrees(servo.max_angular_speed_rad_s):.0f} deg/s, "
            f"filter τ {servo.filter_tau_s:.3f} s"
            if servo is not None
            else "?"
        )
        speed = (
            f"manual {robot.speed_scale_manual:.0f} %, auto {robot.speed_scale_auto:.0f} %"
            if robot is not None
            else "n/a"
        )
        self._limits_markdown.content = "\n".join(
            [
                "**Speed layers (do not mix them up)**",
                "",
                f"1. **Software / OInK limit** (servo parameters): joint velocity [deg/s] {velocity}; "
                f"joint acceleration [deg/s²] {acceleration}; Cartesian {cartesian}.",
                f"2. **FAIRINO global speed scaling** (Apply Global Speed, `SetSpeedInstant`): {speed}. "
                "Measured on SimMachine: it does not slow down ServoJ.",
                "3. **FAIRINO safety-speed threshold** (WebApp: Initial Setup → Safety → Safety Speed; "
                "`SetVelReducePara` below, SimMachine only): not readable through SDK 3.9.9. Measured on "
                "SimMachine: enforced for MoveJ, not for ServoJ.",
                "4. **Physical / model maximum** (cannot be changed): FR20 J1–J3 120 deg/s, J4–J6 180 deg/s, "
                "TCP 2 m/s typical.",
            ]
        )
        allowed = robot is not None and robot.safety_config_writes_allowed
        self._safety_info.content = (
            "**Changes the FAIRINO safety configuration.** Current values cannot be read back through "
            "SDK 3.9.9 — verify in the WebApp after every change. Automatic speed limiting is only "
            "available in manual-mode enable. "
            + (
                "Writes are **ALLOWED** by the hardware parameter."
                if allowed
                else "Writes are **refused**: launch with `allow_safety_config_writes:=true` (SimMachine only)."
            )
        )

    def _update_audit(self, events) -> None:
        if not events:
            return
        lines = ["**Audit log** (newest first; also in the ROS log as `AUDIT`)", ""]
        for e in events:
            stamp = time.strftime("%H:%M:%S", time.localtime(e.stamp.sec))
            change = f" `{e.old_value}` → `{e.new_value}`" if (e.old_value or e.new_value) else ""
            code = f" code {e.result_code}" if e.result_code else ""
            lines.append(
                f"- {'✓' if e.success else '✗'} **{stamp}** {e.source.replace('fairino_', '')} · "
                f"{e.category} · {e.action}{change}{code} — {e.message}"
            )
        self._audit_markdown.content = "\n".join(lines)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FairinoViserUi()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
