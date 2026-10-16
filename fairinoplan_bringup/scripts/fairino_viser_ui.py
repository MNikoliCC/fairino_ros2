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
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory

import fairino_trajectory_tools as tt
from fairino_viser_trajectories import PathEditor, RecordingPanel, axes_options, resolve_trajectory_dir


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
STATE_ICON = {
    "ACTIVE": "🟢", "HOLD": "🟡", "DISABLED": "🔵", "SYNCING": "🔄", "RECOVERING": "🔄",
    "FAULT": "🟠", "LOCKOUT": "🔴",
}
PLAN_ICON = {"planned": "🟦", "planning": "⏳", "executing": "▶️", "succeeded": "✅", "failed": "❌", "checked": "✔️"}
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


# Small frame-rate readout (requestAnimationFrame rate of the page, which is
# the render rate of the 3D view). Viser renders GUI HTML with innerHTML, so
# the script runs from an image error handler; it installs itself once.
FPS_OVERLAY_HTML = """<img src="data:," style="display:none" onerror="
if(!document.getElementById('fps-overlay')){
const d=document.createElement('div');d.id='fps-overlay';
d.style.cssText='position:fixed;left:64px;bottom:10px;z-index:1000;font:11px monospace;color:#888;pointer-events:none';
document.body.appendChild(d);let n=0,t0=performance.now(),worst=0,last=t0;
const f=(t)=>{n++;worst=Math.max(worst,t-last);last=t;
if(t-t0>=1000){d.textContent=(n*1000/(t-t0)).toFixed(0)+' fps · max frame '+worst.toFixed(0)+' ms';n=0;t0=t;worst=0;}
requestAnimationFrame(f);};requestAnimationFrame(f);}">"""



POINT_DEFAULT_M = (-3.0, 0.0, 0.0)  # point tracking / Orient to point [m, base_link]
POINT_TRACKING_TURN_RATE = np.radians(45.0)  # servoing: turn toward the point at most this fast [rad/s]



def _set_button(button, label: str, color: str | None, icon) -> None:
    """Changes a button only when needed (each change is sent to the browser)."""
    if button.label != label:
        button.label = label
    if button.color != color:
        button.color = color
    if button.icon != icon:
        button.icon = icon


def tracking_text(tracking: dict) -> str:
    """One discreet line about the last Execute, plus a warning when the
    robot left the plan (and why, when the ServoJ rate explains it)."""
    kind = {"joint_goal": "plan", "joint_path": "replay", "cartesian_path": "path"}.get(tracking.get("kind"), "motion")
    text = (
        f"*Last execution ({kind}): {tracking['max_deviation_mm']:.1f} mm max / "
        f"{tracking['mean_deviation_mm']:.1f} mm mean from the plan · {tracking['executed_s']:.2f} s "
        f"(planned {tracking['planned_s']:.2f} s)"
    )
    low_rate = None
    expected = tracking.get("servoj_expected_rate")
    if tracking.get("servoj_min_rate") is not None and expected:
        text += f" · ServoJ ≥ {tracking['servoj_min_rate']}/s"
        if tracking["servoj_min_rate"] < 0.95 * expected:
            low_rate = tracking["servoj_min_rate"]
    text += "*"
    if tracking["max_deviation_mm"] > 5.0:
        text += f"  \n⚠️ the robot left the planned path by {tracking['max_deviation_mm']:.0f} mm ({tracking['worst_at_s']:.1f} s in)"
        if low_rate is not None:
            text += (
                f": the controller executed only {low_rate} of {expected} ServoJ points/s, so the joints reached "
                f"at most {100 * low_rate / expected:.0f} % of their speed"
                + (
                    " (SimMachine: start with servoj_cmd_period:=0.002 servoj_max_queue:=1)"
                    if expected > 600
                    else " (the SimMachine is short of CPU: reduce other load on the host or give the segments more time)"
                )
            )
        else:
            text += "; give the segments more time"
    return text

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
        # Tool pointing down, wrist away from singularities (j5 = -90).
        self.declare_parameter("home_joints_deg", [0.0, -90.0, 90.0, -90.0, -90.0, 0.0])
        # Recordings and paths: <trajectory_dir>/recordings, <trajectory_dir>/paths.
        # Empty: $FAIRINO_TRAJECTORY_DIR, /ros2_ws/trajectories (host bind mount) or ~/.ros.
        self.declare_parameter("trajectory_dir", "")
        self.declare_parameter("record_rate_hz", 125.0)
        self._robot_status_required = bool(self.get_parameter("robot_status_required").value)

        # Same URDF as RoboPlan.
        description_share = get_package_share_path("fairino_description")
        self._urdf_path = (description_share / "urdf" / "fairino20_v6.urdf").as_posix()
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
        # Servo point tracking: world rotation of the last tracking target
        # (the next one is reached from it by the smallest rotation).
        self._look_rotation: np.ndarray | None = None
        self._marker_dragging = False
        self._tracking_on = False
        self._editor_shows_gizmo = True
        # Gizmo axes: "tool" = the gizmo is the target pose (rotate it);
        # "base" = base_link axes, move only, the target keeps
        # _target_rotation (shown by small axes). Point tracking uses "base".
        self._axes_mode = "tool"
        self._target_rotation: np.ndarray | None = None
        self._look_time = 0.0
        self._look_updating = False
        self._last_servo_state = ""
        self._servo: ServoStatus | None = None
        self._robot: RobotStatus | None = None
        self._plan_status: dict = {"state": "unknown", "message": "planner not running"}
        self._events: deque = deque(maxlen=60)
        self._servoj_counts: deque = deque()
        self._last_tracking: dict | None = None
        # Planned trajectory for the preview: times [s] and joint positions (N x 6, JOINT_NAMES).
        self._preview_times: np.ndarray | None = None
        self._preview_positions: np.ndarray | None = None
        self._preview_start = 0.0
        self._preview_shown = False
        self._last_result = "—"
        self._start_time = self.get_clock().now()

        # ROS interfaces.
        self._target_pub = self.create_publisher(PoseStamped, "/fairino_servo/target_pose", 1)
        # Point tracking: the servo tracks the position and the tool z axis
        # only (roll free, chosen by OInK within the joint limits).
        self._free_roll_client = self.create_client(SetBool, "/fairino_servo/set_free_tool_roll")
        self._free_roll_sent = 0.0  # time of the last request
        self._free_roll_started = False  # full pose requested once at UI start
        self._plan_goal_pub = self.create_publisher(PoseStamped, "/fairinoplan/pose_goal", 1)
        self._joint_goal_pub = self.create_publisher(JointState, "/fairinoplan/joint_goal", 1)
        self.path_goal_pub = self.create_publisher(String, "/fairinoplan/path_goal", 1)
        self.path_check_pub = self.create_publisher(String, "/fairinoplan/path_check", 1)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 1)
        self.create_subscription(ServoStatus, "/fairino_servo/status", self._on_servo_status, 5)
        self.create_subscription(RobotStatus, "/fairino_robot/status", self._on_robot_status, 5)
        self.create_subscription(
            PoseStamped, "/fairino_servo/reset_target_pose", self._on_reset_target, LATCHED
        )
        self.create_subscription(AuditEvent, "/fairino_robot/events", self._on_event, EVENTS_QOS)
        self.create_subscription(AuditEvent, "/fairino_servo/events", self._on_event, EVENTS_QOS)
        self.create_subscription(String, "/fairinoplan/status", self._on_plan_status, LATCHED)
        self.create_subscription(String, "/fairinoplan/path_check_result", self._on_path_check, 10)
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

        # Interface used by the trajectory panels.
        self.server = self._server
        self.model = self._model
        self.joint_q_index = self._joint_q_index
        self.base_frame = self._base_frame
        self.tip_frame = self._tip_frame
        directory = resolve_trajectory_dir(str(self.get_parameter("trajectory_dir").value))
        self.get_logger().info(f"Trajectories and paths in {directory}")
        self.path_editor = PathEditor(self, directory)
        self.recorder = RecordingPanel(
            self, directory, self._urdf_path, float(self.get_parameter("record_rate_hz").value)
        )

        self._build_scene()
        self._build_gui()

        self.create_timer(1.0 / float(self.get_parameter("display_frequency").value), self._display)
        self.create_timer(1.0 / float(self.get_parameter("display_frequency").value), self._animate_preview)
        self.create_timer(
            1.0 / float(self.get_parameter("heartbeat_frequency").value), self._heartbeat
        )
        self.create_timer(0.5, self._update_panels)
        self.create_timer(0.5, self._safe(self.recorder.tick))
        self.create_timer(1.0 / 15.0, self._safe(self.path_editor.tick))
        self.create_timer(0.05, self._safe(self._tracking_tick))
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
        self._marker.on_drag_start(lambda _: setattr(self, "_marker_dragging", True))
        self._marker.on_drag_end(lambda _: self._on_marker_drag_end())
        # Servo point tracking: the point (red sphere, translation gizmo), the
        # resulting tool orientation (axes at the target) and a line to the
        # point; only while tracking is on.
        self._track_point = np.asarray(POINT_DEFAULT_M, dtype=float)  # world (= base_link here)
        self._look_gizmo = scene.add_icosphere(
            "/fairino/servo_tracking/point", radius=0.02, color=(230, 40, 40), visible=False
        )
        self._look_axes = scene.add_frame(
            "/fairino/servo_tracking/tool", axes_length=0.12, axes_radius=0.005, visible=False
        )
        self._look_line = scene.add_line_segments(
            "/fairino/servo_tracking/line", points=np.zeros((1, 2, 3)), colors=(230, 40, 40), thickness=0.0015,
            visible=False,
        )
        # Measured TCP (flange) axes; the gizmo shows the target.
        self._tcp_axes = scene.add_frame("/fairino/tcp_measured", axes_length=0.08, axes_radius=0.004)
        # base_link axes: the frame of all poses and offsets in the UI.
        data = self._model.createData()
        pin.framesForwardKinematics(self._model, data, pin.neutral(self._model))
        base = data.oMf[self._base_frame]
        self._base_rotation = base.rotation.copy()
        self._track_point = base.act(np.asarray(POINT_DEFAULT_M, dtype=float))
        self._look_gizmo.position = tuple(self._track_point)
        base_q = pin.Quaternion(base.rotation)
        scene.add_frame(
            "/fairino/base_link", axes_length=0.25, axes_radius=0.006, position=base.translation.copy(),
            wxyz=(base_q.w, base_q.x, base_q.y, base_q.z),
        )
        scene.add_label("/fairino/base_link/label", "base_link", position=(0.0, 0.0, -0.03))
        self._path_handle = None

    # -------------------------------------------------------------------- gui
    def _build_gui(self) -> None:
        gui = self._server.gui
        gui.configure_theme(
            control_width="large", brand_color=(30, 110, 200), show_share_button=False, show_logo=False
        )
        # Browser frame rate, bottom left of the 3D view (measured by the page).
        gui.add_html(FPS_OVERLAY_HTML)
        self._summary = gui.add_markdown("**Servo:** starting…")
        self._result_markdown = gui.add_markdown("**Last command:** —")
        tabs = gui.add_tab_group()

        with tabs.add_tab("Control", icon=viser.Icon.ROBOT):
            # Point tracking (servo): the tool z axis keeps pointing at a point
            # given in base_link (paths have their own, in the Path tab).
            with gui.add_folder("Point tracking"):
                self._track_button = gui.add_button(
                    "Enable Point Tracking", icon=viser.Icon.TARGET,
                    hint="The gizmo moves the TCP (base_link axes), the tool z axis keeps looking at the point "
                    "(red sphere; set its position below). Path › Orient to point uses the same point.",
                )
                self._look_vector = gui.add_vector3(
                    "Point [mm]", tuple(1000.0 * v for v in POINT_DEFAULT_M), step=1.0, hint="base_link",
                    visible=False,
                )
                self._look_info = gui.add_markdown("", visible=False)
            # One button enables/disables the servo (plan mode when disabled),
            # one holds/resumes it.
            with gui.add_folder("Servo"):
                self._enable_button = gui.add_button("Enable Servo", color="green", icon=viser.Icon.HAND_STOP)
                self._hold_button = gui.add_button(
                    "Servo Hold", color="yellow", icon=viser.Icon.PLAYER_PAUSE,
                    hint="Hold: stop tracking the gizmo, the servo keeps the joints; Resume: track it again",
                )
                self._reset_target_button = gui.add_button("Reset Target to Current TCP", icon=viser.Icon.TARGET)
                # Same setting as in the Path tab (TCP and waypoint gizmo).
                self._axes_control = gui.add_button_group("Gizmo axes", axes_options("tool"))
            with gui.add_folder("Plan (RRT + TOPP-RA)"):
                self._plan_button = gui.add_button("Plan", icon=viser.Icon.ROUTE_2, hint="Plan to the gizmo pose (plan mode only)")
                self._home_button = gui.add_button(
                    "Home", icon=viser.Icon.HOME, hint="Plan to the home joint pose; check the preview, then Execute"
                )
            # The plan waiting for Execute, from any source: Plan, Home, a
            # replay (Record tab) or a path (Path tab).
            with gui.add_folder("Planned motion"):
                self._plan_markdown = gui.add_markdown("—")
                self._preview_checkbox = gui.add_checkbox(
                    "Preview planned motion", True, hint="Translucent robot replays the plan before Execute"
                )
                self._preview_speed = gui.add_slider("Preview speed", min=0.25, max=2.0, step=0.25, initial_value=1.0)
                self._execute_button = gui.add_button("Execute", color="blue", icon=viser.Icon.PLAYER_PLAY)
                # How well the last Execute followed its plan.
                self._tracking_markdown = gui.add_markdown("")
            with gui.add_folder("Fault"):
                self._fault_markdown = gui.add_markdown("—")
                self._recover_button = gui.add_button(
                    "Recover",
                    color="orange",
                    icon=viser.Icon.FIRST_AID_KIT,
                    hint="FAULT: controlled recovery; LOCKOUT or unknown fault: operator reset once the cause is gone",
                )

        with tabs.add_tab("Record", icon=viser.Icon.PLAYER_RECORD):
            self.recorder.build_gui(gui)

        with tabs.add_tab("Path", icon=viser.Icon.VECTOR_BEZIER_2):
            self.path_editor.build_gui(gui)

        with tabs.add_tab("Status", icon=viser.Icon.LIST):
            self._status_markdown = gui.add_markdown("waiting for status…")
            self._timing_markdown = gui.add_markdown("")

        with tabs.add_tab("Speed", icon=viser.Icon.SETTINGS):
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

        with tabs.add_tab("Audit", icon=viser.Icon.TIMELINE):
            self._audit_markdown = gui.add_markdown("no events yet")

        self._enable_button.on_click(lambda _: self._on_servo_button())
        self._hold_button.on_click(lambda _: self._on_hold_button())
        self._reset_target_button.on_click(lambda _: self._reset_target())
        self._track_button.on_click(lambda _: self._toggle_tracking())
        self._axes_control.on_click(lambda e: self.set_gizmo_axes("base" if "base_link" in str(e.target.value) else "tool"))
        self._look_vector.on_update(lambda _: self._on_look_vector())
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
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0.0:
            stamp = time.monotonic()
        self.recorder.on_joint_state(stamp, np.asarray(values))

    def _on_servo_status(self, msg: ServoStatus) -> None:
        with self._lock:
            self._servo = msg

    def _on_robot_status(self, msg: RobotStatus) -> None:
        with self._lock:
            self._robot = msg
            # Executed ServoJ points per second (counter modulo 10000).
            now = time.monotonic()
            self._servoj_counts.append((now, msg.servoj_cmd_num))
            while len(self._servoj_counts) > 2 and now - self._servoj_counts[0][0] > 1.0:
                self._servoj_counts.popleft()

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
            if status.get("tracking"):
                self._last_tracking = dict(status["tracking"], kind=status.get("kind", ""), state=status.get("state"))
        self.path_editor.on_plan_status(status)

    def _on_path_check(self, msg: String) -> None:
        try:
            result = json.loads(msg.data)
        except ValueError:
            return
        self.path_editor.on_check(result)

    def reset_gizmo_to_robot(self) -> None:
        self._reset_target()

    def set_tcp_gizmo_visible(self, visible: bool) -> None:
        """The Path editor hides the TCP gizmo in Edit mode; in servo mode it
        is always shown (it is what the robot follows)."""
        self._editor_shows_gizmo = bool(visible)
        self._apply_gizmo_visibility(self._servo)

    def _apply_gizmo_visibility(self, servo) -> None:
        servoing = servo is not None and servo.mode == "servo" and servo.state != "DISABLED"
        visible = self._editor_shows_gizmo or servoing
        if self._marker.visible != visible:
            self._marker.visible = visible

    def set_gizmo_pose(self, position: np.ndarray, wxyz: np.ndarray) -> None:
        """Places the TCP gizmo (base_link pose) and publishes it as the
        target; in servo mode the robot follows at the servo limits."""
        with self._lock:
            if not self._have_joint_state:
                return
            world_base, _ = self._world_poses()
            target = pin.SE3(pin.Quaternion(*[float(v) for v in wxyz]).normalized().matrix(), np.asarray(position, dtype=float))
            self._set_marker(world_base * target)
        pose = self._marker_pose()
        if pose is not None:
            self._target_pub.publish(pose)

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
            # Plan cleared: remove the planned path and the preview.
            if self._path_handle is not None:
                self._path_handle.remove()
                self._path_handle = None
            with self._lock:
                self._preview_times = None
                self._preview_positions = None
            return
        indices = [msg.joint_names.index(name) for name in JOINT_NAMES if name in msg.joint_names]
        if len(indices) != len(JOINT_NAMES):
            return
        # TCP of every trajectory point, kept every ~2 mm of TCP travel: the
        # line lies on the trajectory (approach included) and stays smooth
        # where the robot is fast (sampling by time would leave long chords).
        with self._lock:
            q = self._q.copy()
            data = self._model.createData()
        tcp = np.empty((len(msg.points), 3))
        for k, point in enumerate(msg.points):
            for name, index in zip(JOINT_NAMES, indices):
                q[self._joint_q_index[name]] = point.positions[index]
            pin.framesForwardKinematics(self._model, data, q)
            tcp[k] = data.oMf[self._tip_frame].translation
        travel = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(tcp, axis=0), axis=1))])
        spacing = max(0.002, travel[-1] / 4000.0)
        keep = np.unique(np.searchsorted(travel, np.arange(0.0, travel[-1], spacing)))
        points = np.vstack([tcp[keep], tcp[-1:]]) if len(keep) else tcp[[0, -1]]
        if self._path_handle is not None:
            self._path_handle.remove()
        self._path_handle = self._server.scene.add_line_segments(
            "/fairino/planned_path", points=np.stack([points[:-1], points[1:]], axis=1),
            colors=(30, 120, 220), thickness=0.0025,
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
        # The target orientation; tracking continues from it.
        self._look_rotation = world_pose.rotation.copy()
        self._target_rotation = world_pose.rotation.copy()
        rotation = self._base_rotation if self._base_axes() else world_pose.rotation
        quaternion = pin.Quaternion(rotation)
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
            rotation = self._marker_rotation()
            world_target = pin.SE3(rotation, np.asarray(self._marker.position, dtype=float))
            world_target = self._look_at_target(world_target)
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
        if self._base_axes():
            self._show_tracking_target()

    def _marker_rotation(self) -> np.ndarray:
        """Target orientation from the gizmo. tool axes: the gizmo is the
        target. base_link axes: the gizmo rests at the base axes; its
        rotation away from them (during a drag) turns the kept target
        orientation about the base axes. Caller holds self._lock."""
        wxyz = np.asarray(self._marker.wxyz, dtype=float)
        gizmo = pin.Quaternion(wxyz[0], wxyz[1], wxyz[2], wxyz[3]).normalized().matrix()
        if not self._base_axes() or self._target_rotation is None:
            return gizmo
        if self._tracking_on:
            return self._target_rotation  # the point turns it (rotation rings are off)
        return gizmo @ self._base_rotation.T @ self._target_rotation

    def _base_axes(self) -> bool:
        return self._tracking_on or self._axes_mode == "base"

    def gizmo_axes(self) -> str:
        """"base" (base_link axes, move only) or "tool"; point tracking
        always uses base_link axes."""
        return "base" if self._base_axes() else "tool"

    def set_gizmo_axes(self, mode: str) -> None:
        """Gizmo axes for the TCP gizmo and the Path editor's waypoint gizmo
        (one setting, shown in the Control and the Path tab)."""
        if mode == self._axes_mode:
            return
        with self._lock:
            current = self._current_target_rotation()
        self._axes_mode = mode
        self._apply_marker_axes(current)
        self.path_editor.on_gizmo_axes(mode)
        self._axes_control.options = axes_options(mode)
        self._set_result(
            "Gizmo axes: base_link (move and rotate about the base axes)" if mode == "base"
            else "Gizmo axes: tool (move and rotate about the tool axes)"
        )

    def _current_target_rotation(self) -> np.ndarray | None:
        if not self._marker_initialized:
            return None
        if self._tracking_on and self._look_rotation is not None:
            return self._look_rotation.copy()
        if self._axes_mode == "base" and self._target_rotation is not None:
            return self._marker_rotation().copy()
        return pin.Quaternion(*[float(v) for v in self._marker.wxyz]).normalized().matrix()

    def _apply_marker_axes(self, rotation: np.ndarray | None) -> None:
        """Shows the gizmo with base_link axes (rotation rings off) or as the
        target pose, without changing the target."""
        base = self._base_axes()
        if rotation is not None:
            self._target_rotation = rotation.copy()
            self._look_rotation = rotation.copy()
        q = pin.Quaternion(self._base_rotation if base or rotation is None else rotation)
        self._suppress_marker_event = True
        try:
            self._marker.disable_rotations = self._tracking_on  # base_link and tool both rotate
            self._marker.wxyz = np.array([q.w, q.x, q.y, q.z])
        finally:
            self._suppress_marker_event = False
        self._look_axes.visible = base
        self._look_line.visible = self._tracking_on
        if base:
            self._show_tracking_target()

    def tracking_on(self) -> bool:
        return self._tracking_on

    def tracking_point(self) -> np.ndarray:
        """The point (base_link) of point tracking and of Orient to point."""
        return self._base_point(self._track_point)

    def _base_point(self, world: np.ndarray) -> np.ndarray:
        base = pin.SE3(self._base_rotation, np.zeros(3))
        return base.inverse().act(world)

    # ---------------------------------------------------- servo point tracking
    def _look_at_target(self, world_target: pin.SE3) -> pin.SE3:
        """Point tracking: the target orientation turned (smallest rotation
        from the previous tracking target) so that its z axis points at the
        point; while servoing it turns there at most POINT_TRACKING_TURN_RATE
        (switching tracking on, or a point typed far away, must not swing the
        robot round at once). The roll about the tool axis is free in the servo
        (free_tool_roll: OInK picks it within the joint limits); here it
        follows the robot's own roll, so the target axes show what the robot
        does and plans keep the robot's roll. Caller holds self._lock."""
        if not self._tracking_on:
            return world_target
        previous = self._look_rotation if self._look_rotation is not None else world_target.rotation
        _, world_tip = self._world_poses()
        previous = tt.rotation_between(world_tip.rotation[:, 2], previous[:, 2]) @ world_tip.rotation
        desired = tt.look_at_rotation(previous, world_target.translation, self._look_point_world())
        now = time.monotonic()
        servo = self._servo
        if servo is not None and servo.state == "ACTIVE":
            step = POINT_TRACKING_TURN_RATE * float(np.clip(now - self._look_time, 0.0, 0.1))
            delta = pin.log3(previous.T @ desired)
            angle = float(np.linalg.norm(delta))
            rotation = previous @ pin.exp3(delta * (step / angle)) if angle > step else desired
        else:
            rotation = desired
        self._look_time = now
        self._look_rotation = rotation
        return pin.SE3(rotation, world_target.translation.copy())

    def _tracking_tick(self) -> None:
        """While the servo turns toward the point, keep publishing."""
        servo = self._servo
        if not self._tracking_on or servo is None or servo.state != "ACTIVE" or self._look_rotation is None:
            return
        with self._lock:
            if not self._marker_initialized:
                return
            desired = tt.look_at_rotation(
                self._look_rotation, np.asarray(self._marker.position, dtype=float), self._look_point_world()
            )
        if np.linalg.norm(pin.log3(self._look_rotation.T @ desired)) > np.radians(0.2):
            self._on_marker_moved()

    def _look_point_world(self) -> np.ndarray:
        return self._track_point.copy()

    def _tool_axis_error_deg(self) -> float | None:
        """Angle between the robot's tool z axis and the direction to the point."""
        with self._lock:
            if not self._have_joint_state:
                return None
            _, world_tip = self._world_poses()
        direction = self._look_point_world() - world_tip.translation
        if np.linalg.norm(direction) < 1e-6:
            return None
        cos = float(world_tip.rotation[:, 2] @ direction / np.linalg.norm(direction))
        return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))

    def _toggle_tracking(self) -> None:
        """On: only once the servo confirms the free tool roll (otherwise the
        servo would keep the UI's roll and could run j6 into its limit).
        Off: at once; the servo is told to track the full pose again."""
        self._free_roll_sent = time.monotonic()
        if not self._tracking_on:
            self._call(
                self._free_roll_client, SetBool.Request(data=True), "Point tracking",
                on_done=lambda response: self._set_tracking(True) if response.success else None,
            )
            return
        self._call(self._free_roll_client, SetBool.Request(data=False), "Point tracking")
        self._set_tracking(False)

    def _set_tracking(self, on: bool) -> None:
        if on == self._tracking_on:
            return
        with self._lock:
            current = self._current_target_rotation()
            self._look_time = time.monotonic()
            if self._tracking_on and self._have_joint_state:
                # Switching off: the target takes the robot's tool orientation
                # (plan mode: the robot did not turn; servo: its roll was chosen
                # by OInK), so the full-pose servo starts exactly where it is.
                _, world_tip = self._world_poses()
                current = world_tip.rotation.copy()
        self._tracking_on = on
        _set_button(
            self._track_button, "Disable Point Tracking" if self._tracking_on else "Enable Point Tracking",
            "red" if self._tracking_on else None, viser.Icon.TARGET,
        )
        for handle in (self._look_gizmo, self._look_vector, self._look_info):
            handle.visible = self._tracking_on
        self._axes_control.visible = not self._tracking_on  # always base_link while tracking
        self._apply_marker_axes(current)
        self.path_editor.on_gizmo_axes(self.gizmo_axes())
        if self._tracking_on:
            self._sync_point_vector()
            self._on_marker_moved()  # the target turns toward the point
            self._set_result("Point tracking: the tool turns toward the red sphere (set its position under Point [mm])")
        else:
            self._set_result("Point tracking: off (target orientation: the robot's tool, as it is now)")

    def _sync_point_vector(self) -> None:
        point = self.tracking_point()
        self._look_updating = True
        try:
            self._look_vector.value = tuple(round(1000.0 * float(v), 1) for v in point)
        finally:
            self._look_updating = False

    def _on_look_vector(self) -> None:
        if self._look_updating:
            return
        base = pin.SE3(self._base_rotation, np.zeros(3))
        world = base.act(np.asarray(self._look_vector.value, dtype=float) / 1000.0)
        if np.linalg.norm(world - self._track_point) < 5e-4:
            return  # echo of a value set from here
        self._track_point = world
        self._look_gizmo.position = tuple(world)
        if self._tracking_on:
            self._on_marker_moved()  # the target follows the point

    def _on_marker_drag_end(self) -> None:
        self._marker_dragging = False
        if self._base_axes() and not self._tracking_on:
            # base_link axes: keep the turned orientation, the gizmo goes back
            # to the base axes (shown by the small axes at the target).
            with self._lock:
                if self._target_rotation is None:
                    return
                rotation = self._marker_rotation()
                self._target_rotation = rotation
                q = pin.Quaternion(self._base_rotation)
                self._suppress_marker_event = True
                try:
                    self._marker.wxyz = np.array([q.w, q.x, q.y, q.z])
                finally:
                    self._suppress_marker_event = False
            self._show_tracking_target()

    def _show_tracking_target(self) -> None:
        with self._lock:
            if self._tracking_on:
                rotation = self._look_rotation
            else:
                rotation = self._marker_rotation() if self._target_rotation is not None else None
        if rotation is None:
            return
        position = np.asarray(self._marker.position, dtype=float)
        q = pin.Quaternion(rotation)
        self._look_axes.position = tuple(position)
        self._look_axes.wxyz = (q.w, q.x, q.y, q.z)
        if self._tracking_on:
            self._look_line.points = np.array([[position, self._look_point_world()]])

    def _update_look_at(self, servo) -> None:
        if not self._free_roll_started and self._free_roll_client.service_is_ready():
            # This UI (re)started without point tracking: the servo must not be
            # left in the free-roll mode of an earlier session.
            self._free_roll_started = True
            self._free_roll_sent = time.monotonic()
            self._free_roll_client.call_async(SetBool.Request(data=False))
        if (
            self._tracking_on and servo is not None and not servo.free_tool_roll
            and time.monotonic() - self._free_roll_sent > 2.0 and self._free_roll_client.service_is_ready()
        ):
            # Tracking is on but the servo restarted (it starts with the full
            # pose): request the free roll again. Never forced off here, so
            # another client (e.g. the benchmark) may use the mode.
            self._free_roll_sent = time.monotonic()
            self._free_roll_client.call_async(SetBool.Request(data=True))
        if not self._tracking_on:
            return
        with self._lock:
            target = np.asarray(self._marker.position, dtype=float)
        error = self._tool_axis_error_deg()
        self._look_info.content = (
            f"*{np.linalg.norm(self._look_point_world() - target):.2f} m from the target"
            + (f" · robot tool axis {error:.0f}° off" if error is not None else "")
            + "*"
        )

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
        self._apply_gizmo_visibility(servo)
        self._update_look_at(servo)
        self._update_buttons(servo, robot, plan)
        self._update_status(servo, robot)
        self._update_speed(servo, robot)
        self._plan_markdown.content = self._plan_text(plan)
        with self._lock:
            tracking = self._last_tracking
        self._tracking_markdown.content = tracking_text(tracking) if tracking else ""
        plan_mode = servo is not None and servo.mode == "plan" and servo.state == "DISABLED"
        self.recorder.refresh(plan_mode)
        self.path_editor.refresh(plan_mode)
        self._update_audit(events)

    def _servoj_rate_text(self, robot: RobotStatus) -> str:
        with self._lock:
            counts = list(self._servoj_counts)
        if len(counts) < 2 or counts[-1][0] - counts[0][0] < 0.2:
            return "—"
        executed = sum((b[1] - a[1]) % 10000 for a, b in zip(counts, counts[1:]))
        rate = executed / (counts[-1][0] - counts[0][0])
        expected = 1.0 / robot.servoj_cmd_period_s if robot.servoj_cmd_period_s > 0 else 0.0
        text = (
            f"{rate:.0f} points/s (cmdT {1000.0 * robot.servoj_cmd_period_s:.0f} ms) · queue "
            f"{robot.servoj_queue_estimate}/{robot.servoj_max_queue} · skips {robot.servoj_flow_skips} · "
            f"limited points {robot.servoj_point_limited}"
        )
        moving = len(robot.joint_velocity_deg_s) and max(abs(v) for v in robot.joint_velocity_deg_s) > 1.0
        if robot.streaming_state == RobotStatus.STREAMING_ARMED and moving and rate < 0.85 * expected:
            text += f" · ⚠️ below {expected:.0f}/s: joints reach at most {100.0 * rate / expected:.0f} % of v_max"
        return text

    @staticmethod
    def _plan_text(plan: dict) -> str:
        state = plan.get("state", "?")
        text = f"{PLAN_ICON.get(state, '•')} **{state}** — {plan.get('message', '')}"
        if "points" in plan and state in ("planned", "executing", "succeeded"):
            kind = {"joint_goal": "plan", "joint_path": "replay", "cartesian_path": "path"}.get(plan.get("kind"), "plan")
            text += f"  \n{kind}: {plan.get('trajectory_duration_s', 0.0):.2f} s"
            if plan.get("approach_duration_s"):
                text += f" (approach {plan['approach_duration_s']:.2f} s + {plan.get('path_duration_s', 0.0):.2f} s)"
            if plan.get("time_scale", 1.0) > 1.001:
                text += f" · slowed ×{plan['time_scale']:.2f} to stay within the joint limits"
            text += f" · planning {plan.get('planning_time_s', 0.0):.2f} s"
        return text

    def _safe(self, function):
        def wrapper():
            try:
                function()
            except Exception as exc:  # noqa: BLE001 - a display bug must not stop the UI
                self.get_logger().error(f"{function.__qualname__} failed: {exc}", throttle_duration_sec=5.0)

        return wrapper

    # ------------------------------------------------- interface for the panels
    def servo_status(self) -> ServoStatus | None:
        return self._servo

    def robot_status(self) -> RobotStatus | None:
        return self._robot

    def set_result(self, text: str) -> None:
        self._set_result(text)

    def plan_mode_ready(self, label: str) -> bool:
        servo = self._servo
        if servo is not None and (servo.state != "DISABLED" or servo.mode != "plan"):
            self._set_result(f"{label}: only in plan mode with the servo DISABLED")
            return False
        return True

    def gizmo_pose(self):
        pose = self._marker_pose()
        if pose is None:
            return None
        p, o = pose.pose.position, pose.pose.orientation
        return np.array([p.x, p.y, p.z]), np.array([o.w, o.x, o.y, o.z])

    def robot_q(self) -> np.ndarray | None:
        with self._lock:
            if not self._have_joint_state:
                return None
            return np.array([self._q[self._joint_q_index[name]] for name in JOINT_NAMES])

    def planned_start(self) -> np.ndarray | None:
        """First configuration of the current plan (JOINT_NAMES order)."""
        with self._lock:
            return None if self._preview_positions is None else self._preview_positions[0].copy()

    def plan_status(self) -> dict:
        with self._lock:
            return dict(self._plan_status)

    def robot_tcp_pose(self):
        with self._lock:
            if not self._have_joint_state:
                return None
            world_base, world_tip = self._world_poses()
        tip = world_base.inverse() * world_tip
        quaternion = pin.Quaternion(tip.rotation)
        return tip.translation.copy(), np.array([quaternion.w, quaternion.x, quaternion.y, quaternion.z])

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
            f"### {STATE_ICON.get(servo.state, '⚪')} {servo.state} · {servo.mode} mode\n"
            f"controller `{servo.active_position_controller}` · FAIRINO {connection}  \n{servo.reason}"
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
        # Enable ↔ Disable Servo, Servo Hold ↔ Resume Servo.
        enable = state == "DISABLED"
        _set_button(self._enable_button, "Enable Servo" if enable else "Disable Servo",
                    "green" if enable else "blue", viser.Icon.HAND_STOP if enable else viser.Icon.ROUTE)
        self._enable_button.disabled = state in ("RECOVERING", "SYNCING", "UNKNOWN")
        hold = state != "HOLD"
        _set_button(self._hold_button, "Servo Hold" if hold else "Resume Servo",
                    "yellow" if hold else "green", viser.Icon.PLAYER_PAUSE if hold else viser.Icon.PLAYER_PLAY)
        self._hold_button.disabled = state not in ("ACTIVE", "HOLD")
        self._recover_button.disabled = state not in ("FAULT", "LOCKOUT")
        self._plan_button.disabled = not plan_mode
        self._home_button.disabled = not plan_mode
        self._execute_button.disabled = not plan_mode or plan.get("state") != "planned"
        self._safety_button.disabled = robot is None or not robot.safety_config_writes_allowed

    def _on_servo_button(self) -> None:
        # Act on what the button shows (the state may change meanwhile).
        if self._enable_button.label == "Enable Servo":
            self._trigger("enable")
        else:
            self._trigger("disable", "Disable Servo")

    def _on_hold_button(self) -> None:
        if self._hold_button.label == "Servo Hold":
            self._trigger("hold", "Servo Hold")
        else:
            self._trigger("enable", "Resume Servo")

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
                f"| Orientation task | {'tool axis only (roll free, point tracking)' if servo.free_tool_roll else 'full pose'} |",
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
                f"| ServoJ executed by the controller | {self._servoj_rate_text(robot)} |",
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
