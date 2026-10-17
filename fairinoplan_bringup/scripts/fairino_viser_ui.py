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
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory

from fairino_viser_gizmo import TargetGizmo
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



VIEWS = (("servo", "Servo"), ("trajectory", "Trajectory"), ("files", "Files"))


def view_options(view: str) -> tuple[str, ...]:
    """The view switch: check boxes, exactly one ticked."""
    return tuple(f"{'☑' if key == view else '☐'} {label}" for key, label in VIEWS)


def view_from_label(label: str) -> str:
    return next(key for key, name in VIEWS if label.endswith(name))


def axes_message(mode: str) -> str:
    return (
        "Gizmo axes: base_link (move and rotate about the base axes)" if mode == "base"
        else "Gizmo axes: tool (move and rotate about the tool axes)"
    )

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
        # Viser's Share button: a public link through Viser's relay server;
        # whoever has the link sees and can operate this UI (the robot).
        self.declare_parameter("show_share_button", True)
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
        # Two views: "servo" (servo mode, servo gizmo) and "trajectory" (plan
        # mode, trajectory gizmo, paths, plans, Execute).
        self._view = "trajectory"  # shown: servo | trajectory | files
        self._mode_view = "trajectory"  # working view (mode, gizmo): servo | trajectory
        self._homing = False  # Home (plan + execute) running
        self._last_joint_plan = "home"  # what the last joint-goal plan was
        self._view_initialized = False
        self._tracking_on = False  # point tracking, shared by both views
        self._editor_shows_gizmo = True  # the Path editor hides the trajectory gizmo in Edit mode
        self._tip_rotation: np.ndarray | None = None  # robot tool orientation (world), cached at display rate
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
        self._speed_pub = self.create_publisher(Float64, "/fairinoplan/speed_scale", LATCHED)
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
        self.create_timer(PathEditor.SKETCH_DT_S, self._safe(self._sketch_tick))
        self._publish_speed()  # the planner starts each UI session at the slider's speed
        self.get_logger().info(
            "Viser UI ready at http://localhost:%d" % int(self.get_parameter("port").value)
        )

    # ------------------------------------------------------------------ scene
    def _build_scene(self) -> None:
        scene = self._server.scene
        scene.add_grid("/ground", width=4.0, height=4.0, cell_size=0.25, position=(0.0, 0.0, 0.0))
        self._server.initial_camera.position = (1.6, -2.6, 2.0)
        self._server.initial_camera.look_at = (-0.4, 0.0, 0.7)
        # Point tracking: the point (red sphere, set by coordinates), shown
        # while tracking is on.
        self._track_point = np.asarray(POINT_DEFAULT_M, dtype=float)  # world (= base_link here)
        self._look_gizmo = scene.add_icosphere(
            "/fairino/tracking_point", radius=0.02, color=(230, 40, 40), visible=False
        )
        # Measured TCP (flange) axes; the gizmo shows the target.
        self._tcp_axes = scene.add_frame("/fairino/tcp_measured", axes_length=0.08, axes_radius=0.004)
        # base_link axes: the frame of all poses and offsets in the UI.
        data = self._model.createData()
        pin.framesForwardKinematics(self._model, data, pin.neutral(self._model))
        base = data.oMf[self._base_frame]
        self._base_pose = base.copy()
        self._base_rotation = base.rotation.copy()
        self._track_point = base.act(np.asarray(POINT_DEFAULT_M, dtype=float))
        self._look_gizmo.position = tuple(self._track_point)
        # Arrows: one axis; squares: translation in a plane; rings: rotation.
        # Servo gizmo: the target the servo follows (synchronized with the
        # robot on every enable); with point tracking it keeps the robot's
        # roll and turns toward the point at most POINT_TRACKING_TURN_RATE.
        self._servo_gizmo = TargetGizmo(
            scene, "/fairino/tcp_target", base_rotation=self._base_rotation, point=lambda: self._track_point,
            roll_source=lambda: self._tip_rotation, turn_rate=POINT_TRACKING_TURN_RATE,
            turning=lambda: self._servo is not None and self._servo.state == "ACTIVE",
            on_change=self._publish_servo_target,
            on_hold=self.path_editor.servo_hold,  # drawing in the servo view
        )
        # Trajectory gizmo: drawing and planning; the robot does not follow it.
        self._plan_gizmo = TargetGizmo(
            scene, "/fairino/plan_target", base_rotation=self._base_rotation, point=lambda: self._track_point,
        )
        self._servo_gizmo.set_visible(False)
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
            control_width="large", brand_color=(30, 110, 200), show_logo=False, dark_mode=True,
            show_share_button=bool(self.get_parameter("show_share_button").value),
        )
        # Browser frame rate, bottom left of the 3D view (measured by the page).
        gui.add_html(FPS_OVERLAY_HTML)
        # Top left (floating window): state, controller, last command, and
        # the fault with Recover only when there is one.
        self._status_panel = gui.add_panel()
        with self._status_panel.add_tab("Robot", icon=viser.Icon.ROBOT):
            self._summary = gui.add_markdown("**Servo:** starting…")
            # What is planned / executed and how well the last Execute
            # followed its plan (here, so the working panel keeps its size).
            self._plan_markdown = gui.add_markdown("—")
            self._tracking_markdown = gui.add_markdown("")
            self._result_markdown = gui.add_markdown("**Last command:** —")
            self._fault_markdown = gui.add_markdown("", visible=False)
            self._recover_button = gui.add_button(
                "Recover", color="orange", icon=viser.Icon.FIRST_AID_KIT, visible=False,
                hint="FAULT: controlled recovery; LOCKOUT or unknown fault: operator reset once the cause is gone",
            )
        self._status_panel.float(x=12, y=12, width=380)

        # The views (like tabs): Servo and Trajectory also switch the robot's
        # mode; Files does not. Viser tabs cannot do that: the selected tab
        # is not reported to the server.
        self._view_select = gui.add_button_group("View", view_options("trajectory"))

        # Shared by all views: recording (servo and executed motion alike),
        # point tracking (one point; each view's gizmo reacts its own way)
        # and Home (plans and executes).
        self.recorder.build_controls(gui)
        self._track_button = gui.add_button(
            "Enable Point Tracking", icon=viser.Icon.TARGET,
            hint="The tool z axis keeps looking at the point (red sphere). Servo: the robot follows; Trajectory: "
            "the drawing gizmo and moved waypoints turn toward it; Look at point uses it too.",
        )
        self._look_vector = gui.add_vector3(
            "Point [mm]", tuple(1000.0 * v for v in POINT_DEFAULT_M), step=1.0, hint="base_link", visible=False,
        )
        self._look_info = gui.add_markdown("", visible=False)
        self._home_button = gui.add_button(
            "Home", icon=viser.Icon.HOME,
            hint="Plans and executes the motion to the home joint pose (from the servo view it returns to servoing)",
        )

        # ---- Servo view
        servo_folder = gui.add_folder("Servo")
        with servo_folder:
            self._enable_button = gui.add_button(
                "Enable Servo", color="green", icon=viser.Icon.HAND_STOP,
                hint="Shown when the servo is not active in this view (refused or after a fault): try again",
            )
            self._hold_button = gui.add_button(
                "Servo Hold", color="yellow", icon=viser.Icon.PLAYER_PAUSE,
                hint="Hold: stop tracking the gizmo, the servo keeps the joints; Resume: track it again",
            )
            self._axes_control = gui.add_button_group("Gizmo axes", axes_options("tool"))
            # Drawing: while the gizmo is held its motion is recorded from
            # the last waypoint (holding it still = a stop); Confirm Waypoint
            # turns it into waypoints and adds one at the gizmo.
            self._drawing_markdown = gui.add_markdown("")
            self._servo_confirm_button = gui.add_button(
                "Confirm Waypoint", color="green", icon=viser.Icon.CHECK,
                hint="Adds the motion drawn with the held gizmo since the last waypoint (holding it still is a stop) "
                "and a waypoint at the gizmo; without a drawing only the waypoint (the first one starts the path)",
            )
            self._drawing_buttons = gui.add_button_group("Drawing", ["Undo", "Discard"])
        self._servo_folders = [servo_folder]

        # ---- Trajectory view
        path_folder = self.path_editor.build_path(gui)
        # What to plan (whole path, segments From → To, or a free motion to
        # the gizmo), the plan waiting for Execute and Execute right below.
        motion_folder = gui.add_folder("Plan & Execute")
        with motion_folder:
            self.path_editor.build_plan_controls(gui)
            # Execution speed of every plan (path, segments, gizmo, Home,
            # replay): the planner divides the times by it, the preview shows
            # exactly the trajectory Execute sends.
            self._speed_slider = gui.add_slider(
                "Speed", min=0.05, max=1.0, step=0.05, initial_value=1.0,
                hint="1: the planned times (fastest or as set per segment); 0.5: everything takes twice as long. "
                "The preview and Execute are the same.",
            )
            self.path_editor.build_plan_button(gui)
            self._execute_button = gui.add_button("Execute", color="blue", icon=viser.Icon.PLAYER_PLAY)
            self._gizmo_home_button = gui.add_button(
                "Gizmo to Robot TCP", icon=viser.Icon.TARGET, hint="Puts the trajectory gizmo on the robot TCP"
            )
        detail_folders = self.path_editor.build_details(gui)
        self._trajectory_folders = [path_folder, motion_folder, *detail_folders]

        # ---- Files view: recordings and path files
        self._files_folders = [*self.recorder.build_library(gui), *self.path_editor.build_files(gui)]
        for folder in self._files_folders:
            folder.visible = False

        system_folder = gui.add_folder("System", expand_by_default=False)
        with system_folder:
            tabs = gui.add_tab_group()

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

        self._view_select.on_click(lambda e: self._set_view(view_from_label(str(e.target.value)), request=True))
        self._enable_button.on_click(lambda _: self._trigger("enable", "Enable Servo"))
        self._hold_button.on_click(lambda _: self._on_hold_button())
        self._gizmo_home_button.on_click(lambda _: self.reset_gizmo_to_robot())
        self._servo_confirm_button.on_click(lambda _: self.path_editor.servo_confirm())
        self._drawing_buttons.on_click(
            lambda e: self.path_editor.servo_undo() if e.target.value == "Undo" else self.path_editor.servo_discard()
        )
        self._speed_slider.on_update(lambda _: self._publish_speed())
        self._track_button.on_click(lambda _: self._toggle_tracking())
        self._axes_control.on_click(lambda e: self._on_servo_axes("base" if "base_link" in str(e.target.value) else "tool"))
        self._look_vector.on_update(lambda _: self._on_look_vector())
        self._home_button.on_click(lambda _: self._home())
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

    # ----------------------------------------------------------------- views
    def view(self) -> str:
        """Working view ("servo" or "trajectory"; Files keeps the previous one)."""
        return self._mode_view

    def show_view(self, view: str) -> None:
        """Switch views from a panel (e.g. Load / Edit as Path → Trajectory)."""
        if self._view != view:
            self._set_view(view, request=True)

    def _set_view(self, view: str, request: bool) -> None:
        """Servo: servo mode and the servo gizmo. Trajectory: plan mode, the
        trajectory gizmo, paths, plans and Execute. Files: recordings and
        path files, the mode stays. request: also switch the robot's mode
        (False when following a mode set elsewhere)."""
        if request and view == "servo" and (self._plan_status.get("state") == "executing" or self._homing):
            self._set_result("Servo: wait until the motion has finished")
            return
        self._view = view
        if view != "files":
            self._mode_view = view
        work = self._mode_view
        self._view_select.options = view_options(view)
        for folders, shown in (
            (self._servo_folders, view == "servo"),
            (self._trajectory_folders, view == "trajectory"),
            (self._files_folders, view == "files"),
        ):
            for folder in folders:
                if folder.visible != shown:
                    folder.visible = shown
        self._servo_gizmo.set_visible(work == "servo")
        self._apply_gizmo_visibility()
        if self._path_handle is not None:
            self._path_handle.visible = work == "trajectory"  # the plan belongs to the trajectory view
        self.path_editor.on_view(work)
        if not request or view == "files":
            return
        servo = self._servo
        if work == "servo":
            if servo is None or servo.state in ("DISABLED", "HOLD") or servo.mode != "servo":
                self._trigger("enable", "Servo")
            if self._tracking_on:
                self._request_free_roll(True)
        else:
            if self._tracking_on:
                self._request_free_roll(False)
            if servo is None or servo.mode != "plan" or servo.state != "DISABLED":
                self._trigger("disable", "Trajectory (plan mode)")

    def _apply_gizmo_visibility(self) -> None:
        self._plan_gizmo.set_visible(self._mode_view == "trajectory" and self._editor_shows_gizmo)

    # ------------------------------------------- interface for the Path editor
    def reset_gizmo_to_robot(self) -> None:
        """Trajectory gizmo on the robot TCP."""
        with self._lock:
            if not self._have_joint_state:
                return
            _, world_tip = self._world_poses()
        self._plan_gizmo.set_pose(world_tip)

    def set_tcp_gizmo_visible(self, visible: bool) -> None:
        """The Path editor hides the trajectory gizmo in Edit mode (and with
        it Gizmo to Robot TCP)."""
        self._editor_shows_gizmo = bool(visible)
        self._apply_gizmo_visibility()
        if self._gizmo_home_button.visible != self._editor_shows_gizmo:
            self._gizmo_home_button.visible = self._editor_shows_gizmo

    def set_gizmo_pose(self, position: np.ndarray, wxyz: np.ndarray) -> None:
        """Places the trajectory gizmo (base_link pose)."""
        base_pose = pin.SE3(pin.Quaternion(*[float(v) for v in wxyz]).normalized().matrix(), np.asarray(position, dtype=float))
        self._plan_gizmo.set_pose(self._base_pose * base_pose)

    def gizmo_pose(self):
        """Trajectory gizmo pose in base_link (position, wxyz), or None."""
        pose = self._plan_gizmo.pose()
        if pose is None:
            return None
        base_pose = self._base_pose.inverse() * pose
        q = pin.Quaternion(base_pose.rotation)
        return base_pose.translation.copy(), np.array([q.w, q.x, q.y, q.z])

    def gizmo_axes(self) -> str:
        """Axes of the trajectory gizmo: "base" or "tool" (base while tracking)."""
        return self._plan_gizmo.axes()

    def set_gizmo_axes(self, mode: str) -> None:
        """Trajectory gizmo axes (Path folder)."""
        self._plan_gizmo.set_axes(mode)
        self.path_editor.on_gizmo_axes(self._plan_gizmo.axes())
        self._set_result(axes_message(mode))

    def tracking_on(self) -> bool:
        return self._tracking_on

    def tracking_point(self) -> np.ndarray:
        """The point (base_link) of point tracking and of Look at point."""
        return self._base_pose.inverse().act(self._track_point)

    # ------------------------------------------------------------ servo gizmo
    def _servo_target_msg(self) -> PoseStamped | None:
        pose = self._servo_gizmo.pose()
        if pose is None:
            return None
        base_target = self._base_pose.inverse() * pose
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in base_target.translation)
        quaternion = pin.Quaternion(base_target.rotation)
        msg.pose.orientation.x = float(quaternion.x)
        msg.pose.orientation.y = float(quaternion.y)
        msg.pose.orientation.z = float(quaternion.z)
        msg.pose.orientation.w = float(quaternion.w)
        return msg

    def _publish_servo_target(self) -> None:
        msg = self._servo_target_msg()
        if msg is not None:
            self._target_pub.publish(msg)

    def servo_gizmo_pose(self):
        """Servo gizmo target in base_link (position, wxyz), or None."""
        pose = self._servo_gizmo.pose()
        if pose is None:
            return None
        base_pose = self._base_pose.inverse() * pose
        q = pin.Quaternion(base_pose.rotation)
        return base_pose.translation.copy(), np.array([q.w, q.x, q.y, q.z])

    def _sketch_tick(self) -> None:
        """Servo view: the held servo gizmo is sampled for the drawing."""
        if self._mode_view != "servo" or not self._servo_gizmo.held:
            return
        pose = self.servo_gizmo_pose()
        if pose is not None:
            self.path_editor.servo_sample(*pose)

    def _publish_speed(self) -> None:
        speed = float(self._speed_slider.value)
        self._speed_pub.publish(Float64(data=speed))
        self._set_result(f"Speed {speed:.2f}: plans take {1.0 / speed:.2f}× their time (preview = Execute)")

    def _on_servo_axes(self, mode: str) -> None:
        self._servo_gizmo.set_axes(mode)
        self._axes_control.options = axes_options(mode)
        self._set_result(axes_message(mode))

    def _on_reset_target(self, msg: PoseStamped) -> None:
        # Ignore a latched reset pose from a previous session.
        if rclpy.time.Time.from_msg(msg.header.stamp) < self._start_time:
            return
        p = msg.pose.position
        o = msg.pose.orientation
        base_target = pin.SE3(pin.Quaternion(o.w, o.x, o.y, o.z).normalized().matrix(), np.array([p.x, p.y, p.z]))
        self._servo_gizmo.set_pose(self._base_pose * base_target)

    # ---------------------------------------------------------- point tracking
    def _tracking_tick(self) -> None:
        """While the servo turns toward the point, keep publishing."""
        servo = self._servo
        if not self._tracking_on or self._mode_view != "servo" or servo is None or servo.state != "ACTIVE":
            return
        if self._servo_gizmo.turning_left() > np.radians(0.2):
            self._publish_servo_target()

    def _tool_axis_error_deg(self) -> float | None:
        """Angle between the robot's tool z axis and the direction to the point."""
        with self._lock:
            if not self._have_joint_state:
                return None
            _, world_tip = self._world_poses()
        direction = self._track_point - world_tip.translation
        if np.linalg.norm(direction) < 1e-6:
            return None
        cos = float(world_tip.rotation[:, 2] @ direction / np.linalg.norm(direction))
        return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))

    def _request_free_roll(self, on: bool, on_done=None) -> None:
        """Servo: track only the position and the tool axis (roll free)."""
        self._free_roll_sent = time.monotonic()
        if on_done is not None:
            self._call(self._free_roll_client, SetBool.Request(data=on), "Point tracking", on_done=on_done)
        elif self._free_roll_client.service_is_ready():
            self._free_roll_client.call_async(SetBool.Request(data=on))

    def _toggle_tracking(self) -> None:
        """In the servo view tracking starts only once the servo confirms the
        free tool roll (otherwise it would keep the UI's roll and could run
        j6 into its limit). Off: at once."""
        if not self._tracking_on and self._mode_view == "servo":
            self._request_free_roll(True, on_done=lambda response: self._set_tracking(True) if response.success else None)
            return
        if self._tracking_on and self._mode_view == "servo":
            self._request_free_roll(False)
        self._set_tracking(not self._tracking_on)

    def _set_tracking(self, on: bool) -> None:
        if on == self._tracking_on:
            return
        self._tracking_on = on
        # Off: both targets take the robot's tool orientation (the servo
        # chose its roll; in plan mode the robot did not turn).
        robot = None if on else self._tip_rotation
        self._servo_gizmo.set_tracking(on, robot)
        self._plan_gizmo.set_tracking(on, robot)
        _set_button(
            self._track_button, "Disable Point Tracking" if on else "Enable Point Tracking",
            "red" if on else None, viser.Icon.TARGET,
        )
        for handle in (self._look_gizmo, self._look_vector, self._look_info):
            handle.visible = on
        self._axes_control.visible = not on  # base_link axes while tracking
        self.path_editor.on_gizmo_axes(self._plan_gizmo.axes())
        if on:
            self._sync_point_vector()
            self._publish_servo_target()  # the servo target turns toward the point
            self._set_result("Point tracking: the tool turns toward the red sphere (set its position under Point [mm])")
        else:
            self._publish_servo_target()
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
        world = self._base_pose.act(np.asarray(self._look_vector.value, dtype=float) / 1000.0)
        if np.linalg.norm(world - self._track_point) < 5e-4:
            return  # echo of a value set from here
        self._track_point = world
        self._look_gizmo.position = tuple(world)
        self._plan_gizmo.refresh()
        if self._tracking_on:
            self._publish_servo_target()  # the servo target follows the point

    def _update_look_at(self, servo) -> None:
        if not self._free_roll_started and self._free_roll_client.service_is_ready():
            # This UI (re)started without point tracking: the servo must not be
            # left in the free-roll mode of an earlier session.
            self._free_roll_started = True
            self._request_free_roll(False)
        if (
            self._tracking_on and self._mode_view == "servo" and servo is not None and not servo.free_tool_roll
            and time.monotonic() - self._free_roll_sent > 2.0
        ):
            # Tracking is on but the servo restarted (it starts with the full
            # pose): request the free roll again. Never forced off here, so
            # another client (e.g. the benchmark) may use the mode.
            self._request_free_roll(True)
        if not self._tracking_on:
            return
        gizmo = self._servo_gizmo if self._mode_view == "servo" else self._plan_gizmo
        pose = gizmo.pose()
        error = self._tool_axis_error_deg()
        self._look_info.content = (
            (f"*{np.linalg.norm(self._track_point - pose.translation):.2f} m from the target" if pose is not None else "*")
            + (f" · robot tool axis {error:.0f}° off" if error is not None else "")
            + "*"
        )

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
            colors=(30, 120, 220), thickness=0.0025, visible=self._mode_view == "trajectory",
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

    # ----------------------------------------------------------------- timers
    def _display(self) -> None:
        with self._lock:
            if not self._have_joint_state:
                return
            q = self._q.copy()
            _, world_tip = self._world_poses()
        self._tip_rotation = world_tip.rotation.copy()
        self._viz.display(q)
        quaternion = pin.Quaternion(world_tip.rotation)
        self._tcp_axes.position = world_tip.translation.copy()
        self._tcp_axes.wxyz = np.array([quaternion.w, quaternion.x, quaternion.y, quaternion.z])
        for gizmo in (self._servo_gizmo, self._plan_gizmo):
            if not gizmo.initialized:
                gizmo.set_pose(world_tip)

    def _animate_preview(self) -> None:
        # The preview runs only while a plan waits for Execute. It never
        # commands the robot.
        with self._lock:
            times = self._preview_times
            positions = self._preview_positions
            start = self._preview_start
            planned = self._plan_status.get("state") == "planned"
            q = self._q.copy()
        show = planned and times is not None and len(times) > 1 and self._mode_view == "trajectory"
        t = 0.0
        if show:
            # Real time: the plan already has the execution speed. Once per
            # plan (1 s on the goal); repeated with Plan continuously.
            duration = float(times[-1])
            elapsed = time.monotonic() - start
            if self.path_editor.continuous():
                t = min(elapsed % (duration + 1.0), duration)
            elif elapsed > duration + 1.0:
                show = False
            else:
                t = min(elapsed, duration)
        if show != self._preview_shown:
            self._preview.displayVisuals(show)
            self._preview_shown = show
        if not show:
            return
        for column, name in enumerate(JOINT_NAMES):
            q[self._joint_q_index[name]] = np.interp(t, times, positions[:, column])
        self._preview.display(q)

    def _heartbeat(self) -> None:
        # A still gizmo is still a valid reference. The heartbeat only runs
        # while a browser is connected, so closing the browser lets the servo
        # target watchdog switch to HOLD.
        if self._server.get_clients() and self._mode_view == "servo":
            self._publish_servo_target()

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

    def plan_to_gizmo(self) -> None:
        """Plan: To gizmo (RRT + TOPP-RA) to the trajectory gizmo pose."""
        servo = self._servo
        if servo is not None and (servo.state != "DISABLED" or servo.mode != "plan"):
            self._set_result("Plan to gizmo: only in plan mode with the servo DISABLED")
            return
        gizmo = self.gizmo_pose()
        if gizmo is None:
            self._set_result("Plan: no gizmo pose yet")
            return
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "base_link"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (float(v) for v in gizmo[0])
        pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z = (
            float(v) for v in gizmo[1]
        )
        self._last_joint_plan = "to the gizmo"
        self._plan_goal_pub.publish(pose)
        self._set_result("Plan: to the trajectory gizmo (RRT + TOPP-RA)")

    def _home(self) -> None:
        if self._homing:
            return
        self._homing = True
        threading.Thread(target=self._home_sequence, daemon=True).start()

    def _wait_for(self, condition, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if condition():
                return True
            time.sleep(0.05)
        return False

    def _service(self, client, request, timeout: float = 15.0):
        if not client.wait_for_service(timeout_sec=2.0):
            return None
        future = client.call_async(request)
        if not self._wait_for(future.done, timeout):
            return None
        return future.result()

    def _home_sequence(self) -> None:
        """Home: plan mode if needed → plan to the home pose → Execute → back
        to servoing if it was started from the servo view."""
        back_to_servo = self._mode_view == "servo"
        try:
            servo = self._servo
            if servo is not None and (servo.mode != "plan" or servo.state != "DISABLED"):
                self._set_result("Home: switching to plan mode…")
                response = self._service(self._servo_clients["disable"], Trigger.Request())
                if response is None or not response.success:
                    raise RuntimeError(f"plan mode refused ({getattr(response, 'message', 'no answer')})")
                if not self._wait_for(
                    lambda: self._servo is not None and self._servo.mode == "plan" and self._servo.state == "DISABLED", 10.0
                ):
                    raise RuntimeError("the servo did not reach plan mode")
            before = float(self._plan_status.get("stamp", 0.0))
            self._plan_home()

            def planned() -> bool:
                plan = self._plan_status
                return float(plan.get("stamp", 0.0)) > before and plan.get("state") in ("planned", "failed")

            if not self._wait_for(planned, 20.0) or self._plan_status.get("state") != "planned":
                raise RuntimeError(f"planning failed ({self._plan_status.get('message', 'no answer')})")
            self._set_result("Home: executing…")
            response = self._service(self._execute_client, Trigger.Request())
            if response is None or not response.success:
                raise RuntimeError(f"Execute refused ({getattr(response, 'message', 'no answer')})")
            if not self._wait_for(lambda: self._plan_status.get("state") in ("succeeded", "failed"), 60.0):
                raise RuntimeError("no end of the motion reported")
            if self._plan_status.get("state") != "succeeded":
                raise RuntimeError(self._plan_status.get("message", "execution failed"))
            if back_to_servo and self._mode_view == "servo":
                self._service(self._servo_clients["enable"], Trigger.Request())
                if self._tracking_on:
                    self._request_free_roll(True)
            self._set_result("Home: done" + (" (servoing again)" if back_to_servo else ""))
        except RuntimeError as exc:
            self._set_result(f"Home: {exc}")
        finally:
            self._homing = False

    def _plan_home(self) -> None:
        servo = self._servo
        if servo is not None and (servo.state != "DISABLED" or servo.mode != "plan"):
            self._set_result("Home: only in plan mode with the servo DISABLED")
            return
        goal = JointState()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.name = list(JOINT_NAMES)
        goal.position = [float(np.radians(v)) for v in self.get_parameter("home_joints_deg").value]
        self._last_joint_plan = "home"
        self._joint_goal_pub.publish(goal)
        home = ", ".join(f"{v:.0f}" for v in self.get_parameter("home_joints_deg").value)
        self._set_result(f"Home ({home} deg): planning…")

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
        if not self._view_initialized and servo is not None:
            # Start in the view of the servo's current mode (no request).
            self._view_initialized = True
            self._set_view("servo" if servo.mode == "servo" else "trajectory", request=False)
        self._update_summary(servo, robot)
        self._update_look_at(servo)
        self._update_buttons(servo, robot, plan)
        self._update_status(servo, robot)
        self._update_speed(servo, robot)
        problem = self.path_editor.problem()
        plan_text = self._plan_text(plan)
        if problem and not (plan.get("kind") == "cartesian_path" and plan.get("state") == "failed"):
            plan_text += f"  \n❌ Path: {problem}"  # the drawing cannot be executed (check)
        self._plan_markdown.content = plan_text
        self._drawing_markdown.content = self.path_editor.servo_drawing_text()
        with self._lock:
            tracking = self._last_tracking
        self._tracking_markdown.content = tracking_text(tracking) if tracking else ""
        plan_mode = servo is not None and servo.mode == "plan" and servo.state == "DISABLED"
        self.recorder.refresh(plan_mode)
        self.path_editor.refresh(plan_mode)
        self._servo_gizmo.refresh()
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

    def _plan_text(self, plan: dict) -> str:
        """What is planned (path, segments, gizmo, Home, replay), its state
        and duration, in two short lines."""
        state = plan.get("state", "?")
        what = {
            "joint_goal": self._last_joint_plan,
            "joint_path": f"replay {self.recorder.loaded_name()}",
            "cartesian_path": self.path_editor.plan_label() or "path",
        }.get(plan.get("kind"), "")
        if state in ("ready", "unknown") or not what:
            return "Nothing planned." if state != "unknown" else plan.get("message", "")
        verb = {"planning": "Planning", "planned": "Planned", "executing": "Executing",
                "succeeded": "Executed", "failed": "Failed"}.get(state, state)
        text = f"{PLAN_ICON.get(state, '•')} **{verb}: {what}**"
        if "points" in plan and state in ("planned", "executing", "succeeded"):
            text += f" · {plan.get('trajectory_duration_s', 0.0):.2f} s"
            if plan.get("approach_duration_s"):
                text += f" (approach {plan['approach_duration_s']:.2f} s)"
            if plan.get("speed", 1.0) < 0.999:
                text += f" · speed {plan['speed']:.2f}"
            if plan.get("time_scale", 1.0) > 1.001:
                text += f" · slowed ×{plan['time_scale']:.2f} (joint limits)"
        if state == "failed":
            text += f"  \n{plan.get('message', '')}"
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
        # Servo view: Enable Servo only when the servo is not running there
        # (refused, or after a fault); Servo Hold ↔ Resume Servo.
        retry = state in ("DISABLED",) or (servo is not None and servo.mode != "servo")
        if self._enable_button.visible != retry:
            self._enable_button.visible = retry
        self._enable_button.disabled = state in ("RECOVERING", "SYNCING", "UNKNOWN", "FAULT", "LOCKOUT")
        hold = state != "HOLD"
        _set_button(self._hold_button, "Servo Hold" if hold else "Resume Servo",
                    "yellow" if hold else "green", viser.Icon.PLAYER_PAUSE if hold else viser.Icon.PLAYER_PLAY)
        self._hold_button.disabled = state not in ("ACTIVE", "HOLD")
        fault = state in ("FAULT", "LOCKOUT")
        if self._recover_button.visible != fault:  # top left, only when needed
            self._recover_button.visible = fault
            self._fault_markdown.visible = fault
        self._home_button.disabled = self._homing or state in ("FAULT", "LOCKOUT", "RECOVERING", "UNKNOWN")
        self._execute_button.disabled = not plan_mode or plan.get("state") != "planned"
        self._safety_button.disabled = robot is None or not robot.safety_config_writes_allowed

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
