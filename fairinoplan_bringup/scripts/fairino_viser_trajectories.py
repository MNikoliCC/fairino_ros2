"""Viser panels for trajectory recording/replay and the Cartesian path editor.

Both panels only build requests: planning, IK and timing run in the planner
(/fairinoplan/path_goal), and every motion goes through the same
plan -> preview -> Execute flow as a normal plan. Recording only reads
/joint_states. See docs/trajectory_recording_and_paths.md.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import pinocchio as pin
import viser
from std_msgs.msg import String

import fairino_trajectory_tools as tt

JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]
TIMING_LABELS = ["Recorded timing", "Fastest within limits"]


def resolve_trajectory_dir(configured: str) -> Path:
    """Configured dir, else the bind-mounted /ros2_ws/trajectories, else ~/.ros."""
    candidates = [configured] if configured else []
    candidates += [os.environ.get("FAIRINO_TRAJECTORY_DIR", ""), "/ros2_ws/trajectories"]
    for candidate in candidates:
        if candidate and os.path.isdir(candidate) and os.access(candidate, os.W_OK):
            return Path(candidate)
    fallback = Path.home() / ".ros" / "fairino_trajectories"
    fallback.mkdir(parents=True, exist_ok=True)
    return fallback


def list_json(directory: Path) -> list[str]:
    if not directory.is_dir():
        return []
    files = sorted(directory.glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True)
    return [f.name for f in files]


# ===========================================================================
# Recording and replay
# ===========================================================================


AUTOSAVE_NAME = "runtime_trajectory.json"  # running recording (prototype name), not listed


class RecordingPanel:
    """Records the measured motion (any mode) from /joint_states.

    Armed -> recording starts with the first motion (0.25 s pre-roll, so it
    starts at rest) -> pauses 0.5 s after the motion stops (so it ends at
    rest) -> resumes with the next motion. Pauses are not part of the time
    axis. Samples: t, FAIRINO-style TCP pose (frame flange), joints [rad]."""

    PRE_ROLL_S = 0.25
    STILL_S = 0.5
    MOTION_RAD_S = math.radians(0.5)

    def __init__(self, ui, directory: Path, urdf_path: str, record_rate_hz: float):
        self._ui = ui
        self._dir = directory / "recordings"
        self._dir.mkdir(parents=True, exist_ok=True)
        self._urdf_path = urdf_path
        self._period = 1.0 / record_rate_hz
        self._lock = threading.Lock()
        self._model = ui.model
        self._data = self._model.createData()
        self._state = "idle"  # idle | armed | recording | paused
        self._samples: list[dict] = []
        self._segments: list[dict] = []
        self._pre_roll: deque = deque()
        self._active_time = 0.0
        self._segment_start_stamp = 0.0
        self._segment_start_index = 0
        self._segment_mode = "planning"
        self._last_stamp = None
        self._last_q = None
        self._last_kept = -1e9
        self._last_motion = 0.0
        self._dirty = False
        self._loaded: dict | None = None
        self._loaded_name = ""
        self._trail = None
        self._trail_count = -1
        self._trail_source = None
        self._trail_done = 0
        self._trail_points: list = []

    # ------------------------------------------------------------------ GUI
    def build_gui(self, gui) -> None:
        with gui.add_folder("Recording"):
            self._name = gui.add_text("Name", "recording")
            self._record_button = gui.add_button(
                "Record", color="red", icon=viser.Icon.PLAYER_RECORD,
                hint="Arms the recorder: it starts with the next motion (servo or Execute) and pauses when the robot stops",
            )
            self._stop_button = gui.add_button("Stop & Save", icon=viser.Icon.PLAYER_STOP)
            self._discard_button = gui.add_button("Discard", icon=viser.Icon.TRASH)
            self._status = gui.add_markdown("Idle.")
        with gui.add_folder("Library"):
            self._library = gui.add_dropdown("Recording", ["—"])
            # The orange TCP trail shows the loaded recording (and the one
            # being recorded); Unload removes both.
            self._library_buttons = gui.add_button_group("Files", ["Load", "Unload", "Refresh"])
            self._loaded_info = gui.add_markdown("No recording loaded.")
        with gui.add_folder("Replay"):
            self._timing = gui.add_dropdown("Timing", TIMING_LABELS, initial_value=TIMING_LABELS[0])
            self._speed = gui.add_slider("Speed [%]", min=10, max=100, step=5, initial_value=100)
            self._replay_button = gui.add_button(
                "Plan Replay", icon=viser.Icon.ROUTE, hint="Approach + recorded motion; check the preview, then Execute"
            )
        with gui.add_folder("Edit as path"):
            self._tolerance = gui.add_number("Simplify tolerance [mm]", 3.0, min=0.5, max=50.0, step=0.5)
            self._edit_button = gui.add_button(
                "Edit as Path", icon=viser.Icon.PENCIL, hint="Turns the loaded recording into editable waypoints (Path tab)"
            )
        self._record_button.on_click(lambda _: self._arm())
        self._stop_button.on_click(lambda _: self._stop_and_save())
        self._discard_button.on_click(lambda _: self._discard())
        self._library_buttons.on_click(lambda e: self._library_action(e.target.value))
        self._replay_button.on_click(lambda _: self._plan_replay())
        self._edit_button.on_click(lambda _: self._edit_as_path())
        self._refresh_library()

    # ------------------------------------------------------------ recording
    def on_joint_state(self, stamp: float, q: np.ndarray) -> None:
        with self._lock:
            if self._state == "idle":
                self._last_stamp, self._last_q = stamp, q
                return
            moving = False
            if self._last_q is not None and self._last_stamp is not None and stamp > self._last_stamp:
                moving = np.max(np.abs(q - self._last_q)) / (stamp - self._last_stamp) > self.MOTION_RAD_S
            self._last_stamp, self._last_q = stamp, q
            if moving:
                self._last_motion = stamp
            if self._state in ("armed", "paused"):
                self._pre_roll.append((stamp, q))
                while self._pre_roll and stamp - self._pre_roll[0][0] > self.PRE_ROLL_S:
                    self._pre_roll.popleft()
                if moving:
                    self._start_segment()
                return
            # recording
            if stamp - self._last_kept >= self._period - 1e-4:
                self._append(stamp, q)
            if stamp - self._last_motion > self.STILL_S:
                self._end_segment()

    def _start_segment(self) -> None:
        servo = self._ui.servo_status()
        self._segment_mode = "servo" if servo is not None and servo.mode == "servo" else "planning"
        self._segment_start_stamp = self._pre_roll[0][0]
        self._segment_start_index = len(self._samples)
        self._state = "recording"
        self._last_kept = -1e9
        for stamp, q in list(self._pre_roll):
            if stamp - self._last_kept >= self._period - 1e-4:
                self._append(stamp, q)
        self._pre_roll.clear()

    def _append(self, stamp: float, q: np.ndarray) -> None:
        full = pin.neutral(self._model)
        for name, value in zip(JOINT_NAMES, q):
            full[self._ui.joint_q_index[name]] = value
        pin.framesForwardKinematics(self._model, self._data, full)
        tcp = self._data.oMf[self._ui.base_frame].inverse() * self._data.oMf[self._ui.tip_frame]
        t = self._active_time + (stamp - self._segment_start_stamp)
        if self._samples and t <= self._samples[-1]["t"]:
            t = self._samples[-1]["t"] + 1e-4
        self._samples.append(
            {
                "t": round(float(t), 6),
                "pose": [round(v, 4) for v in tt.pose_to_fairino(tcp.translation, tt.matrix_to_quat_wxyz(tcp.rotation))],
                "joint_positions": [round(float(v), 7) for v in q],
            }
        )
        self._last_kept = stamp
        self._dirty = True

    def _end_segment(self) -> None:
        if self._state != "recording":
            return
        if len(self._samples) > self._segment_start_index:
            start, end = self._segment_start_index, len(self._samples) - 1
            self._segments.append(
                {
                    "mode": self._segment_mode,
                    "command_pose": None,
                    "start_index": start,
                    "end_index": end,
                    "start_time": self._samples[start]["t"],
                    "end_time": self._samples[end]["t"],
                }
            )
            # Next segment starts one sample period later (pauses are cut).
            self._active_time = self._samples[end]["t"] + self._period
        self._state = "paused"
        self._pre_roll.clear()

    def _arm(self) -> None:
        with self._lock:
            if self._state != "idle":
                self._ui.set_result("Record: already armed or recording")
                return
            self._samples, self._segments = [], []
            self._active_time = 0.0
            self._pre_roll.clear()
            self._state = "armed"
        self._ui.set_result("Record: armed — recording starts with the next motion")

    def _discard(self) -> None:
        with self._lock:
            self._state = "idle"
            self._samples, self._segments = [], []
            self._pre_roll.clear()
        self._draw_trail(force=True)
        self._ui.set_result("Record: discarded")

    def _stop_and_save(self) -> None:
        with self._lock:
            if self._state == "recording":
                self._end_segment()
            samples, segments = list(self._samples), list(self._segments)
            self._state = "idle"
        if len(samples) < 2:
            self._ui.set_result("Record: nothing recorded (the robot did not move)")
            return
        name = tt.safe_name(str(self._name.value), "recording")
        payload = self._payload(name, samples, segments)
        path = tt.write_json(self._dir / f"{name}_{time.strftime('%Y%m%d_%H%M%S')}.json", payload)
        tt.write_json(self._dir / AUTOSAVE_NAME, payload)
        # Saved, not loaded: the recording's trail goes away; Load shows it.
        self._refresh_library(select=path.name)
        self._draw_trail(force=True)
        self._ui.set_result(
            f"Record: saved {path.name} ({len(samples)} samples, {payload['total_duration_s']:.1f} s) — Load to replay or edit it"
        )

    def _payload(self, name: str, samples: list[dict], segments: list[dict]) -> dict:
        robot = self._ui.robot_status()
        return tt.recording_payload(
            robot_name="fairino20_v6",
            urdf_path=self._urdf_path,
            joint_names=JOINT_NAMES,
            samples=samples,
            segments=segments,
            name=name,
            tool_number=None if robot is None else int(robot.tool_number),
        )

    def tick(self) -> None:
        """2 Hz: autosave of the running recording and the trail."""
        with self._lock:
            autosave = self._dirty and self._state in ("recording", "paused") and len(self._samples) > 1
            samples, segments = (list(self._samples), list(self._segments)) if autosave else ([], [])
            self._dirty = False
        if autosave:
            tt.write_json(self._dir / AUTOSAVE_NAME, self._payload("runtime", samples, segments))
        self._draw_trail()

    def status_text(self) -> str:
        with self._lock:
            state, count = self._state, len(self._samples)
            duration = self._samples[-1]["t"] if self._samples else 0.0
            segments = len(self._segments) + (1 if state == "recording" else 0)
        icon = {"idle": "⚪", "armed": "🟡", "recording": "🔴", "paused": "⏸️"}[state]
        text = {
            "idle": "Idle.",
            "armed": "Armed — waiting for motion.",
            "recording": "Recording.",
            "paused": "Paused — robot at rest; continues with the next motion.",
        }[state]
        if state == "idle":
            return f"{icon} {text}"
        return f"{icon} **{text}**  \n{count} samples · {duration:.1f} s · {segments} segment(s)"

    def refresh(self, plan_mode: bool) -> None:
        self._status.content = self.status_text()
        with self._lock:
            state = self._state
        self._record_button.disabled = state != "idle"
        self._stop_button.disabled = state == "idle"
        self._discard_button.disabled = state == "idle"
        has_loaded = self._loaded is not None and len(self._loaded.get("samples", [])) > 1
        self._replay_button.disabled = not (plan_mode and has_loaded and state == "idle")
        self._edit_button.disabled = not has_loaded

    # ------------------------------------------------------------- library
    def _refresh_library(self, select: str | None = None) -> None:
        # The autosave of the running recording is not a library entry.
        files = [f for f in list_json(self._dir) if f != AUTOSAVE_NAME]
        self._library.options = tuple(files) if files else ("—",)
        if select and select in files:
            self._library.value = select

    def _library_action(self, action: str) -> None:
        if action == "Refresh":
            self._refresh_library()
            return
        if action == "Unload":
            name, self._loaded, self._loaded_name = self._loaded_name, None, ""
            self._describe_loaded()
            self._draw_trail(force=True)
            self._ui.set_result(f"Unload: {name}" if name else "Unload: no recording loaded")
            return
        name = str(self._library.value)
        if name == "—":
            return
        try:
            payload = json.loads((self._dir / name).read_text(encoding="utf-8"))
            if len(payload.get("samples", [])) < 2:
                raise ValueError("fewer than two samples")
        except Exception as exc:  # noqa: BLE001
            self._ui.set_result(f"Load: {name} failed ({exc})")
            return
        self._loaded, self._loaded_name = payload, name
        self._describe_loaded()
        self._draw_trail(force=True)
        self._ui.set_result(f"Load: {name}")

    def _describe_loaded(self) -> None:
        p = self._loaded
        if p is None:
            self._loaded_info.content = "No recording loaded."
            return
        modes = sorted({s.get("mode", "?") for s in p.get("segments", [])}) or ["—"]
        self._loaded_info.content = (
            f"**{self._loaded_name}**  \n{p.get('sample_count', len(p['samples']))} samples · "
            f"{p.get('total_duration_s', 0.0):.1f} s · {p.get('segment_count', 0)} segment(s) ({', '.join(modes)}) · "
            f"created {p.get('created_at', '?')}"
        )

    TRAIL_SPACING_M = 0.002

    def _draw_trail(self, force: bool = False) -> None:
        with self._lock:
            live = self._state in ("recording", "paused") and len(self._samples) > 1
            samples = self._samples if live else (self._loaded or {}).get("samples", [])
            count = len(samples)
            if not force and count == self._trail_count:
                return
            # Only the samples added since the last call are new (the list
            # grows while recording); anything else starts over.
            if samples is not self._trail_source or count < self._trail_done:
                self._trail_source, self._trail_done, self._trail_points = samples, 0, []
            fresh = [s["pose"][:3] for s in samples[self._trail_done:]]
            self._trail_done = count
            kept = self._trail_points
        self._trail_count = count
        # A point every 2 mm of TCP travel, chosen from the start: points
        # already drawn never change while the recording grows (picking every
        # n-th sample with n growing made the trail jump), and the polyline
        # is fine enough to show the curves.
        for p in np.asarray(fresh, dtype=float).reshape(-1, 3) / 1000.0:
            if not kept or np.linalg.norm(p - kept[-1]) >= self.TRAIL_SPACING_M:
                kept.append(p)
        points = list(kept)
        if fresh and np.linalg.norm(np.asarray(fresh[-1]) / 1000.0 - points[-1]) > 1e-6:
            points.append(np.asarray(fresh[-1], dtype=float) / 1000.0)
        if self._trail is not None:
            self._trail.remove()
            self._trail = None
        if len(points) > 1:
            points = np.asarray(points)
            self._trail = self._ui.server.scene.add_line_segments(
                "/fairino/recording_trail",
                points=np.stack([points[:-1], points[1:]], axis=1),
                colors=(240, 120, 20),
                thickness=0.003,
            )

    # --------------------------------------------------------------- replay
    def _plan_replay(self) -> None:
        if not self._ui.plan_mode_ready("Plan Replay"):
            return
        p = self._loaded
        samples = p["samples"]
        request = {
            "type": "joint_path",
            "name": self._loaded_name,
            "joint_names": list(p.get("joint_names", JOINT_NAMES)),
            "times": [float(s["t"]) for s in samples],
            "positions": [list(map(float, s["joint_positions"])) for s in samples],
            "timing": "original" if self._timing.value == TIMING_LABELS[0] else "optimal",
            "speed_scale": float(self._speed.value) / 100.0,
        }
        self._ui.path_goal_pub.publish(String(data=json.dumps(request)))
        self._ui.set_result(f"Plan Replay: {self._loaded_name} sent to the planner ({self._timing.value}, {self._speed.value:.0f} %)")

    def _edit_as_path(self) -> None:
        try:
            waypoints, segments = tt.recording_to_path(self._loaded, position_tol=float(self._tolerance.value) / 1000.0)
        except Exception as exc:  # noqa: BLE001
            self._ui.set_result(f"Edit as Path: {exc}")
            return
        name = Path(self._loaded_name).stem
        self._ui.path_editor.load_path(waypoints, segments, name)
        self._ui.set_result(f"Edit as Path: {len(waypoints)} waypoints from {self._loaded_name} (Path tab)")


# ===========================================================================
# Path editor
# ===========================================================================

COLOR_SEGMENT = (40, 170, 70)
COLOR_SELECTED = (245, 200, 0)
COLOR_FAILED = (230, 50, 50)
COLOR_DRAFT = COLOR_SEGMENT
COLOR_HANDLE = (0, 185, 225)
SAME_POINT_M = 5e-4


_REDRAW = object()  # draft-line key that never equals a drawn state


def axes_options(mode: str) -> tuple[str, str]:
    """Gizmo axes as a pair of check boxes: base_link (move only) / tool."""
    return (f"{'☑' if mode == 'base' else '☐'} base_link", f"{'☑' if mode == 'tool' else '☐'} tool")


def _mode_options(mode: str) -> tuple[str, str]:
    """Labels of the Draw/Edit pair: a check box look, exactly one ticked."""
    return tuple(f"{'☑' if m == mode else '☐'} {m}" for m in ("Draw", "Edit"))


def _align_axis(wxyz: np.ndarray, direction: np.ndarray) -> np.ndarray:
    """Smallest rotation of a gizmo so that the signed local axis closest to
    `direction` lies exactly on it (prototype: orient_curve_gizmo_toward_neighbor)."""
    norm = float(np.linalg.norm(direction))
    if norm < 1e-9:
        return wxyz
    target = direction / norm
    rotation = tt.quat_wxyz_to_matrix(wxyz)
    axes = np.hstack([rotation, -rotation]).T  # +x, +y, +z, -x, -y, -z in world
    best = axes[int(np.argmax(axes @ target))]
    cross = np.cross(best, target)
    dot = float(np.clip(best @ target, -1.0, 1.0))
    if np.linalg.norm(cross) < 1e-9:
        return wxyz  # already on the line (best is the closest axis, so dot > 0)
    angle = math.atan2(float(np.linalg.norm(cross)), dot)
    delta = pin.exp3(cross / np.linalg.norm(cross) * angle)
    return tt.matrix_to_quat_wxyz(delta @ rotation)


def gui_number(gui, label: str, value: float):
    return gui.add_number(label, value, min=0.0, max=600.0, step=0.1, hint="Time of this segment [s]; 0 = as fast as possible")


def _short(message: str) -> str:
    """Planner message without the hints meant for the log."""
    return message.split(" — ")[0].strip()


def _pose_text(position: np.ndarray, wxyz: np.ndarray) -> str:
    """'x, y, z mm · rx, ry, rz°' in the FAIRINO convention (no '-0.0')."""
    values = [round(v, 1) + 0.0 for v in tt.pose_to_fairino(position, wxyz)]
    return f"{values[0]:.1f}, {values[1]:.1f}, {values[2]:.1f} mm · {values[3]:.1f}, {values[4]:.1f}, {values[5]:.1f}°"


class PathEditor:
    """Cartesian path editor with two modes.

    Draw: the TCP gizmo is the pen; a light green line runs from the last
    waypoint to it (red as soon as that pose cannot be reached) and Confirm
    Waypoint fixes it; clicking an earlier waypoint adds a segment back to it.
    Edit: the pen is hidden; clicking a waypoint shows a gizmo to move/rotate
    it and two tangent handles (toward the previous and the next waypoint, at
    35 % while straight); dragging a handle bends the path through the
    waypoint as a Bezier (the other handle mirrors it), as in the FAIRINO
    Viser prototype.

    The path is checked (failing segments red, fastest time per segment) or,
    with Plan while drawing, planned after every change. Execute is in the
    Control tab."""

    CHECK_DELAY_S = 0.4
    DRAFT_CHECK_PERIOD_S = 0.3
    SNAP_FRACTION = 0.35
    DRAG_SETTLE_S = 0.6  # a handle is re-aligned this long after its last drag event
    CHIP_START_M = 0.05  # numbered spheres of a shared point
    CHIP_STEP_M = 0.04

    def __init__(self, ui, directory: Path):
        self._ui = ui
        self._dir = directory / "paths"
        self._dir.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._waypoints: list[tt.Waypoint] = []
        self._segments: list[tt.Segment] = []
        self._mode = "Draw"
        self._seg: int | None = None
        self._wp: int | None = None
        self._fastest: list[float] = []
        self._planned: list[float] = []
        self._failed: set[int] = set()
        self._plan_text = ""
        self._run_text = ""  # tracking of the last execution of this path
        self._undo: list = []
        self._redo: list = []
        self._last_edit = 0.0
        self._changed_at = 0.0
        self._check_due = False
        self._stale_plan = False
        self._ours = False  # the latest plan came from this editor
        self._request_id = 0
        self._draft_id = 0
        self._draft_failed = False
        self._draft_checked_key = None
        self._draft_sent_at = 0.0
        self._last_plan_request = 0.0
        self._last_q = None
        self._last_motion = 0.0
        self._dirty = True
        self._updating = False
        self._handles: list = []
        self._dragged_handle: str | None = None
        self._drag_time = 0.0
        self._realign_due = False
        self._draft = None
        self._draft_key = _REDRAW

    # ------------------------------------------------------------------ GUI
    def build_gui(self, gui) -> None:
        scene = self._ui.server.scene
        with gui.add_folder("Path"):
            self._new_button = gui.add_button(
                "New Path", icon=viser.Icon.ROUTE, hint="Starts a new path at the robot's TCP and puts the gizmo there"
            )
            self._summary = gui.add_markdown("")
            # Rows of small buttons: what is edited, the gizmo's axes, and
            # the orientation shortcut (TCP gizmo in Draw, waypoint in Edit).
            self._mode_select = gui.add_button_group("Mode", _mode_options("Draw"))
            # The same setting as in the Control tab; both move and rotate,
            # only the axes differ. Hidden while point tracking is on.
            self._gizmo_axes = gui.add_button_group("Gizmo axes", axes_options("tool"))
            self._orient_button = gui.add_button_group("Orientation", ["Look at point"])
            self._confirm_button = gui.add_button(
                "Confirm Waypoint", color="green", icon=viser.Icon.CHECK,
                hint="Adds the TCP gizmo pose as the next waypoint (end of the light green line)",
            )
            self._edit_buttons = gui.add_button_group("Edit", ["Undo", "Redo", "Clear"])
            self._auto_plan = gui.add_checkbox(
                "Plan while drawing", True,
                hint="On: the trajectory is planned after every change (the preview robot follows). "
                "Off: only a check (red segments, fastest times); press Plan Trajectory.",
            )
            self._plan_button = gui.add_button(
                "Plan Trajectory", icon=viser.Icon.ROUTE_2,
                hint="Approach + path; check the preview, then Execute in the Control tab",
            )
        with gui.add_folder("Waypoint"):
            self._wp_select = gui.add_dropdown("Waypoint", ["—"])
            self._straight_button = gui.add_button("Straight", hint="Removes the curve of the selected waypoint")
            self._delete_wp = gui.add_button("Delete Waypoint", icon=viser.Icon.TRASH)
        # All segments at once: one time field per segment, its label says
        # the fastest possible time (0 = as fast as possible).
        self._seg_folder = gui.add_folder("Segments")
        with self._seg_folder:
            self._seg_info = gui.add_markdown("")
            self._seg_page = gui.add_dropdown("Show", ["—"], visible=False)
        self._seg_inputs: dict[int, object] = {}
        self._seg_labels: dict[int, str] = {}
        self._seg_values: dict[int, float] = {}
        self._seg_rows: tuple = ()
        with gui.add_folder("Coordinates", expand_by_default=False):
            self._coord_info = gui.add_markdown("")
            self._move_by = gui.add_vector3("Move [mm]", (0.0, 0.0, 0.0), step=0.1)
            self._rotate_by = gui.add_vector3("Rotate [deg]", (0.0, 0.0, 0.0), step=0.1)
            self._offset_frame = gui.add_dropdown(
                "Frame", ["base_link", "local"], initial_value="base_link",
                hint="Axes of the offset: robot base, or the gizmo's / waypoint's own axes",
            )
            self._apply_offset = gui.add_button(
                "Apply Offset", hint="Moves the TCP gizmo (Draw) or the selected waypoint (Edit) by the offset"
            )
            self._waypoint_table = gui.add_markdown("")
        with gui.add_folder("Files", expand_by_default=False):
            self._file_name = gui.add_text("Name", "path")
            self._files = gui.add_dropdown("Path file", ["—"])
            self._file_buttons = gui.add_button_group("Files", ["Save", "Load", "Refresh"])

        self._pose_gizmo = scene.add_transform_controls(
            "/fairino/path_pose", scale=0.16, depth_test=False, disable_sliders=False, visible=False
        )
        self._prev_gizmo = scene.add_transform_controls(
            "/fairino/path_tangent_prev", scale=0.08, depth_test=False, disable_rotations=True, visible=False
        )
        self._next_gizmo = scene.add_transform_controls(
            "/fairino/path_tangent_next", scale=0.08, depth_test=False, disable_rotations=True, visible=False
        )
        self._pose_gizmo.on_update(lambda _: self._on_pose_gizmo())
        # A gizmo that is being dragged is never moved from here: a pose set
        # during a drag makes it jump (seen with point tracking, where the
        # shown orientation differs from the gizmo's own).
        self._held: set[str] = set()
        self._pose_ref: np.ndarray | None = None  # waypoint rotation when a base_link drag started
        for key, gizmo in (("pose", self._pose_gizmo), ("prev", self._prev_gizmo), ("next", self._next_gizmo)):
            gizmo.on_drag_start(lambda _, key=key: self._on_drag_start(key))
            gizmo.on_drag_end(lambda _, key=key: self._on_drag_end(key))
        self._prev_gizmo.on_update(lambda _: self._on_tangent_gizmo(previous=True))
        self._next_gizmo.on_update(lambda _: self._on_tangent_gizmo(previous=False))
        # The path's tracking point (purple sphere, set by coordinates),
        # separate from the servo's.
        self._axes = "tool"
        self._mode_select.on_click(lambda e: self._on_mode(str(e.target.value)))
        self._new_button.on_click(lambda _: self._new_path())
        self._confirm_button.on_click(lambda _: self._confirm())
        self._edit_buttons.on_click(lambda e: self._edit(e.target.value))
        self._auto_plan.on_update(lambda _: self._on_auto_plan())
        self._gizmo_axes.on_click(
            lambda e: self._ui.set_gizmo_axes("base" if "base_link" in str(e.target.value) else "tool")
        )
        self._orient_button.on_click(lambda _: self._orient_to_point())  # the red point (Control › Point tracking)
        self._plan_button.on_click(lambda _: self._plan())
        self._straight_button.on_click(lambda _: self._waypoint_action("Straight"))
        self._wp_select.on_update(lambda _: self._on_select_waypoint())
        self._delete_wp.on_click(lambda _: self._delete_selected_waypoint())
        self._seg_page.on_update(lambda _: self._refresh_segment_table())
        self._apply_offset.on_click(lambda _: self._on_apply_offset())
        self._file_buttons.on_click(lambda e: self._file(e.target.value))
        self._refresh_files()

    # ------------------------------------------------------------- helpers
    def _snapshot(self, merge: bool = False) -> None:
        now = time.monotonic()
        if merge and now - self._last_edit < 0.4:
            self._last_edit = now
            return
        self._last_edit = now
        self._undo.append(([w.copy() for w in self._waypoints], [s.copy() for s in self._segments]))
        del self._undo[:-100]
        self._redo.clear()

    def _changed(self) -> None:
        """Caller holds the lock: redraw and check/plan shortly. A plan made
        before the change no longer matches the drawing."""
        self._dirty = True
        self._failed.clear()
        self._planned = []
        self._changed_at = time.monotonic()
        self._check_due = len(self._waypoints) >= 2
        self._plan_text = ""
        self._run_text = ""
        self._stale_plan = self._ours and not bool(self._auto_plan.value)
        self._draft_checked_key = None

    @staticmethod
    def _wp_label(j: int) -> str:
        return str(j + 1)

    def _groups(self, waypoints: list[tt.Waypoint]) -> list[list[int]]:
        """Waypoints at the same position (e.g. a path back to an earlier
        waypoint) share one sphere and one label."""
        groups: list[list[int]] = []
        for j, wp in enumerate(waypoints):
            for g in groups:
                if np.linalg.norm(waypoints[g[0]].position - wp.position) < SAME_POINT_M:
                    g.append(j)
                    break
            else:
                groups.append([j])
        return groups

    def load_path(self, waypoints: list[tt.Waypoint], segments: list[tt.Segment], name: str) -> None:
        with self._lock:
            self._snapshot()
            self._waypoints = [w.copy() for w in waypoints]
            self._segments = [s.copy() for s in segments]
            self._seg = 0 if segments else None
            self._wp = None
            self._file_name.value = name
            self._fastest = []
            self._changed()
        # Not in servo mode: a loaded path may end far away from the robot.
        self._gizmo_to_end(servo_too=False)
        self._sync_fields()

    # ------------------------------------------------------------------ modes
    def _on_mode(self, clicked: str) -> None:
        self._set_mode("Edit" if "Edit" in clicked else "Draw")

    def _set_mode(self, mode: str) -> None:
        with self._lock:
            self._mode = mode
            if mode == "Draw":
                self._wp = None
            self._dirty = True
            self._draft_key = _REDRAW
        self._ui.set_tcp_gizmo_visible(mode == "Draw")
        self._updating = True
        try:
            self._mode_select.options = _mode_options(mode)
        finally:
            self._updating = False
        self._sync_fields()

    def _on_auto_plan(self) -> None:
        if self._auto_plan.value:
            with self._lock:
                if len(self._waypoints) >= 2:
                    self._check_due = True
                    self._changed_at = 0.0
        else:
            # Off: the preview of the plan made while drawing goes away now;
            # only the check runs until Plan Trajectory.
            self._clear_plan()
            with self._lock:
                self._check_due = len(self._waypoints) >= 2
                self._changed_at = 0.0
                self._dirty = True

    # -------------------------------------------------------------- drawing
    def _new_path(self, reset_gizmo: bool = True) -> None:
        pose = self._ui.robot_tcp_pose()
        if pose is None:
            self._ui.set_result("New Path: no robot pose yet")
            return
        with self._lock:
            self._snapshot()
            self._waypoints = [tt.Waypoint(*pose)]
            self._segments = []
            self._seg, self._wp = None, None
            self._fastest = []
            self._changed()
        self._sync_fields()
        self._clear_plan()
        self._set_mode("Draw")
        if reset_gizmo:
            self._ui.reset_gizmo_to_robot()
            self._ui.set_result("New Path: starts at the robot TCP — move the gizmo and press Confirm Waypoint")

    def _append(self, position: np.ndarray, wxyz: np.ndarray, label: str) -> None:
        with self._lock:
            last = self._waypoints[-1]
            if np.linalg.norm(last.position - position) < 1e-4 and tt.angle_between_wxyz(last.wxyz, wxyz) < 1e-3:
                self._ui.set_result(f"{label}: same pose as the last waypoint")
                return
            self._snapshot()
            self._waypoints.append(tt.Waypoint(position.copy(), wxyz.copy()))
            self._segments.append(tt.Segment())
            self._seg = len(self._segments) - 1
            self._changed()
        self._sync_fields()

    def _confirm(self) -> None:
        pose = self._ui.gizmo_pose()
        if pose is None:
            self._ui.set_result("Confirm: no gizmo pose yet")
            return
        if not self._waypoints:
            self._new_path(reset_gizmo=False)  # the gizmo stays on the pose being confirmed
        self._append(pose[0], pose[1], "Confirm")

    def _edit(self, action: str) -> None:
        clear = False
        with self._lock:
            if action == "Undo" and self._undo:
                self._redo.append(([w.copy() for w in self._waypoints], [s.copy() for s in self._segments]))
                self._waypoints, self._segments = self._undo.pop()
            elif action == "Redo" and self._redo:
                self._undo.append(([w.copy() for w in self._waypoints], [s.copy() for s in self._segments]))
                self._waypoints, self._segments = self._redo.pop()
            elif action == "Clear":
                self._snapshot()
                self._waypoints, self._segments = [], []
                clear = True
            else:
                return
            self._clamp_selection()
            self._fastest = []
            self._changed()
        if clear or len(self._waypoints) < 2:
            self._clear_plan(any_plan=clear)
        if action in ("Undo", "Redo"):
            self._gizmo_to_end()
        self._sync_fields()

    def _gizmo_to_end(self, servo_too: bool = True) -> None:
        """Rule: after Undo, Redo, Delete Waypoint and Load the TCP gizmo is on
        the last waypoint (the end of the path), where drawing continues. In
        servo mode the robot follows the gizmo; servo_too=False leaves it."""
        with self._lock:
            last = self._waypoints[-1].copy() if self._waypoints else None
        if last is None:
            return
        servo = self._ui.servo_status()
        if not servo_too and servo is not None and servo.state == "ACTIVE":
            return
        self._ui.set_gizmo_pose(last.position, last.wxyz)

    def _clear_plan(self, any_plan: bool = False) -> None:
        """Removes the plan (planner, preview, planned path): the editor's own
        plan, or with any_plan whatever plan is shown (Clear)."""
        if self._ours or any_plan:
            self._ui.path_goal_pub.publish(String(data=json.dumps({"type": "clear"})))
            self._ours = False
        with self._lock:
            self._plan_text = ""
            self._planned = []

    def _delete_selected_waypoint(self) -> None:
        with self._lock:
            i = self._wp
            if i is None or i >= len(self._waypoints):
                return
            self._snapshot()
            del self._waypoints[i]
            if self._segments:
                del self._segments[min(i, len(self._segments) - 1)]
            self._wp = None
            self._clamp_selection()
            self._fastest = []
            self._changed()
        if len(self._waypoints) < 2:
            self._clear_plan()
        self._gizmo_to_end()
        self._sync_fields()

    def _clamp_selection(self) -> None:
        if self._seg is not None and self._seg >= len(self._segments):
            self._seg = len(self._segments) - 1 if self._segments else None
        if self._wp is not None and self._wp >= len(self._waypoints):
            self._wp = None

    # ----------------------------------------------------- waypoint editing
    def _click_group(self, group: list[int]) -> None:
        with self._lock:
            mode = self._mode
            last = len(self._waypoints) - 1
        if mode == "Draw":
            # Draw: the gizmo goes to the clicked waypoint; Confirm Waypoint
            # then adds the segment back to it.
            if last in group:
                self._ui.set_result("Draw: that is the last waypoint")
                return
            wp = self._waypoints[group[0]]
            self._ui.set_gizmo_pose(wp.position.copy(), wp.wxyz.copy())
            self._ui.set_result(f"Gizmo on waypoint {self._wp_label(group[0])} — Confirm Waypoint to go back to it")
            return
        with self._lock:
            # Edit: select; clicking a shared point again cycles through it.
            if self._wp in group and len(group) > 1:
                self._wp = group[(group.index(self._wp) + 1) % len(group)]
            else:
                self._wp = group[0]
            if self._segments:
                self._seg = max(0, min(self._wp - 1, len(self._segments) - 1))
            self._dirty = True
        self._sync_fields()

    def _waypoint_action(self, action: str) -> None:
        with self._lock:
            i = self._wp
            if i is None or i >= len(self._waypoints):
                return
            if action == "Straight":
                self._snapshot()
                self._set_tangent(i, np.zeros(3))
                self._changed()
        self._sync_fields()

    def _set_tangent(self, i: int, tangent: np.ndarray) -> None:
        """Caller holds the lock. A zero tangent makes the waypoint straight."""
        wp = self._waypoints[i]
        wp.tangent = np.asarray(tangent, dtype=float)
        wp.curve = bool(np.linalg.norm(wp.tangent) > 1e-6)

    def _handle_positions(self, waypoints: list[tt.Waypoint], i: int):
        """Previous/next tangent handle of waypoint i: waypoint -/+ tangent for
        a curve; 35 % toward the neighbours while straight."""
        wp = waypoints[i]
        prev_pos = next_pos = None
        if wp.curve:
            if i > 0:
                prev_pos = wp.position - wp.tangent
            if i < len(waypoints) - 1:
                next_pos = wp.position + wp.tangent
        else:
            if i > 0:
                prev_pos = wp.position + self.SNAP_FRACTION * (waypoints[i - 1].position - wp.position)
            if i < len(waypoints) - 1:
                next_pos = wp.position + self.SNAP_FRACTION * (waypoints[i + 1].position - wp.position)
        return prev_pos, next_pos

    def _on_pose_gizmo(self) -> None:
        if self._updating:
            return
        with self._lock:
            i = self._wp
            if i is None or i >= len(self._waypoints):
                return
            self._snapshot(merge=True)
            wp = self._waypoints[i]
            wp.position = np.asarray(self._pose_gizmo.position, dtype=float).copy()
            if self._ui.tracking_on():
                # Point tracking: the moved waypoint looks at the point.
                point = self._ui.tracking_point()
                if np.linalg.norm(point - wp.position) > tt.LOOK_AT_MIN_DISTANCE_M:
                    wp.wxyz = tt.matrix_to_quat_wxyz(
                        tt.look_at_rotation(tt.quat_wxyz_to_matrix(wp.wxyz), wp.position, point)
                    )
            else:
                gizmo = tt.quat_wxyz_to_matrix(np.asarray(self._pose_gizmo.wxyz, dtype=float))
                if self._ui.gizmo_axes() == "tool":
                    rotation = gizmo  # the gizmo is the waypoint pose
                else:
                    # base_link axes: the gizmo's rotation since the drag
                    # started (it starts at the base axes) turns the waypoint.
                    reference = self._pose_ref if self._pose_ref is not None else tt.quat_wxyz_to_matrix(wp.wxyz)
                    rotation = gizmo @ reference
                wp.wxyz = tt.matrix_to_quat_wxyz(rotation)
            self._changed()

    def _on_drag_start(self, key: str) -> None:
        self._held.add(key)
        if key == "pose":
            with self._lock:
                i = self._wp
                if i is not None and i < len(self._waypoints):
                    self._pose_ref = tt.quat_wxyz_to_matrix(self._waypoints[i].wxyz)

    def _on_drag_end(self, key: str) -> None:
        self._held.discard(key)
        if key == "pose":
            self._pose_ref = None
        with self._lock:
            self._dirty = True  # show the final pose/handles

    def _on_tangent_gizmo(self, previous: bool) -> None:
        if self._updating:
            return
        with self._lock:
            i = self._wp
            if i is None or i >= len(self._waypoints):
                return
            self._snapshot(merge=True)
            self._dragged_handle, self._drag_time = ("prev" if previous else "next"), time.monotonic()
            self._realign_due = True
            wp = self._waypoints[i]
            handle = np.asarray((self._prev_gizmo if previous else self._next_gizmo).position, dtype=float)
            # The other handle mirrors this one (path tangent through the waypoint).
            self._set_tangent(i, (wp.position - handle) if previous else (handle - wp.position))
            self._changed()

    def _on_select_waypoint(self) -> None:
        if self._updating:
            return
        value = str(self._wp_select.value)
        with self._lock:
            self._wp = int(value) - 1 if value.isdigit() else None
            if self._wp is not None and self._segments:
                self._seg = max(0, min(self._wp - 1, len(self._segments) - 1))
            self._dirty = True
        if self._wp is not None and self._mode != "Edit":
            self._set_mode("Edit")
        self._sync_fields()

    # ------------------------------------------------------- segment table
    SEG_PAGE = 25

    def _on_seg_time(self, k: int, handle) -> None:
        if self._updating:
            return
        value = float(handle.value)
        with self._lock:
            if k >= len(self._segments):
                return
            self._snapshot(merge=True)
            self._segments[k].duration = value if value > 0.0 else None
            self._seg = k  # highlighted in the 3D view
            self._changed()
        self._seg_values[k] = value

    def _seg_label(self, k: int, fastest: float | None, failed: bool) -> str:
        # Short enough for one line in the label column.
        name = f"{k + 1}→{k + 2}"
        if failed:
            return f"{name} ❌"
        return f"{name} ≥{fastest:.2f}s" if fastest is not None else f"{name} …"

    def _refresh_segment_table(self) -> None:
        with self._lock:
            durations = [s.duration for s in self._segments]
            fastest = list(self._fastest)
            failed = set(self._failed)
        n = len(durations)
        pages = [(i, min(i + self.SEG_PAGE, n)) for i in range(0, n, self.SEG_PAGE)]
        names = tuple(f"{a + 1}–{b}" for a, b in pages) or ("—",)
        if tuple(self._seg_page.options) != names:
            self._updating = True
            try:
                self._seg_page.options = names
                self._seg_page.value = names[0]
            finally:
                self._updating = False
        self._seg_page.visible = n > self.SEG_PAGE
        page = names.index(self._seg_page.value) if self._seg_page.value in names else 0
        rows = tuple(range(*pages[page])) if pages else ()
        if rows != self._seg_rows:
            # Rows changed (segments added/removed, other page): rebuild.
            for handle in self._seg_inputs.values():
                handle.remove()
            self._seg_inputs, self._seg_labels, self._seg_values = {}, {}, {}
            with self._seg_folder:
                for k in rows:
                    label = self._seg_label(k, fastest[k] if k < len(fastest) else None, k in failed)
                    value = round(durations[k], 2) if durations[k] else 0.0
                    handle = gui_number(self._ui.server.gui, label, value)
                    handle.on_update(lambda _, k=k, handle=handle: self._on_seg_time(k, handle))
                    self._seg_inputs[k], self._seg_labels[k], self._seg_values[k] = handle, label, value
            self._seg_rows = rows
        self._updating = True
        try:
            for k, handle in self._seg_inputs.items():
                label = self._seg_label(k, fastest[k] if k < len(fastest) else None, k in failed)
                if label != self._seg_labels.get(k):
                    handle.label = label
                    self._seg_labels[k] = label
                value = round(durations[k], 2) if durations[k] else 0.0
                if abs(value - self._seg_values.get(k, -1.0)) > 1e-6:  # changed elsewhere (undo, load)
                    handle.value = value
                    self._seg_values[k] = value
        finally:
            self._updating = False
        too_fast = [
            f"{self._wp_label(k)} → {k + 2}"
            for k in range(min(n, len(fastest)))
            if durations[k] is not None and durations[k] < fastest[k] * 0.999
        ]
        text = "Time per segment [s]; 0 = as fast as possible; ≥ = the fastest possible." if n else "No segments yet."
        if too_fast:
            text += f"  \nFaster than possible (the fastest time is used): {', '.join(too_fast)}"
        if self._seg_info.content != text:
            self._seg_info.content = text

    def _on_apply_offset(self) -> None:
        move_mm = np.asarray(self._move_by.value, dtype=float)
        rotate_deg = np.asarray(self._rotate_by.value, dtype=float)
        move = move_mm / 1000.0
        rotate = pin.rpy.rpyToMatrix(*np.radians(rotate_deg))
        applied = (
            f"{', '.join(f'{v:g}' for v in move_mm)} mm, {', '.join(f'{v:g}' for v in rotate_deg)}° "
            f"({self._offset_frame.value})"
        )
        with self._lock:
            target_wp = self._wp if self._mode == "Edit" else None
        if target_wp is not None:
            with self._lock:
                wp = self._waypoints[target_wp]
                position, rotation = wp.position, tt.quat_wxyz_to_matrix(wp.wxyz)
        else:
            pose = self._ui.gizmo_pose()
            if pose is None:
                return
            position, rotation = pose[0], tt.quat_wxyz_to_matrix(pose[1])
        if self._offset_frame.value == "local":
            new_position, new_rotation = position + rotation @ move, rotation @ rotate
        else:
            new_position, new_rotation = position + move, rotate @ rotation
        if target_wp is not None:
            with self._lock:
                self._snapshot()
                self._waypoints[target_wp].position = new_position
                self._waypoints[target_wp].wxyz = tt.matrix_to_quat_wxyz(new_rotation)
                self._changed()
            self._ui.set_result(f"Offset {applied} applied to waypoint {self._wp_label(target_wp)}")
        else:
            self._ui.set_gizmo_pose(new_position, tt.matrix_to_quat_wxyz(new_rotation))
            self._ui.set_result(f"Offset {applied} applied to the TCP gizmo")

    def _sync_fields(self) -> None:
        with self._lock:
            n_wp = len(self._waypoints)
            i = self._wp
            wp = self._waypoints[i] if i is not None and i < n_wp else None
        self._updating = True
        try:
            self._wp_select.options = tuple(self._wp_label(j) for j in range(n_wp)) or ("—",)
            self._wp_select.value = self._wp_label(i) if wp is not None else "—"
        finally:
            self._updating = False

    # -------------------------------------------------------------- planning
    def _request(self, waypoints: list[tt.Waypoint] | None = None, segments: list[tt.Segment] | None = None) -> dict:
        waypoints = self._waypoints if waypoints is None else waypoints
        segments = self._segments if segments is None else segments
        point = self.look_at_point()
        return {
            "name": tt.safe_name(str(self._file_name.value), "path"),
            "waypoints": [w.to_dict() for w in waypoints],
            "segments": [s.to_dict() for s in segments],
            "look_at_m": None if point is None else [float(v) for v in point],
        }

    # -------------------------------------------------------- point tracking
    def look_at_point(self) -> np.ndarray | None:
        """Paths are planned with the waypoints' own orientations (the point
        only turns waypoints on request: Orient to point)."""
        return None

    def on_gizmo_axes(self, mode: str) -> None:
        """Gizmo axes changed (here, in the Control tab, or by point tracking)."""
        self._axes = mode
        self._gizmo_axes.options = axes_options(mode if not self._ui.tracking_on() else "base")
        with self._lock:
            self._dirty = True  # re-place the waypoint gizmo with the new axes

    def _orient_to_point(self) -> None:
        """Turns the tool z axis of the selected waypoint (Edit) or of the TCP
        gizmo (Draw) toward the point, by the smallest rotation."""
        point = self._ui.tracking_point()
        with self._lock:
            i = self._wp if self._mode == "Edit" else None
            if i is not None and i < len(self._waypoints):
                wp = self._waypoints[i]
                if np.linalg.norm(point - wp.position) < tt.LOOK_AT_MIN_DISTANCE_M:
                    self._ui.set_result("Orient to point: the waypoint is at the point")
                    return
                self._snapshot()
                wp.wxyz = tt.matrix_to_quat_wxyz(
                    tt.look_at_rotation(tt.quat_wxyz_to_matrix(wp.wxyz), wp.position, point)
                )
                self._changed()
                label = self._wp_label(i)
            else:
                label = None
        if label is not None:
            self._ui.set_result(f"Orient to point: waypoint {label} looks at the point")
            return
        pose = self._ui.gizmo_pose()
        if pose is None:
            return
        rotation = tt.look_at_rotation(tt.quat_wxyz_to_matrix(pose[1]), pose[0], point)
        self._ui.set_gizmo_pose(pose[0], tt.matrix_to_quat_wxyz(rotation))
        self._ui.set_result("Orient to point: the TCP gizmo looks at the point")

    def _shown_waypoints(self, waypoints: list[tt.Waypoint]) -> list[tt.Waypoint]:
        return waypoints

    def _send_check(self) -> None:
        with self._lock:
            if len(self._waypoints) < 2:
                return
            self._check_due = False
            self._request_id += 1
            request = dict(self._request(), request_id=f"path-{self._request_id}")
        self._ui.path_check_pub.publish(String(data=json.dumps(request)))

    def _plan(self, automatic: bool = False) -> None:
        if not automatic and not self._ui.plan_mode_ready("Plan Trajectory"):
            return
        with self._lock:
            if len(self._waypoints) < 2:
                if not automatic:
                    self._ui.set_result("Plan Trajectory: confirm at least one waypoint")
                return
            request = dict(self._request(), type="cartesian_path")
            self._plan_text = "⏳"
            self._check_due = False
            self._last_plan_request = time.monotonic()
            self._dirty = True
        self._ours = True
        self._ui.path_goal_pub.publish(String(data=json.dumps(request)))
        if not automatic:
            self._ui.set_result(f"Plan Trajectory: {len(request['waypoints'])} waypoints sent to the planner")

    def _maybe_check_draft(self) -> None:
        """While drawing: is the pose at the gizmo reachable from the last
        waypoint? The draft line turns red before Confirm if not."""
        if self._mode != "Draw":
            return
        gizmo = self._ui.gizmo_pose()
        with self._lock:
            if gizmo is None or not self._waypoints:
                return
            key = (tuple(np.round(gizmo[0], 4)), tuple(np.round(gizmo[1], 4)), len(self._waypoints))
            now = time.monotonic()
            if key == self._draft_checked_key or now - self._draft_sent_at < self.DRAFT_CHECK_PERIOD_S:
                return
            last = self._waypoints[-1]
            if np.linalg.norm(last.position - gizmo[0]) < 1e-4 and tt.angle_between_wxyz(last.wxyz, gizmo[1]) < 1e-3:
                self._draft_checked_key = key
                self._draft_failed = False
                return
            self._draft_checked_key = key
            self._draft_sent_at = now
            self._draft_id += 1
            waypoints = self._waypoints + [tt.Waypoint(gizmo[0].copy(), gizmo[1].copy())]
            segments = self._segments + [tt.Segment()]
            request = dict(self._request(waypoints, segments), request_id=f"draft-{self._draft_id}")
        self._ui.path_check_pub.publish(String(data=json.dumps(request)))

    def on_check(self, result: dict) -> None:
        rid = str(result.get("request_id"))
        with self._lock:
            if rid == f"draft-{self._draft_id}":
                draft_segment = len(self._segments)
                failed = {int(k) for k in result.get("failed_segments", [])}
                was = self._draft_failed
                self._draft_failed = (not result.get("ok")) and draft_segment in failed
                if was != self._draft_failed:
                    self._draft_key = _REDRAW  # redraw the line in its new colour
                return
            if rid != f"path-{self._request_id}":
                return  # answer to an older drawing
            if result.get("ok"):
                self._failed = set()
                self._fastest = list(result.get("segment_fastest_s", []))
            else:
                self._failed = {int(k) for k in result.get("failed_segments", [])}
                where = ", ".join(f"{self._wp_label(k)} → {k + 2}" for k in sorted(self._failed))
                self._plan_text = f"❌ {_short(result.get('message', ''))}" + (f" (segment {where})" if where else "")
            self._dirty = True

    def on_plan_status(self, status: dict) -> None:
        kind = status.get("kind")
        state = status.get("state")
        if kind in ("joint_goal", "joint_path") and state in ("planning", "planned"):
            self._ours = False  # another plan (Plan, Home, replay) replaced ours
            return
        if kind != "cartesian_path":
            return
        with self._lock:
            if state == "failed":
                self._failed = {int(k) for k in status.get("failed_segments", [])}
                where = ", ".join(f"{self._wp_label(k)} → {k + 2}" for k in sorted(self._failed))
                self._plan_text = f"❌ {_short(status.get('message', ''))}" + (f" (segment {where})" if where else "")
            elif state == "planned":
                self._failed.clear()
                self._fastest = list(status.get("segment_fastest_s", self._fastest))
                self._planned = list(status.get("segment_planned_s", []))
                self._plan_text = "✅"
            elif state == "succeeded":
                self._plan_text = ""  # the plan was used; planned again while drawing
            tracking = status.get("tracking")
            if tracking and state in ("succeeded", "failed"):
                self._run_text = (
                    f"*Last run: {tracking['max_deviation_mm']:.1f} mm max from the plan"
                    + (" ⚠️ (details in Control)" if tracking["max_deviation_mm"] > 5.0 else "")
                    + "*"
                )
            self._dirty = True

    def _robot_resting_elsewhere(self) -> bool:
        q = self._ui.robot_q()
        if q is None:
            return False
        now = time.monotonic()
        if self._last_q is None or np.max(np.abs(q - self._last_q)) > 1e-4:
            self._last_motion = now
        self._last_q = q
        if now - self._last_motion < 0.5:
            return False
        start = self._ui.planned_start()
        return start is not None and np.max(np.abs(q - start)) > 0.003

    # ----------------------------------------------------------------- files
    def _refresh_files(self) -> None:
        files = list_json(self._dir)
        self._files.options = tuple(files) if files else ("—",)

    def _file(self, action: str) -> None:
        if action == "Refresh":
            self._refresh_files()
            return
        if action == "Save":
            with self._lock:
                if len(self._waypoints) < 2:
                    self._ui.set_result("Save: the path has no segment yet")
                    return
                name = tt.safe_name(str(self._file_name.value), "path")
                payload = tt.path_payload(name, self._waypoints, self._segments)
            path = tt.write_json(self._dir / f"{name}.json", payload)
            self._refresh_files()
            self._files.value = path.name
            self._ui.set_result(f"Save: {path.name} ({len(payload['waypoints'])} waypoints)")
            return
        name = str(self._files.value)
        if name == "—":
            return
        try:
            payload = json.loads((self._dir / name).read_text(encoding="utf-8"))
            waypoints, segments = tt.path_from_payload(payload)
            point = tt.look_at_from_payload(payload)
        except Exception as exc:  # noqa: BLE001
            self._ui.set_result(f"Load: {name} failed ({exc})")
            return
        if point is not None:
            # Older file planned to look at a point: the waypoints get the
            # orientation they had along that path.
            waypoints = tt.apply_look_at(waypoints, point)
        self.load_path(waypoints, segments, Path(name).stem)
        self._ui.set_result(
            f"Load: {name} ({len(waypoints)} waypoints" + (", looks at a point)" if point is not None else ")")
        )

    # --------------------------------------------------------------- drawing
    def tick(self) -> None:
        with self._lock:
            if self._realign_due and time.monotonic() - self._drag_time >= self.DRAG_SETTLE_S:
                self._realign_due = False
                self._dirty = True  # the dragged handle gets its axis back on the line
            due = self._check_due and time.monotonic() - self._changed_at >= self.CHECK_DELAY_S
            stale, self._stale_plan = self._stale_plan, False
        if stale:
            self._clear_plan()  # the plan no longer matches the drawing
        auto = bool(self._auto_plan.value)
        if due:
            # Plan while drawing: the plan also reports failing segments and
            # fastest times; otherwise only the check runs.
            self._plan(automatic=True) if auto else self._send_check()
        elif auto and self._ours:
            servo = self._ui.servo_status()
            plan = self._ui.plan_status()
            idle = servo is None or servo.state in ("DISABLED", "HOLD")
            if (
                idle and plan.get("state") in ("planned", "succeeded")
                and time.monotonic() - self._last_plan_request > 1.5 and self._robot_resting_elsewhere()
            ):
                self._plan(automatic=True)  # the robot moved since planning: new approach
        self._maybe_check_draft()
        self._draw_draft()
        with self._lock:
            if not self._dirty:
                return
            self._dirty = False
            waypoints = [w.copy() for w in self._waypoints]
            k, i, failed, mode = self._seg, self._wp, set(self._failed), self._mode
        waypoints = self._shown_waypoints(waypoints)
        scene = self._ui.server.scene
        # Children (labels, numbered spheres) were added after their parent
        # frames: remove in reverse order.
        for handle in reversed(self._handles):
            handle.remove()
        self._handles = []
        for g, group in enumerate(self._groups(waypoints)):
            first = waypoints[group[0]]
            frame = scene.add_frame(
                f"/fairino/path/wp_{g}", wxyz=tuple(first.wxyz), position=tuple(first.position),
                axes_length=0.04, axes_radius=0.002,
            )
            selected = i in group
            color = COLOR_SELECTED if selected else (240, 240, 240) if 0 in group else (40, 130, 230)
            sphere = scene.add_icosphere(f"/fairino/path/wp_{g}/dot", radius=0.011, color=color)
            sphere.on_click(lambda _, members=list(group): self._click_group(members))
            self._handles += [sphere, frame]
            if len(group) == 1:
                self._handles.append(scene.add_label(
                    f"/fairino/path/wp_{g}/label", self._wp_label(group[0]), position=(0.0, 0.0, 0.03)
                ))
            if len(group) > 1:
                # Shared point: one small numbered sphere per waypoint, in a
                # row off the point (the waypoint gizmo has no handles in its
                # negative octant), so each one can be picked directly.
                direction = -np.ones(3) / math.sqrt(3.0)
                chips = [direction * (self.CHIP_START_M + self.CHIP_STEP_M * n) for n in range(len(group))]
                self._handles.append(scene.add_line_segments(
                    f"/fairino/path/wp_{g}/chip_line", points=np.array([[np.zeros(3), chips[-1]]]),
                    colors=(150, 150, 150), thickness=0.0015,
                ))
                for n, j in enumerate(group):
                    chip = scene.add_icosphere(
                        f"/fairino/path/wp_{g}/chip_{j}", radius=0.008, position=tuple(chips[n]),
                        color=COLOR_SELECTED if j == i else (40, 130, 230),
                    )
                    chip.on_click(lambda _, member=j: self._click_group([member]))
                    chip_label = scene.add_label(
                        f"/fairino/path/wp_{g}/chip_{j}/label", self._wp_label(j), position=(0.0, 0.0, 0.015)
                    )
                    self._handles += [chip, chip_label]
        for j in range(len(waypoints) - 1):
            p0, p3 = waypoints[j].position, waypoints[j + 1].position
            cp = tt.control_points(waypoints, j)
            c1, c2 = cp if cp is not None else (p0 + (p3 - p0) / 3.0, p0 + 2.0 * (p3 - p0) / 3.0)
            color = COLOR_FAILED if j in failed else COLOR_SELECTED if j == k else COLOR_SEGMENT
            self._handles.append(
                scene.add_spline_cubic_bezier(
                    f"/fairino/path/segment_{j}",
                    points=np.array([p0, p3]),
                    control_points=np.array([c1, c2]),
                    thickness=0.006 if j == k or j in failed else 0.004,
                    color=color,
                    segments=64,
                )
            )
        self._updating = True
        try:
            editing = mode == "Edit" and i is not None and i < len(waypoints)
            self._pose_gizmo.visible = editing
            base_axes = self._ui.gizmo_axes() == "base"
            tracking = self._ui.tracking_on()  # the point decides the orientation
            if self._pose_gizmo.disable_rotations != tracking:
                self._pose_gizmo.disable_rotations = tracking
            prev_pos = next_pos = None
            if editing:
                wp = waypoints[i]
                if "pose" not in self._held:
                    self._pose_gizmo.position = tuple(wp.position)
                    self._pose_gizmo.wxyz = (1.0, 0.0, 0.0, 0.0) if base_axes else tuple(wp.wxyz)
                prev_pos, next_pos = self._handle_positions(waypoints, i)
                segments_lines = [[wp.position, p] for p in (prev_pos, next_pos) if p is not None]
                if segments_lines:
                    self._handles.append(
                        scene.add_line_segments(
                            "/fairino/path/handles", points=np.array(segments_lines), colors=COLOR_HANDLE, thickness=0.002
                        )
                    )
            self._prev_gizmo.visible = prev_pos is not None
            self._next_gizmo.visible = next_pos is not None
            dragging = self._dragged_handle if time.monotonic() - self._drag_time < self.DRAG_SETTLE_S else None
            for name, gizmo, pos in (("prev", self._prev_gizmo, prev_pos), ("next", self._next_gizmo, next_pos)):
                if pos is None or name in self._held:
                    continue
                gizmo.position = tuple(pos)
                if name != dragging:
                    # One gizmo axis along the line waypoint -> handle; not
                    # while that handle is being dragged.
                    gizmo.wxyz = tuple(_align_axis(np.asarray(gizmo.wxyz, dtype=float), pos - waypoints[i].position))
        finally:
            self._updating = False

    def _draw_draft(self) -> None:
        """Light green line from the last waypoint (or the robot TCP) to the
        gizmo in Draw mode; red when that pose is not reachable."""
        gizmo = self._ui.gizmo_pose() if self._mode == "Draw" else None
        with self._lock:
            start = self._waypoints[-1].position.copy() if self._waypoints else None
            failed = self._draft_failed
        if start is None and gizmo is not None:
            robot = self._ui.robot_tcp_pose()
            start = None if robot is None else robot[0]
        key = None if gizmo is None or start is None else (tuple(np.round(start, 5)), tuple(np.round(gizmo[0], 5)), failed)
        # Nothing to draw (Edit mode, no gizmo) always removes the line; the
        # key only skips redrawing an unchanged line. (Resetting the key to
        # None used to equal "nothing to draw" and left the line behind.)
        if key == self._draft_key and (key is not None or self._draft is None):
            return
        self._draft_key = key
        if self._draft is not None:
            self._draft.remove()
            self._draft = None
        if key is not None and np.linalg.norm(gizmo[0] - start) > 1e-4:
            self._draft = self._ui.server.scene.add_line_segments(
                "/fairino/path/draft",
                points=np.array([[start, gizmo[0]]]),
                colors=COLOR_FAILED if failed else COLOR_DRAFT,
                thickness=0.004,
            )

    def refresh(self, plan_mode: bool) -> None:
        with self._lock:
            n_wp, n_seg = len(self._waypoints), len(self._segments)
            i = self._wp
            wp = self._waypoints[i] if i is not None and i < n_wp else None
            plan_text = self._plan_text
            run_text = self._run_text
            mode = self._mode
            curves = sum(1 for w in self._waypoints if w.curve)
            total = sum(self._planned) if len(self._planned) == n_seg and n_seg else None
            length = sum(
                float(np.sum(np.linalg.norm(np.diff(tt.segment_points(self._waypoints, j, 0.01)[0], axis=0), axis=1)))
                for j in range(n_seg)
            )
            table = [(self._wp_label(j), w.position * 1000.0) for j, w in enumerate(self._waypoints)]
        if n_wp == 0:
            self._summary.content = "Press **New Path** (starts at the robot TCP), move the TCP gizmo and press **Confirm Waypoint**."
        else:
            # ✅ planned and executable, ❌ cannot be executed (reason below),
            # ⏳ planning, ⚪ not planned yet.
            icon = plan_text[:1] if plan_text else ("⚪" if n_seg else "")
            reason = plan_text[1:].strip() if icon == "❌" else ""
            self._summary.content = (
                f"{icon} **{n_wp} waypoints · {n_seg} segments** · {curves} curve(s) · {length * 1000.0:.0f} mm"
                + (f" · {total:.2f} s" if total is not None and icon == "✅" else "")
                + (f"  \n{reason}" if reason else "")
                + (f"  \n{run_text}" if run_text else "")
            )
        self._refresh_segment_table()
        # Coordinates (collapsed folder): kept short, base_link frame.
        gizmo = self._ui.gizmo_pose()
        lines = []
        if gizmo is not None:
            lines.append(f"Gizmo: {_pose_text(gizmo[0], gizmo[1])}")
        if wp is not None:
            lines.append(f"Waypoint {self._wp_label(i)}: {_pose_text(wp.position, wp.wxyz)}")
        self._coord_info.content = "  \n".join(lines) + "  \n*Offset target: " + (
            f"waypoint {self._wp_label(i)}*" if mode == "Edit" and wp is not None else "TCP gizmo*"
        )
        if table:
            rows = ["| # | x | y | z [mm] |", "|---|---:|---:|---:|"]
            rows += [f"| {name} | {p[0]:.1f} | {p[1]:.1f} | {p[2]:.1f} |" for name, p in table[:40]]
            self._waypoint_table.content = "\n".join(rows)
        else:
            self._waypoint_table.content = ""
        self._confirm_button.disabled = mode != "Draw"
        tracking = self._ui.tracking_on()
        for handle in (self._gizmo_axes, self._orient_button):  # point tracking decides the orientation
            if handle.visible == tracking:
                handle.visible = not tracking
        self._delete_wp.disabled = wp is None
        self._straight_button.disabled = wp is None or not wp.curve
        self._plan_button.disabled = not plan_mode or n_wp < 2
